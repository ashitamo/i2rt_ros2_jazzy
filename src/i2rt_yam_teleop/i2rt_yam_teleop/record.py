#!/usr/bin/env python3
"""Record YAM teleoperation state and camera topics into trajectory folders."""

import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool

from i2rt_msgs.msg import GripperState


class RecordData(Node):
    """
    Topics that we want to record.
    """
    LEADER_GRIPPER_TOPIC = "/yam_leader/gripper_state"
    LEADER_JOINT_TOPIC = "/yam_leader/joint_states"
    FOLLOWER_GRIPPER_TOPIC = "/yam_follower/gripper_state"
    FOLLOWER_JOINT_TOPIC = "/yam_follower/joint_states"
    COLOR_TOPIC = "/camera/camera/color/image_raw/compressed"
    DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw/compressedDepth"
    INFRA_TOPIC = "/camera/camera/infra1/image_rect_raw"
    SYNC_TOPIC = "/leader_follower/synchronized"

    def __init__(self) -> None:
        super().__init__("record_data")

        self.declare_parameter("output_directory", "recordings")
        self.declare_parameter("jpeg_quality", 95)
        self.output_root = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        if not self.output_root.is_absolute():
            self.output_root = Path.cwd() / self.output_root
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        self.episode_number = self._latest_trajectory_number()
        self.episode_start_ns: int | None = None
        self.session_directory: Path | None = None
        self.color_directory: Path | None = None
        self.depth_directory: Path | None = None
        self.infra_directory: Path | None = None
        self._open_files: list[Any] = []
        self._csv_writers: dict[str, Any] = {}

        self.bridge = CvBridge()
        self.latest_messages: dict[str, Any] = {}
        self.receive_times_ns: dict[str, int] = {}
        self.message_counts: dict[str, int] = {}
        self.recording_enabled = False
        self._topic_subscriptions = []

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sync_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        subscriptions = (
            (GripperState, self.LEADER_GRIPPER_TOPIC, self._leader_gripper_callback, state_qos),
            (JointState, self.LEADER_JOINT_TOPIC, self._leader_joint_callback, state_qos),
            (GripperState, self.FOLLOWER_GRIPPER_TOPIC, self._follower_gripper_callback, state_qos),
            (JointState, self.FOLLOWER_JOINT_TOPIC, self._follower_joint_callback, state_qos),
            (CompressedImage, self.COLOR_TOPIC, self._color_callback, qos_profile_sensor_data),
            (CompressedImage, self.DEPTH_TOPIC, self._depth_callback, qos_profile_sensor_data),
            (Image, self.INFRA_TOPIC, self._infra_callback, qos_profile_sensor_data),
        )
        for message_type, topic, callback, qos in subscriptions:
            self._topic_subscriptions.append(
                self.create_subscription(message_type, topic, callback, qos)
            )
        self._topic_subscriptions.append(
            self.create_subscription(
                Bool, self.SYNC_TOPIC, self._synchronization_callback, sync_qos
            )
        )

        self.get_logger().info(
            f"Waiting for leader/follower synchronization; episode root: "
            f"{self.output_root}"
        )

    def _synchronization_callback(self, message: Bool) -> None:
        if message.data and not self.recording_enabled:
            self._start_episode()
        elif not message.data and self.recording_enabled:
            self._finish_episode("synchronization_ended")

    def _latest_trajectory_number(self) -> int:
        """Return the highest trajectory number already in the output root."""
        trajectory_numbers = []
        for path in self.output_root.iterdir():
            prefix, separator, suffix = path.name.partition("_")
            if path.is_dir() and prefix == "traj" and separator and suffix.isdigit():
                trajectory_numbers.append(int(suffix))
        return max(trajectory_numbers, default=0)

    def _start_episode(self) -> None:
        self.episode_number += 1
        episode_name = f"traj_{self.episode_number}"
        self.session_directory = self.output_root / episode_name
        self.color_directory = self.session_directory / "camera_color"
        self.depth_directory = self.session_directory / "camera_depth"
        self.infra_directory = self.session_directory / "camera_infra"
        self.session_directory.mkdir(parents=False, exist_ok=False)
        for directory in (
            self.color_directory,
            self.depth_directory,
            self.infra_directory,
        ):
            directory.mkdir(exist_ok=False)

        self._open_files = []
        self._csv_writers = {}
        self.latest_messages.clear()
        self.receive_times_ns.clear()
        self.message_counts.clear()
        self.episode_start_ns = self.get_clock().now().nanoseconds
        self._create_csv_files()
        self.recording_enabled = True
        self.get_logger().info(
            f"Leader and follower synchronized; episode {self.episode_number} "
            f"started: {self.session_directory}"
        )

    def _finish_episode(self, reason: str) -> None:
        if not self.recording_enabled or self.session_directory is None:
            return
        self.recording_enabled = False
        end_ns = self.get_clock().now().nanoseconds
        self.close_files()
        metadata = {
            "episode_number": self.episode_number,
            "start_timestamp_ns": self.episode_start_ns,
            "end_timestamp_ns": end_ns,
            "duration_seconds": (
                (end_ns - self.episode_start_ns) * 1e-9
                if self.episode_start_ns is not None
                else None
            ),
            "termination_reason": reason,
            "message_counts": self.message_counts,
        }
        metadata_path = self.session_directory / "episode_metadata.json"
        with metadata_path.open("w", encoding="utf-8") as metadata_file:
            json.dump(metadata, metadata_file, indent=2, sort_keys=True)
            metadata_file.write("\n")
        self.get_logger().info(
            f"Episode {self.episode_number} ended and saved: "
            f"{self.session_directory}"
        )
        self.episode_start_ns = None
        self._csv_writers = {}

    def _create_writer(self, filename: str, fieldnames: list[str]) -> Any:
        if self.session_directory is None:
            raise RuntimeError("Cannot create an episode file before synchronization")
        csv_file = (self.session_directory / filename).open(
            "w", encoding="utf-8", newline="", buffering=1
        )
        self._open_files.append(csv_file)
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        return writer

    def _create_csv_files(self) -> None:
        timestamp_fields = [
            "message_index", "source_timestamp_ns", "receive_timestamp_ns"
        ]
        gripper_fields = timestamp_fields + [
            "position", "velocity", "force", "is_moving", "is_blocked",
            "is_calibrated", "gripper_type",
        ]
        joint_fields = timestamp_fields + [
            "joint_index", "joint_name", "position", "velocity", "effort"
        ]
        image_fields = timestamp_fields + [
            "filename", "encoding", "width", "height"
        ]

        self._csv_writers["leader_gripper"] = self._create_writer(
            "yam_leader_gripper_state.csv", gripper_fields
        )
        self._csv_writers["leader_joint"] = self._create_writer(
            "yam_leader_joint_states.csv", joint_fields
        )
        self._csv_writers["follower_gripper"] = self._create_writer(
            "yam_follower_gripper_state.csv", gripper_fields
        )
        self._csv_writers["follower_joint"] = self._create_writer(
            "yam_follower_joint_states.csv", joint_fields
        )
        self._csv_writers["color"] = self._create_writer(
            "camera_color.csv", image_fields
        )
        self._csv_writers["depth"] = self._create_writer(
            "camera_depth.csv", image_fields
        )
        self._csv_writers["infra"] = self._create_writer(
            "camera_infra.csv", image_fields
        )

    def _remember(self, topic: str, message: Any) -> tuple[int, int, int]:
        receive_ns = self.get_clock().now().nanoseconds
        stamp = message.header.stamp
        source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        message_index = self.message_counts.get(topic, 0) + 1
        self.latest_messages[topic] = message
        self.receive_times_ns[topic] = receive_ns
        self.message_counts[topic] = message_index
        return message_index, source_ns, receive_ns

    @staticmethod
    def _timestamp_row(
        message_index: int, source_ns: int, receive_ns: int
    ) -> dict[str, int]:
        return {
            "message_index": message_index,
            "source_timestamp_ns": source_ns,
            "receive_timestamp_ns": receive_ns,
        }

    def _write_gripper(
        self, key: str, topic: str, message: GripperState
    ) -> None:
        message_index, source_ns, receive_ns = self._remember(topic, message)
        row: dict[str, Any] = self._timestamp_row(
            message_index, source_ns, receive_ns
        )
        row.update(
            position=message.position,
            velocity=message.velocity,
            force=message.force,
            is_moving=message.is_moving,
            is_blocked=message.is_blocked,
            is_calibrated=message.is_calibrated,
            gripper_type=message.gripper_type,
        )
        self._csv_writers[key].writerow(row)

    def _write_joints(self, key: str, topic: str, message: JointState) -> None:
        message_index, source_ns, receive_ns = self._remember(topic, message)
        timestamp_row = self._timestamp_row(message_index, source_ns, receive_ns)
        joint_count = max(
            len(message.name), len(message.position), len(message.velocity),
            len(message.effort), 1,
        )
        for index in range(joint_count):
            self._csv_writers[key].writerow(
                {
                    **timestamp_row,
                    "joint_index": index,
                    "joint_name": message.name[index] if index < len(message.name) else "",
                    "position": message.position[index] if index < len(message.position) else "",
                    "velocity": message.velocity[index] if index < len(message.velocity) else "",
                    "effort": message.effort[index] if index < len(message.effort) else "",
                }
            )

    def _leader_gripper_callback(self, message: GripperState) -> None:
        if not self.recording_enabled:
            return
        self._write_gripper("leader_gripper", self.LEADER_GRIPPER_TOPIC, message)

    def _leader_joint_callback(self, message: JointState) -> None:
        if not self.recording_enabled:
            return
        self._write_joints("leader_joint", self.LEADER_JOINT_TOPIC, message)

    def _follower_gripper_callback(self, message: GripperState) -> None:
        if not self.recording_enabled:
            return
        self._write_gripper("follower_gripper", self.FOLLOWER_GRIPPER_TOPIC, message)

    def _follower_joint_callback(self, message: JointState) -> None:
        if not self.recording_enabled:
            return
        self._write_joints("follower_joint", self.FOLLOWER_JOINT_TOPIC, message)

    def _image_basename(
        self, topic: str, message: Any
    ) -> tuple[str, int, int, int]:
        message_index, source_ns, receive_ns = self._remember(topic, message)
        timestamp_ns = source_ns if source_ns > 0 else receive_ns
        return f"{timestamp_ns}_{message_index:08d}", message_index, source_ns, receive_ns

    def _write_image_metadata(
        self, key: str, message_index: int, source_ns: int, receive_ns: int,
        path: Path, encoding: str, width: int, height: int,
    ) -> None:
        if self.session_directory is None:
            raise RuntimeError("No active recording episode")
        row: dict[str, Any] = self._timestamp_row(
            message_index, source_ns, receive_ns
        )
        row.update(
            filename=str(path.relative_to(self.session_directory)),
            encoding=encoding,
            width=width,
            height=height,
        )
        self._csv_writers[key].writerow(row)

    def _color_callback(self, message: CompressedImage) -> None:
        if not self.recording_enabled:
            return
        if self.color_directory is None:
            raise RuntimeError("Color directory is not initialized")
        basename, index, source_ns, receive_ns = self._image_basename(
            self.COLOR_TOPIC, message
        )
        image = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if image is None:
            self.get_logger().error("Failed to decode a compressed color frame")
            return
        path = self.color_directory / f"{basename}.jpg"
        if not cv2.imwrite(
            str(path), image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ):
            self.get_logger().error(f"Failed to save color frame: {path}")
            return
        height, width = image.shape[:2]
        self._write_image_metadata(
            "color", index, source_ns, receive_ns, path, message.format, width, height
        )

    def _depth_callback(self, message: CompressedImage) -> None:
        # self.get_logger().info(
        #     f"[DEPTH] callback fired: width={message.width}, "
        #     f"height={message.height}, encoding={message.encoding}, "
        #     f"stamp={message.header.stamp.sec}.{message.header.stamp.nanosec}"
        # )
        if not self.recording_enabled:
            # self.get_logger().warn("[DEPTH] callback fired but recording_enabled=False, skipping")
            return
        self.get_logger().info("[DEPTH] recording_enabled=True, proceeding to save")
        if self.depth_directory is None:
            raise RuntimeError("Depth directory is not initialized")
        basename, index, source_ns, receive_ns = self._image_basename(
            self.DEPTH_TOPIC, message
        )
        # compressedDepth prepends a transport header before the PNG payload.
        png_signature = b"\x89PNG\r\n\x1a\n"
        png_offset = message.data.find(png_signature)
        if png_offset < 0:
            self.get_logger().error("Compressed depth frame has no PNG payload")
            return
        image = cv2.imdecode(
            np.frombuffer(message.data[png_offset:], dtype=np.uint8),
            cv2.IMREAD_UNCHANGED,
        )
        if image is None:
            self.get_logger().error("Failed to decode a compressed depth frame")
            return

        # NPY preserves the array's exact dtype, shape, and values.
        path = self.depth_directory / f"{basename}.npy"
        try:
            np.save(path, image, allow_pickle=False)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Failed to save depth frame {path}: {error}")
            return
        self._write_image_metadata(
            "depth", index, source_ns, receive_ns, path, message.format,
            image.shape[1], image.shape[0],
        )

    def _infra_callback(self, message: Image) -> None:
        if not self.recording_enabled:
            return
        if self.infra_directory is None:
            raise RuntimeError("Infrared directory is not initialized")
        basename, index, source_ns, receive_ns = self._image_basename(
            self.INFRA_TOPIC, message
        )
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to convert infrared frame: {error}")
            return

        if image.dtype == np.uint8:
            path = self.infra_directory / f"{basename}.jpg"
            saved = cv2.imwrite(
                str(path), image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
        else:
            path = self.infra_directory / f"{basename}.png"
            saved = cv2.imwrite(str(path), image)
        if not saved:
            self.get_logger().error(f"Failed to save infrared frame: {path}")
            return
        self._write_image_metadata(
            "infra", index, source_ns, receive_ns, path, message.encoding,
            message.width, message.height,
        )

    def close_files(self) -> None:
        for csv_file in self._open_files:
            if not csv_file.closed:
                csv_file.flush()
                csv_file.close()
        self._open_files = []

    def finish_active_episode(self) -> None:
        if self.recording_enabled:
            self._finish_episode("node_shutdown")
        else:
            self.close_files()

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RecordData()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish_active_episode()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
