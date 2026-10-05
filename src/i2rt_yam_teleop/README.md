# YAM Leader–Follower Teleoperation

This package provides dedicated ROS 2 communication for a YAM leader and
follower. It follows the behavior of `minimum_gello.py` without launching
MoveIt, Servo, RViz, or motion-planning controllers.

## System overview

The launch file starts two hardware interfaces and one teleoperation
coordinator:

```text
                       demo_launch_teleop.py
                                  │
                ┌─────────────────┼─────────────────┐
                ▼                 ▼                 ▼
        Leader hardware    Follower hardware    Teleop coordinator
             can1                can0          leader_follower_teleop.py
                │                 │                 │
                └────── ROS 2 topics connect them ──┘
```

The hardware nodes translate between physical CAN devices and ROS 2 messages.
The coordinator contains the synchronization and bilateral-control logic.

## Important Python files

### `demo_launch_teleop.py`

Location:

```text
move_it_yam/launch/demo_launch_teleop.py
```

This is the entry point. It launches:

1. `teleop_hardware_interface` as `yam_leader` on `can1`.
2. `teleop_hardware_interface` as `yam_follower` on `can0`.
3. `leader_follower_teleop`, which coordinates both robots.

It also supplies parameters such as:

- Follower gripper type
- Bilateral `kp` scale
- Slow synchronization duration
- Teleoperation command frequency

The launch file only starts and configures the nodes. It does not run the
teleoperation control loop itself.

### `teleop_hardware_interface.py`

This class extends the existing `YAMHardwareInterface` without modifying it.
The parent interface still handles:

- CAN connection
- Motor commands
- Joint-state publication
- Gripper commands
- Gravity compensation
- The 250 Hz hardware loop

For the leader, the subclass additionally:

- Reads the teaching-handle trigger and buttons.
- Detects button press and release edges.
- Publishes `/yam_leader/teaching_handle_state`.
- Subscribes to `/yam_leader/pd_gain_scale`.
- Starts with zero PD stiffness for hand-guiding.
- Applies scaled `kp` for bilateral feedback.

For the follower, the subclass behaves like the normal hardware interface; the
leader-only additions are disabled.

### `leader_follower_teleop.py`

This node contains the actual teleoperation behavior. It:

- Watches teaching-handle button 1.
- Slowly aligns the follower with the leader.
- Sends leader arm positions to the follower.
- Sends the handle trigger to the follower gripper.
- Sends measured follower positions back to the leader.
- Enables or disables bilateral PD feedback.
- Disables synchronization if robot-state communication becomes stale.

## ROS 2 communication

The nodes do not call one another's Python functions directly. They exchange
ROS 2 messages through these topics:

| Topic | Publisher | Subscriber | Purpose |
|---|---|---|---|
| `/yam_leader/joint_states` | Leader hardware | Coordinator | Measured leader arm position |
| `/yam_leader/teaching_handle_state` | Leader hardware | Coordinator | Trigger and button state |
| `/yam_follower/joint_states` | Follower hardware | Coordinator | Measured follower position |
| `/yam_follower/joint_command` | Coordinator | Follower hardware | Follower arm target |
| `/yam_follower/gripper_command` | Coordinator | Follower hardware | Follower gripper target |
| `/yam_leader/joint_command` | Coordinator | Leader hardware | Bilateral position target |
| `/yam_leader/pd_gain_scale` | Coordinator | Leader hardware | Bilateral feedback strength |

### Leader-to-follower path

```text
Physical leader
    → leader hardware interface
    → /yam_leader/joint_states
    → teleop coordinator
    → /yam_follower/joint_command
    → follower hardware interface
    → physical follower
```

### Follower-to-leader feedback path

```text
Physical follower
    → follower hardware interface
    → /yam_follower/joint_states
    → teleop coordinator
    → /yam_leader/joint_command
    → leader hardware interface
    → force felt at the leader
```

The approximate leader feedback is:

```text
torque ≈ scaled kp × (follower position - leader position)
```

When `bilateral_kp` is `0.0`, this position feedback is disabled.

## Teleoperation state machine

The coordinator uses three states:

```text
IDLE → ALIGNING → SYNCHRONIZED
  ▲                       │
  └──── button press ─────┘
```

### `IDLE`

- Both robots publish their measured states.
- The coordinator sends no synchronized motion commands.
- The leader's bilateral PD gains are zero.

### `ALIGNING`

When teaching-handle button 1 is pressed:

1. The coordinator records both current arm positions.
2. The follower moves gradually from its position to the leader's position.
3. The gripper is also interpolated toward the trigger position.
4. The default alignment time is three seconds.

Keep the leader still during this step.

### `SYNCHRONIZED`

After alignment:

- Leader positions continuously command the follower.
- The handle trigger continuously commands the follower gripper.
- Follower positions continuously become the leader's bilateral target.

Pressing button 1 again returns the system to `IDLE` and sets the leader's PD
gain scale to zero.

If leader or follower state messages stop for too long, synchronization is
also disabled automatically.

## Build and run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to i2rt_yam_teleop move_it_yam

source install/setup.bash

ros2 launch move_it_yam demo_launch_teleop.py
```

Default configuration:

```text
Leader CAN:          can1
Follower CAN:        can0
Follower gripper:    linear_4310
Bilateral Kp scale:  0.0
Alignment duration:  3.0 seconds
Command frequency:   100 Hz
```

Parameters can be overridden:

```bash
ros2 launch move_it_yam demo_launch_teleop.py \
  leader_can_channel:=can1 \
  follower_can_channel:=can0 \
  follower_gripper_type:=linear_4310 \
  bilateral_kp:=0.0 \
  slow_sync_duration:=3.0
```

## Operating procedure

1. Confirm `can0` and `can1` are configured and active.
2. Confirm no other process is controlling either robot.
3. Place both arms in reasonably similar starting positions.
4. Launch the teleoperation system.
5. Wait for all three nodes to report that they are ready.
6. Press teaching-handle button 1.
7. Keep the leader still during slow alignment.
8. Move the leader after synchronization is reported.
9. Press button 1 again to disable teleoperation.
10. Use `Ctrl+C` to shut down the launch.

Start with `bilateral_kp:=0.0`. Increase it only after one-way tracking has
been verified safely and an appropriate gain has been approved for the
hardware.

## Useful diagnostics

List running nodes:

```bash
ros2 node list
```

Inspect states and teaching-handle input:

```bash
ros2 topic echo /yam_leader/joint_states
ros2 topic echo /yam_follower/joint_states
ros2 topic echo /yam_leader/teaching_handle_state
```

Expected nodes:

```text
/yam_leader_hardware_interface
/yam_follower_hardware_interface
/leader_follower_teleop
```