# Agrobot — Software Contributions

This document covers the work done on the Agrobot repository: the ROS 2 / MoveIt 2 software that lets the arm and linear rail be commanded, planned for, and driven for tomato picking.
Scope: ~49 commits (from `6653fce` to `bf35ac4`) across the `robot_interfaces`, `robot_commander`, `robot_description`, `moveit_config`, `robot_bringup`, and `epos2_bridge` packages.

---

## 1. Command interfaces (`robot_interfaces`)

Created the custom interfaces package used by everything that commands the arm:

| Type | Name | Purpose |
|------|------|---------|
| msg | `JointCommand` | Target values for joints `j0`–`j6` |
| msg | `PoseCommand` | Named pose (e.g. a saved arm configuration) |
| msg | `PositionCommand` | `x y z roll pitch yaw` plus a `cartesian_path` flag |
| action | `PickSequence` | Takes a `PoseArray` of tomato targets; returns success/message; feedback reports tomato index, total, current step, and whether it is awaiting confirmation |
| srv | `SetIndicator` | Indicator control |

## 2. Robot commander (`robot_commander`)

The commander node (`src/commander.cpp`) is the bridge between high-level commands and MoveIt:

- Subscribers for **named goals**, **joint goals**, and **position goals** under the `/agrobot/...` namespace (`/agrobot/pose_cmd`, `/agrobot/joint_cmd`, `/agrobot/position_cmd`, `/agrobot/proceed`).
- A `PickSequence` **action server** that runs the main picking loop (approach → grasp → retract → drop).
- Added `launch/commander.launch.py` and later fixed a link bug in the commander and the main picking loop.
- Renamed all ROS topics to start with `/agrobot`.
- Test programs in `src/tests/`: `named_goal`, `joint_goal`, `position_goal`, `cartesian_path`, plus scripts for the button server, multi-tomato runs, and the main commander loop.

## 3. Tomato picker integration (`tomato_picker.py`, `sequencer.py`)

Started and then fixed the integration of the tomato-picking script, which takes the AI-vision detections and turns them into an ordered pick list for the commander (TF2 frame transform, safety gate on `/agrobot/safe_to_pick`, picked-ID tracking). Also added a placeholder camera transform so detections can be expressed in the robot frame.

## 4. Robot model (`robot_description` / URDF)

- Updated the **linear rail** link and joint for the new zero point, and adjusted joint 0 limits.
- Added the **gripper**: base link, fixed joint, fin links with mimic joints (and corrected the mimic joint limits), and the `Gripper.stl` mesh.
- Changed joint 6 from fixed to **revolute** (URDF and MoveIt config), and adjusted the link 5 → link 6 offset by 10 mm.
- Fixed the robot's orientation relative to the world frame.
- Added new **named poses** and renamed old ones; tidied link comments.
- Tried a mock gripper early on and reverted it in favor of the real gripper model.

## 5. MoveIt configuration (`moveit_config`, `robot_bringup`)

- Switched inverse kinematics from **KDL to TRAC-IK**.
- Redefined the arm as a **kinematic chain** in the SRDF.
- Added gripper groups/joints to the SRDF and MoveIt config.
- Updated joint limits, `ros2_controllers.yaml`, `moveit_controllers.yaml`, and initial positions.
- Added functional kinematics for **interactive markers in RViz**.
- Added a simple display launch file for testing joint updates, and updated the RViz configs.

## 6. Motor integration (`epos2_bridge`)

- Added a motor bringup package for the joint 0 (rail) driver, including its CANopen bus/EDS config and test script. It was later removed as unused once the EPOS2 bridge covered the motors.
- Updated the **EPOS2 bridge** package so all motors integrate with the MoveIt controller stack.

---

## Notes

- Several `*.bak_*` files in `epos2_bridge` and `robot_commander` are backup snapshots from hardware debugging; they are not part of the working code path.
- `src/robot_commander/src/tomato_picker.py` has detailed inline notes on how the picking pipeline works.
