# icra18-fci-tutorial

## Quickstart

Requires `libfranka` >= 0.4.0, `franka_ros` >= 0.3.0

To configure:

 1. Set all used controllers in the `launch/demo.launch` rosparam "controllers". Note: `position_joint_trajectory_controller` is required for MoveIt.
 2. Add the steps you want to perform to `config/demo.yaml`.

To execute:

    roslaunch icra18 demo.launch robot_ip:=<robot-ip>


## Known issues

If the robot is already in the start position, strong vibrations occur.
