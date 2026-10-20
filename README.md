# Lab 1: ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems, Alexandria University (ROS 2 Humble)

## Team
| Name | ID |
|------|----|
| Youssef Mahmoud | 9385 |
| : member 2 |  |
| : member 3 |  |

## Packages
- `lab1_interfaces`: custom interfaces
  - `msg/TargetPose.msg` (x, y, yaw)
  - `action/GoToPose.action` (goal: TargetPose, feedback: distance_remaining)
  - `action/DrawSquare.action` (feedback: current_side, distance_remaining)
  - `srv/SetSpeedLimits.srv` (max_linear_speed, max_angular_speed)
- `lab1_control`: nodes, launch file, parameters
  - `go_to_pose`: `/go_to_pose` action server and `/set_speed_limits` service
  - `reset_service`: `/reset_turtle` service (pen up, teleport, clear, pen down)
  - `draw_square`: `/draw_square` action server (calls reset, then 4 GoToPose goals)

## Build
```bash
mkdir -p ~/ros2_lab1/src && cd ~/ros2_lab1/src
git clone https://github.com/makkawy100/ros2_lab1.git .
cd ~/ros2_lab1
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run (one command)
```bash
ros2 launch lab1_control lab1.launch.py
```
Starts turtlesim and all servers with parameters from `config/params.yaml`.

## Commands to test each requirement
- R2/R8, interfaces: `ros2 interface show lab1_interfaces/action/GoToPose`
- R3, goToPose:
  `ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback`
- R4, reset: `ros2 service call /reset_turtle std_srvs/srv/Trigger`
- R5, square: `ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{}" --feedback`
- R7, speeds:
  - `ros2 param get /go_to_pose_server max_linear_speed`
  - `ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: 0.5, max_angular_speed: 0.5}"`
  - Invalid values (<= 0) are rejected.
- Square side length: `ros2 param set /draw_square_server side_length 3.0`
