# Jim — laser weeding gantry, ROS 2 Jazzy

2-DOF Cartesian XY gantry. Static frame, no mobile base.
Camera detects weeds, gantry positions a laser head over them.

## Hard decisions already made
- NO Nav2 (no mobile base), NO MoveIt (2-DOF Cartesian IK is identity)
- ros2_control + JointTrajectoryController on two prismatic joints
- Gazebo Harmonic via gz_ros2_control; hardware later via a custom
  SystemInterface speaking serial to an Arduino
- Perception is an oracle detector reading Gazebo ground truth,
  publishing vision_msgs/Detection2DArray. No YOLO.

## Numbers
travel        0.250 m both axes
lead screw    T8, 2 mm pitch x 4 start = 8 mm/rev
steps/mm      25 full step, 400 at 16x microstepping
max velocity  0.035 m/s  (lead screw critical speed, not motor torque)
motors        NEMA 17, 1.8 deg, run at 0.8-1.0 A not 1.5 A (PLA mounts)
drivers       MKS TMC2209, StallGuard sensorless homing

## Conventions
- Metres and radians everywhere (REP-103). ONE unit conversion, in the
  hardware interface. ROS never sees steps or mm.
- Tree: base_link -> x_carriage -> y_carriage -> laser_aim_link
- joint_x carries a 180 deg roll from CAD, so machine +Y runs along
  base_link -Y. Not a bug.

## Container
docker compose exec ros bash   ->  workspace at /home/dev/ws
