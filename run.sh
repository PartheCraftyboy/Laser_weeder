#!/usr/bin/env bash
set -e
cd ~/Jim
export HOST_UID=$(id -u) HOST_GID=$(id -g)
xhost +local:docker >/dev/null 2>&1
docker compose up -d
# Clear the previous run. Gazebo is not enough: the rclpy nodes survive a
# gz kill and you end up with two clock bridges and two targeting nodes
# fighting over the same controller. The [x] brackets stop each pattern
# matching the cleanup shell's own command line, which would kill it here.
docker exec jim bash -lc "
  for p in '[g]z sim' '[r]uby' '[r]os2 launch' '[t]argeting_node' \
           '[o]racle_detector' '[r]obot_state_publisher' \
           '[p]arameter_bridge' '[s]tatic_transform_publisher' \
           '[r]viz2' '[c]ontroller_manager' '[s]pawner'; do
    pkill -9 -f \"\$p\" || true
  done
  # The discovery daemon outlives the nodes and will keep listing dead
  # ones, which makes 'ros2 node list' lie during debugging.
  source /opt/ros/jazzy/setup.bash
  ros2 daemon stop >/dev/null 2>&1 || true
  true"
sleep 1
docker exec -it jim bash -lc "
  source /opt/ros/jazzy/setup.bash
  cd ~/ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch jim_sim field.launch.py
"
