echo "Building ROS2 nodes"

cd Examples/ROS2/ORB_SLAM3
# rm -rf build
# rm -rf install
# rm -rf log
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3_ros2 --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON --symlink-install
. install/setup.bash

cd ../../..
