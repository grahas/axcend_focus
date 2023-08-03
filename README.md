# Repository for ROS2 and Axcend Focus LC

## Workflow for building ros2 package and getting them on the board

### Clean workspace

rm -r build install log

Build package

colcon build --packages-select axcend_focus_msgs

catkin_generate_changelog

catkin_prepare_release

bloom-release --rosdistro foxy axcend_focus

https://github.com/grahas/axcend_focus-release.git

https://github.com/grahas/axcend_focus.git

scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-msgs_0.0.3-1-r0.0_armhf.deb root@192.168.1.118:/tmp

ros2 pkg create axcend_focus_actions --build-type ament_cmake --dependencies action_msgs std_msgs rosidl_default_generators

