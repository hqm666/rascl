echo $ROS_DISTRO
ls
cd src
ls
ros2 pkg create --build-type ament_cmake --license Apache-2.0 motorctl_cpp
ls
chmod -R 777 motorctl_cpp/
colcon build --packages-select motorctl_cpp
colcon clean
rosclean
cd ..
ls
colcon build --packages-select motorctl_cpp
cd src/
ls
chmod -R 777 log/
chmod -R 777 install/
chmod -R 777 build/
cd ..
colcon build
rossetup
ros2 run motorctl_cpp slave
