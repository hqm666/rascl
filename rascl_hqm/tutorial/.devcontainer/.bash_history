echo $ROS_DISTRO
ls
mkdir -p ros2_ws3/src
cd ros2_ws3
ls
cd src/
ros pkg create test
ros pkg create my_test
ros2 pkg create my_test
ls
rm -r my_test/
ls
ros2 pkg create -l Apache-2.0 cpp_srvcli -d rclcpp example_interface
ros2 pkg create --license Apache-2.0 cpp_srvcli -d rclcpp example_interface
ros2 pkg create --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interface
ls
tree -L 2
tree -L 3
tree -L 4
chmod -R cpp_srvcli/
chmod -R 777 cpp_srvcli/
touch ./cpp_srvcli/src/add_two_ints_server.cpp
tree -L 3
chmod -R 777 cpp_srvcli/
ls
touch ./cpp_srvcli/src/add_two_ints_client.cpp
chmod -R 777 .
[200~add_executable(client src/add_two_ints_client.cpp)
rosdep install -i --from-path src --rosdistro jazzy -y
cd ..
rosdep install -i --from-path src --rosdistro jazzy -y
ls
cd src/
ls
rosdep install -i --from-path src --rosdistro jazzy -y
cd ..
rosdep install -i --from-path src --rosdistro jazzy -y
lsb_release -a
ls /etc/apt/sources.list.d/
tree /etc/apt/sources.list.d/ -L 3
sudo apt update
apt-cache policy ros-jazzy-example-interfaces
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build
. install/setup.bash
ros2 run cpp_srvcli server
cd ..
mkdir -p ros2_ws4/src
cd ros2_ws4
ls
cd src/
ls
ros2 pkg create --license Apache-2.0 more_interfaces
mkdir more_interfaces/msg
ls
cd more_interfaces/msg/
touch AddresBook.msg
cd ..
cd..
cd ..
chmod -R 777 ./
touch /home/group1/rascl_hqm/tutorial/ros2_ws4/src/more_interfaces/src/more_interfaces/msg/AddressBook.msg
cd /home/group1/rascl_hqm/tutorial/ros2_ws4/src/more_interfaces/src
cd src/
cd more_interfaces/
cd src/
cd ..
mkdir more_interfaces/msg
cd more_interfaces/msg
ls
cd ..
ls
cd src/
ls
touch publish_address_book.cpp
ls
chmod 777 publish_address_book.cpp 
cd ..
colcon build --packages-up-to more_interfaces
exit
