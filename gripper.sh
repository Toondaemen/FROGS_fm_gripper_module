echo "Source built packages"
cd catkin_ws
source /home/ros/catkin_ws/devel/setup.bash
echo "starting the gripper server"

roslaunch module_gripper gripper.launch 

