echo "Source built packages"
cd catkin_ws
source /home/ros/catkin_ws/devel/setup.bash
echo "Giving access right to gripper"

# This needs to be done before docker-compose 
#chmod 777 /dev/ttyUSB0



echo "starting gripper driver"
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0