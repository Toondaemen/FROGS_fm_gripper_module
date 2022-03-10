echo "Source built packages"
cd catkin_ws
source /home/ros/catkin_ws/devel/setup.bash
echo "Giving access right to tactile sensor"

# This needs to be done before docker-compose 
#chmod 777 /dev/ttyUSB0



echo "starting tactile sensor driver"
rosrun rosserial_python serial_node.py /dev/ttyACM0
roslaunch tactile_pressure_sensor run_tactile_pressure.launch
