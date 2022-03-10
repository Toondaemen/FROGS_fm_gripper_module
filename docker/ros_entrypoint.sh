#!/bin/bash
rm -r devel/ build/ log/

#####################################################
# Configure the container to run the pressure sensor
#####################################################
cp /home/ros/catkin_ws/src/extras/tactile_pressure_sensor/00-teensy.rules /etc/udev/rules.d/udevadm control --reload-rules && udevadm trigger

cd /home/ros/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git


#################################
# Build catkin_ws and source it !
#################################
cd /home/ros/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
catkin_make install
source ../home/ros/catkin_ws/devel/setup.bash

################################
# Add missing lib Tactile Sensor
################################
cd ../home/ros/catkin_ws/src/extras/tactile_pressure_sensor/
unzip -o arduino-1.8.16-linux64.zip
cd ../home/ros/catkin_ws/src/extras/tactile_pressure_sensor/arduino-1.8.16-linux64/arduino-1.8.16/libraries
# MS5840
git clone --mirror https://github.com/g-rov/MS5840
# ros_lib
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py . 


######################################
# Update BASHRC file ros / catkin_ws !
######################################
echo "# source ros melodic bash"          >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

echo "# source catkin workspace frogs online decisions bash"  >> ~/.bashrc
echo "source /home/ros/catkin_ws/devel/setup.bash"          >> ~/.bashrc

source ~/.bashrc


######################################
# change mode of pressure sensor + USB
######################################
echo ros | sudo -S chmod +666 /dev/ttyACM0


#######################
# SELECT ONE FROM BELOW
#######################
cd /home/ros/catkin_ws
# 1) roscore application
echo "STARTING melodic ROSCORE MASTER ..."
# roscore


# 2) ...

# 3) ...



# add this at the end of your entrypoint file
# to keep the container running and not exit directly
set -x
while $1
do
    echo "Press [CTRL+C] to stop.."
    sleep 60
    echo "My second and third argument is $2 & $3"
done
