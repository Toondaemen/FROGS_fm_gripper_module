# How to use module

This module is meant to be used together with SkiROS2. The communication between SkiROS2 and this module is done using ROS action request.
This module is a ROS action server that will transform the ROS action request (from SkiROS2) into a driver specific request that will actuate the Robotiq 2 fingers gripper. This package uses the robotiq 2 fingers package which uses Modbus RTU protocol and requires a USB connection.

The ROS action definition is stored in fm_gripper_msgs.

In order to be used with SkiROS2:

1. Both the SkiROS2 container and this module (gripper) container should be on the same network (host) in order to communicate.
2. SkiROS2 should compile the fm_gripper_msgs folder in order to be able to send ROS action request using this action format. For this build the msgs folder directly from the SkiROS2 catkin_ws.

```
# Inside catkin_ws of SkiROS2
catkin_make_isolated --pkg fm_gripper_msgs
source devel_isolated/setup.bash

```


# Launch the gripper module


## 1. Build the image

```
./build_image.sh
```

## 2. Build the package

Navigate in the container with the following

```
./run_container.sh
```

Navigate to the catkin workspace and build the packages

```
cd catkin_ws
catkin_make
```

Then source the built packages

```
source devel/setup.bash
```

## 3. Start the module

This module consists of 2 packages: server and driver
Both packages are started from the the docker-compose.yaml file.

Before starting the module:

1. Plug the usb and check if it is listed as USB0 with the following command

```
ls /dev/ttyUSB0
```

2. Give sudo permission of the gripper 

```
sudo chmod 777 /dev/ttyUSB0
```

3. Start the containers

```
docker-compose up
```


The Robotiq gripper LED should become blue. You can also see if you receive the gripper status by subscribing to the 'Robotiq2FGripperRobotInput' topic (You need to first enter the container with e.g. docker exec -it frogs_fm_gripper bash)



# Troubleshooting

## 1. AttributeError: 'ModbusIOException' object has no attribute 'getRegister'

The response of the gripper status returned NULL. In cmy case, I solved this using the URCaps add-on in the UR teach pendant. The ID of the gripper was set to 2. Changing it to 1 solved this issue.

Online solution explain that waiting adding a while loop waiting for valid answer solved this issue. It did not solve it in my case.


# Documentation

This module uses the ros-industrial robotiq package: http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29


# Author 

Jean Hoyos - jean.hoyos@flandersmake.be





