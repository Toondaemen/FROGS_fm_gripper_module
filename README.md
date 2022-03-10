# Launch the gripper module


## 1. Build the image
Navigate to the docker folder and run the script to build the docker image

```
cd docker
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

## 3. Start the gripper driver

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
roslaunch fm_gripper_module fm_gripper_module.launch
```

The Robotiq gripper LED should become blue. You can also see if you receive the gripper status by subscribing to the 'Robotiq2FGripperRobotInput' topic (You need to first enter the container with e.g. docker exec -it frogs_fm_gripper bash)

## 4. run the server node
in another terminal, open a new session in the running docker container.
```
./shell_container.sh
cd catkin_ws
source devel/setup.bash
rosrun fm_gripper_module gripper_action_server.py 
```



## 5. run the client node
in another terminal, open a new session in the running docker container.
```
./shell_container.sh
cd catkin_ws
source devel/setup.bash
rosrun fm_gripper_module gripper_test_action_client.py 
```

This will trigger the gripper code in the terminal of the server node.



# Troubleshooting

## 1. AttributeError: 'ModbusIOException' object has no attribute 'getRegister'

The response of the gripper status returned NULL. In cmy case, I solved this using the URCaps add-on in the UR teach pendant. The ID of the gripper was set to 2. Changing it to 1 solved this issue.

Online solution explain that waiting adding a while loop waiting for valid answer solved this issue. It did not solve it in my case.


# Documentation

This module uses the ros-industrial robotiq package: http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29


# Author 

Jean Hoyos - jean.hoyos@flandersmake.be





