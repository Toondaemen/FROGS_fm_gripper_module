
# Tactile Pressure Sensor  

## How to configure and prepare the sensor on host PC 

 - **Step 1**:  Install Arduino on the host pc https://www.pjrc.com/teensy/td_download.html
- **Step 2**:  Download the Linux udev rules (link at the top of this page) and copy the file to /etc/udev/rules.d.
						In this project you will find it at this location "extras/00-teensy.rules"
					
		$ sudo cp 00-teensy.rules /etc/udev/rules.d/
		$ sudo cp /tmp/00-teensy.rules /etc/udev/rules.d/
		$ sudo udevadm control --reload-rules && udevadm trigger


- **Step 3**: Download and extract one of Arduino's Linux packages.
	
	***Note***: Arduino from Linux distro packages is not supported.  
	You will find the zip file under the folder "extras/arduino-1.8.16-linux64.tar.xz"

- **Step 4**: Download the corresponding Teensyduino installer.
	Run the installer by adding execute permission and then execute it.
	
	    $ chmod 755 TeensyduinoInstall.linux64
	    $ ./TeensyduinoInstall.linux64

	If it is working close it and move to step2 to install missing libraries
  

## Run using Python / ROS
### Install python packages
			
		pip install pyserial
		
### Add missing library to arduino software
**1. MS5840**
- Navigate to the libraries folder of the arduino software and clone the following repo

		cd [path_to_extras]/extras/arduino-1.8.16-linux64/arduino-1.8.16/libraries
		git clone https://github.com/g-rov/MS5840

**2. ros_arduino**
- More info here http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installation should be done from source code
- Source build instructions are different for groovy+ (catkin) than for earlier (rosbuild) releases. Select the build system based on your release to see appropriate instructions.
- Rosserial has been catkin-ized since the groovy release, and the workflow is a bit different from fuerte and earlier releases. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace.
 
			cd <ws>/src
			git clone https://github.com/ros-drivers/rosserial.git
			cd <ws>
			catkin_make
			catkin_make install

 
- These commands clone rosserial from the github repository, generate the rosserial_msgs needed for communication, and make the library files in the <ws>/devel/lib directory.
 
> ***Note***: currently you HAVE to run catkin_make install, otherwise portions of the ros_lib directory will be missing. This will hopefully be fixed soon.

- After this run the following command
 
 			cd <sketchbook>/libraries
			rm -rf ros_lib
			rosrun rosserial_arduino make_libraries.py . 
		
- List out the folders you have now under libraries you should see "ros_lib"
 
  
## Open Arduino software and flash the Teensyduino

- Make sure that the configuration you are using is the correct one

# Tactile Pressure Sensor  

## How to configure and prepare the sensor on host PC 

 - **Step 1**:  Install Arduino on the host pc https://www.pjrc.com/teensy/td_download.html
- **Step 2**:  Download the Linux udev rules (link at the top of this page) and copy the file to /etc/udev/rules.d.
						In this project you will find it at this location "extras/00-teensy.rules"
					
		$ sudo cp 00-teensy.rules /etc/udev/rules.d/
		$ sudo cp /tmp/00-teensy.rules /etc/udev/rules.d/
		$ sudo udevadm control --reload-rules && udevadm trigger


- **Step 3**: Download and extract one of Arduino's Linux packages.
	
	***Note***: Arduino from Linux distro packages is not supported.  
	You will find the zip file under the folder "extras/arduino-1.8.16-linux64.tar.xz"

- **Step 4**: Download the corresponding Teensyduino installer.
	Run the installer by adding execute permission and then execute it.
	
	    $ chmod 755 TeensyduinoInstall.linux64
	    $ ./TeensyduinoInstall.linux64

	If it is working close it and move to step2 to install missing libraries
  

## Run using Python / ROS
### Install python packages
			
		pip install pyserial
		
### Add missing library to arduino software
**1. MS5840**
- Navigate to the libraries folder of the arduino software and clone the following repo

		cd [path_to_extras]/extras/arduino-1.8.16-linux64/arduino-1.8.16/libraries
		git clone https://github.com/g-rov/MS5840

**2. ros_arduino**
- More info here http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installation should be done from source code
- Source build instructions are different for groovy+ (catkin) than for earlier (rosbuild) releases. Select the build system based on your release to see appropriate instructions.
- Rosserial has been catkin-ized since the groovy release, and the workflow is a bit different from fuerte and earlier releases. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace.
 
			cd <ws>/src
			git clone https://github.com/ros-drivers/rosserial.git
			cd <ws>
			catkin_make
			catkin_make install

 
- These commands clone rosserial from the github repository, generate the rosserial_msgs needed for communication, and make the library files in the <ws>/devel/lib directory.
 
> ***Note***: currently you HAVE to run catkin_make install, otherwise portions of the ros_lib directory will be missing. This will hopefully be fixed soon.

- After this run the following command
 
 			cd <sketchbook>/libraries
			rm -rf ros_lib
			rosrun rosserial_arduino make_libraries.py . 
		
- List out the folders you have now under libraries you should see "ros_lib"
 
  
## Open Arduino software and flash the Teensyduino

- Make sure that the configuration you are using is the correct one

		Tools -> Board
		Tools -> Serial Port

- More details example https://www.pjrc.com/teensy/td_usage.html



# How to run it

* Connect the sensor to USB port and  open Arduino software  
		
		./arduino

* open the file ros_sensor.ino and configure the software using "***Tools-> board" + Tools > Port*** " then hit upload
* After flushing run a roscore then the following command

		rosrun rosserial_arduino serial_node.py /dev/ttyACM0

 * run the following python script to get extra info

		python3 ROS_LocAndForceEstimation.py

> Written with [StackEdit](https://stackedit.io/).




# Tactile Pressure Sensor  

## How to configure and prepare the sensor on host PC 

 - **Step 1**:  Install Arduino on the host pc https://www.pjrc.com/teensy/td_download.html
- **Step 2**:  Download the Linux udev rules (link at the top of this page) and copy the file to /etc/udev/rules.d.
						In this project you will find it at this location "extras/00-teensy.rules"
					
		$ sudo cp 00-teensy.rules /etc/udev/rules.d/
		$ sudo cp /tmp/00-teensy.rules /etc/udev/rules.d/
		$ sudo udevadm control --reload-rules && udevadm trigger


- **Step 3**: Download and extract one of Arduino's Linux packages.
	
	***Note***: Arduino from Linux distro packages is not supported.  
	You will find the zip file under the folder "extras/arduino-1.8.16-linux64.tar.xz"

- **Step 4**: Download the corresponding Teensyduino installer.
	Run the installer by adding execute permission and then execute it.
	
	    $ chmod 755 TeensyduinoInstall.linux64
	    $ ./TeensyduinoInstall.linux64

	If it is working close it and move to step2 to install missing libraries
  

## Run using Python / ROS
### Install python packages
			
		pip install pyserial
		
### Add missing library to arduino software
**1. MS5840**
- Navigate to the libraries folder of the arduino software and clone the following repo

		cd [path_to_extras]/extras/arduino-1.8.16-linux64/arduino-1.8.16/libraries
		git clone https://github.com/g-rov/MS5840

**2. ros_arduino**
- More info here http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installation should be done from source code
- Source build instructions are different for groovy+ (catkin) than for earlier (rosbuild) releases. Select the build system based on your release to see appropriate instructions.
- Rosserial has been catkin-ized since the groovy release, and the workflow is a bit different from fuerte and earlier releases. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace.
 
			cd <ws>/src
			git clone https://github.com/ros-drivers/rosserial.git
			cd <ws>
			catkin_make
			catkin_make install

 
- These commands clone rosserial from the github repository, generate the rosserial_msgs needed for communication, and make the library files in the <ws>/devel/lib directory.
 
> ***Note***: currently you HAVE to run catkin_make install, otherwise portions of the ros_lib directory will be missing. This will hopefully be fixed soon.

- After this run the following command
 
 			cd <sketchbook>/libraries
			rm -rf ros_lib
			rosrun rosserial_arduino make_libraries.py . 
		
- List out the folders you have now under libraries you should see "ros_lib"
 
  
## Open Arduino software and flash the Teensyduino

- Make sure that the configuration you are using is the correct one

		Tools -> Board
		Tools -> Serial Port

- More details example https://www.pjrc.com/teensy/td_usage.html



# How to run it

* Connect the sensor to USB port and  open Arduino software  
		
		./arduino

* open the file ros_sensor.ino and configure the software using "***Tools-> board" + Tools > Port*** " then hit upload
* After flushing run a roscore then the following command

		rosrun rosserial_arduino serial_node.py /dev/ttyACM0

 * run the following python script to get extra info

		python3 ROS_LocAndForceEstimation.py

> Written with [StackEdit](https://stackedit.io/).




# Tactile Pressure Sensor  

## How to configure and prepare the sensor on host PC 

 - **Step 1**:  Install Arduino on the host pc https://www.pjrc.com/teensy/td_download.html
- **Step 2**:  Download the Linux udev rules (link at the top of this page) and copy the file to /etc/udev/rules.d.
						In this project you will find it at this location "extras/00-teensy.rules"
					
		$ sudo cp 00-teensy.rules /etc/udev/rules.d/
		$ sudo cp /tmp/00-teensy.rules /etc/udev/rules.d/
		$ sudo udevadm control --reload-rules && udevadm trigger


- **Step 3**: Download and extract one of Arduino's Linux packages.
	
	***Note***: Arduino from Linux distro packages is not supported.  
	You will find the zip file under the folder "extras/arduino-1.8.16-linux64.tar.xz"

- **Step 4**: Download the corresponding Teensyduino installer.
	Run the installer by adding execute permission and then execute it.
	
	    $ chmod 755 TeensyduinoInstall.linux64
	    $ ./TeensyduinoInstall.linux64

	If it is working close it and move to step2 to install missing libraries
  

## Run using Python / ROS
### Install python packages
			
		pip install pyserial
		
### Add missing library to arduino software
**1. MS5840**
- Navigate to the libraries folder of the arduino software and clone the following repo

		cd [path_to_extras]/extras/arduino-1.8.16-linux64/arduino-1.8.16/libraries
		git clone https://github.com/g-rov/MS5840

**2. ros_arduino**
- More info here http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installation should be done from source code
- Source build instructions are different for groovy+ (catkin) than for earlier (rosbuild) releases. Select the build system based on your release to see appropriate instructions.
- Rosserial has been catkin-ized since the groovy release, and the workflow is a bit different from fuerte and earlier releases. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace.
 
			cd <ws>/src
			git clone https://github.com/ros-drivers/rosserial.git
			cd <ws>
			catkin_make
			catkin_make install

 
- These commands clone rosserial from the github repository, generate the rosserial_msgs needed for communication, and make the library files in the <ws>/devel/lib directory.
 
> ***Note***: currently you HAVE to run catkin_make install, otherwise portions of the ros_lib directory will be missing. This will hopefully be fixed soon.

- After this run the following command
 
 			cd <sketchbook>/libraries
			rm -rf ros_lib
			rosrun rosserial_arduino make_libraries.py . 
		
- List out the folders you have now under libraries you should see "ros_lib"
 
  
## Open Arduino software and flash the Teensyduino

- Make sure that the configuration you are using is the correct one

		Tools -> Board
		Tools -> Serial Port

- More details example https://www.pjrc.com/teensy/td_usage.html



# How to run it

* Connect the sensor to USB port and  open Arduino software  
		
		./arduino

* open the file ros_sensor.ino and configure the software using "***Tools-> board" + Tools > Port*** " then hit upload
* After flushing run a roscore then the following command

		rosrun rosserial_arduino serial_node.py /dev/ttyACM0

 * run the following python script to get extra info

		python3 ROS_LocAndForceEstimation.py

> Written with [StackEdit](https://stackedit.io/).




		Tools -> Board
		Tools -> Serial Port

- More details example https://www.pjrc.com/teensy/td_usage.html



# How to run it

* Connect the sensor to USB port and  open Arduino software  
		
		./arduino

* open the file ros_sensor.ino and configure the software using "***Tools-> board" + Tools > Port*** " then hit upload
* After flushing run a roscore then the following command

		rosrun rosserial_arduino serial_node.py /dev/ttyACM0

 * run the following python script to get extra info

		python3 ROS_LocAndForceEstimation.py

> Written with [StackEdit](https://stackedit.io/).



