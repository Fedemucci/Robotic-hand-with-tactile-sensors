# ROBOTIC HAND WITH TACTILE SENSORS 
The provided software is designed to control an AR10 humanoid robotic hand and acquire data from piezo-electric tactile sensors applied on its fingers.
The controlled hand has three sensors applied on Thumb, Index and Ring fingers, so those are the only three joints controlled by the software.


***
## DEPENDENCIES/PAKAGES
1. Ubuntu OS
2. Python3 (Libraries-csv,matplotlib,numpy,,os,rospy,serial,struct,sys,time,yaml)
3. ROS(Robot operating system , version-kinetic) [Instalation Guide](http://wiki.ros.org/ROS/Installation "ROS")


***
## SOFTWARE DESCRIPTION
* **pub_serial.py**
This node reads the data from sensors,unpack them and then publishes the signal f each finger.
When publishing, it subtracts the mean values to the raw data values, making a tare.

Publish on:	Thumb, Index, Ring.
Subscribed to:	Mean_T, Mean_I, Mean_R.
Services: None

> settings:

18	self.ser = serial.Serial('/dev/ttyUSB0', 1000000)	#!change with the port to which the sensors are connected


* **Tarer.py**
This node contains the service to update the mean values to do the tare.

Publish on:	Mean_T, Mean_I, Mean_R.
Subscribed to:	Thumb, Index, Ring.
Services: 	'tare' input: number of datas to read before calculating the mean.


* **Hand.py**
This node controls the hand, giving motion inputs and getting feedback position values. It publishes the actual and set position for each joint of each finger and the set speed. It contains two services to initialize and move the hand

Publish on:	Set_T, Set_I, Set_R, Pos_T, Pos_I, Pos_R, Velocity
Subscribed to:	None
Services:	'grasp_initialization'	input:	joints to set, open position
		'grasp_routine'		input:	moving fingers, speed values,
						joints to move, close position, depth

* **Recorder**
This node reads data from all the topics. When the start service is called, it starts storing them.
When is called the stop one, it save the acquisition.

Publish on:	None
Subscribed to:	Thumb, Index, Ring, Set_T, Set_I, Set_R, Pos_T, Pos_I, Pos_R, Velocity
Services:	'start_recording'	input: finger to save, save directory
		'stop_recording'	input: None

* **Controller**
This node calls the services from the other nodes to make acquisitions. It has to be customised with the wanted parameters. It creates the saving directory and saves in it a file with parameters.

> settings

18	self.vel_steps = [5,10,20]	#! velocity steps for the grasping routine 
19	#                 Thumb     Index     Ring
20      self.qi_set =  [   0,1,      8,9,      4,5   ]	#! joints to be set in open position
21      self.q_set =   [5530,8000,6800,7900,7000,7800]	#! values of the joints in open position
22      self.qi_move = [    1,        9,        5    ]	#! joints to be moved to close position during grasping
23      self.q_max =   [   7300,     6750,     6800  ]	#! max value of the joints in close position during grasping
24      self.q_depth = 100                            	#! offset subtracted to the max value of the joints in close position ...
25      self.dataset_dir = ''                         	# ... to achieve different pressure levels

51	self.dataset_dir = '/home/lar03/ros/catkin_ws/Dataset'	#!change with the path of the directory where to save the dataset

89	#         T I R
90	Ctrl.run([1,1,1])	#!select finger (1: active, 0: inactive)

* **ros_AR10_class.py**
It's not a node, contains the class to comunicate with the AR10 hand.
> settings:
```
33	self.usb = serial.Serial('/dev/ttyACM0', baudrate=9600)	#!change with the port to which the hand is connected
```
note: the hand creates two virtual ports, select the first

* **ros_AR10_calibrate.py**
It's not a node, should be run once at the beginning to calibrate the hand.

* **ros_AR10_calibrate_custom.py**
As before, but can be set the points of calibration based on the range of motion needed.

> settings:

 5	#!set the desired points of calibration for each joint
 6	joint_targets = [[5000, 5200, 5400, 5600,],  #0
 7	                 [7300, 7500, 7700, 7900,],  #1
 8      	         [6000, 6500, 7000, 7500,],  #2
 9      	         [7300, 7500, 7700, 7900,],  #3
10      	         [6700, 6900, 7100, 7300,],  #4
11      	         [6000, 6500, 7000, 7500,],  #5
12      	         [7300, 7500, 7700, 7900,],  #6
13      	         [6700, 6900, 7100, 7300,],  #7
14      	         [6900, 7100, 7300, 7500,],  #8
15      	         [6000, 6500, 7000, 7500,],] #9

* **velocity_control_GUI_right.py**
It's not a node, can be used to move the hand to understand it's functioning.

* **plotter.py**
This program can be used to visualize the acquire data, to do that copy the directory path.

> settings

5	start_dir= '/home/lar03/ros/'     #!Change this with the folder the workspace is in
6	copied_dir= 'catkin_ws/Dataset/Dataset_T_5530_8000_7300_2'  #!Change with the path of the folder you want to plot


***
## HOW TO RUN
> First check to have installed all the needed dependencies
> Create the workspace folder 
> Open terminal, select working directory, create src folder and enter it
```
$ mkdir catkin_ws
$ cd ~/catkin_ws/
$ mkdir src
$ cd src
```
> Copy all the files using [git clone](https://github.com/Fedemucci/Robotic-hand-with-tactile-sensors.git) in src
``` 
$ git clone https://github.com/Fedemucci/Robotic-hand-with-tactile-sensors.git
```
> Build the package from workspace directory
```
$ cd ..
$ catkin_make
```
> Source the setup file, this will be needed in any new terminal tab where you want to use the package
```
$ source devel/setup.bash
```
> Now, open a new terminal e start roscore
```
$ roscore
```
> Now, for each node to run, open a new tab and source setup file.
> pub_serial and Tarer can be run toghether using the launch file piezo_signal.launch.
> Run Controller.py last, after setting parameters to make acquisition.
```
$ roslaunch piezosensors piezo_signal.launch
```
```
$ rosrun piezosensors Recorder.py
```
```
$ rosrun piezosensors Hand.py
```
```
$ rosrun piezosensors Controller.py
```


