TinyRover
=========
RGI 6X6 TinyRover is a 6X6 skid steer platform the rgi has a X200 zoom camera mounted on a PT turret.
Status
------
 * Visualization in Gazebo/Rviz is working
 * All interfaces are standard ROS  
 * An example launch file to run the GAZEBO simulation plus RVIZ and rqt interfaces.
		roslaunch rgi_gazebo rgi_gazebo_full_monti.launch
 * There are python scripts to demonstrate the operation of the RGI
  *  The joystick control script – tested with a gamepad. The right stick drives the robot, the left stick drives the camera PT.
  * Keyboard control script -  drives the robot
Joystick
--------
 * run the joystick node 
		rosrun joy joy_node
 * run the joystick control script
		rosrun rgi_control joy_teleop.py
Keyboard
------------
		rosrun rgi_control  key_teleop.py
