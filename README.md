# Mobile robot

Project of an autonomous mobile robot as a master's thesis project.

As for now, there is Arduino Nano for DC motors steering via L298N modules. As a reference speed there are optical encoders used. For optimal and synchronized speed for every wheel there is PID used, based on Arduino PID v1 library.

However, the 'brain' is Raspberry 5 with Ubuntu 24.04 and ROS2 Jazzy workspace. There are nodes for:
- communication with Arduino via Serial port,
- publishing odometry,
- publishing pose and initial pose for mapping and SLAM purpose,
- node for AHT10 sensor for reading temperature and humidity,

and scripts:
-  for turning on/off LED, when the sensor values are above safe threshold.

The robot can be controlled manually using teleop_twist_keyboard package.
Then mapping can be done, using slam_toolbox package.
After that, robot can move autonomously, using nav2_bringup package.
