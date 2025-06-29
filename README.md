# Mobile robot

Project of an autonomous mobile robot as a master's thesis project.

As for now, there is Arduino Nano for DC motors steering via L298N modules. As a reference speed there are optical encoders used. For optimal and synchronized speed for every wheel there is PID used, based on Arduino PID v1 library.

However, the 'brain' is Raspberry 5 with Ubuntu 24.04 and ROS2 Jazzy workspace. Currently, there is only a node for receiving nd sending speeds and distance. It will be upgraded soon...

