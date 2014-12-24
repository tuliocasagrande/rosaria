Introduction
==================

This repo just adds an application to the RosAria package. If you want the latest building and features, please go to the original repository [amor-ros-pkg/rosaria](https://github.com/amor-ros-pkg/rosaria).

The application we created applies a simple autonomous algorithm and a teleoperation control using an Android device sending the gyroscope generated information. The autonomous navigation is naive and just avoids obstacles and follows walls.

Installation
==================

First, you need to install the [Robot Operating System (ROS)](http://wiki.ros.org/ROS/Installation) and the [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) simulator.

Done that, you need to install the Rosaria package. The instructions to install the original RosAria are available [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA).

* In order to install our version, in the step **1.3** just be sure to clone this repository, instead of the amor-ros-pkg/rosaria:

        git clone https://github.com/tuliocasagrande/rosaria.git

* If you don't use git, you can simply download this repository and unzip it as:

        ~/catkin_ws/src/rosaria

* If you already have the RosAria package, you can import our application. However, be aware that Rosaria is under development and our application may stop working in future releases. Just download the `AndroidTeleop.cpp` and put it under `~/catkin_ws/src/rosaria`. In addition, you need to append to your `~/catkin_ws/src/rosaria/CMakeList.txt`:

        # Android & Autonomous
        add_executable(AndroidTeleop AndroidTeleop.cpp)
        target_link_libraries(AndroidTeleop ${catkin_LIBRARIES})

Lastly, you need to install the [ROS Android Sensors Driver](https://play.google.com/store/apps/details?id=org.ros.android.sensors_driver) [wiki](http://wiki.ros.org/android_sensors_driver) to send the gyroscope information of your Android device to the robot. Tested with Android 4.3 and 4.4.

If you don't install or if you don't open the Android app, the robot will just assume there isn't manual operation.


Usage
==================

We only tested with the [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) simulator, but it is expected to work properly in a real robot. You need to open 4 terminals:

    $ roscore
    $ MobileSim -m ~/catkin_ws/src/rosaria/Maps/cross.map 
    $ rosrun rosaria RosAria
    $ rosrun rosaria AndroidTeleop


Troubleshooting
==================

If the robot is not responding to the manual operation, check the Android connection:

    $ rosnode list
    $ rostopic list

Read the Android IMU Sensors (inertial measurement unit):

    $ rostopic echo /android/imu
