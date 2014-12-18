Execute
==================

    $ roscore
    $ MobileSim
    $ rosrun rosaria RosAria
    $ rosrun rosaria AndroidTeleop

Android App
==================

Check the Android connection:

    $ rosnode list
    $ rostopic list

Read the Android IMU Sensors (inertial measurement unit):

    $ rostopic echo /android/imu
