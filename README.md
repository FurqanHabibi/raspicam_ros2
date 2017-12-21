# raspicam_ros2
Simple raspberry pi camera v2 node for ROS2.

## Install
### Install dependencies
```
$ sudo apt update
$ sudo apt install python-picamera python3-picamera
```
### Build
```
# with ROS2 already sourced
$ cd <ros2_workspace>/src
$ git clone https://github.com/FurqanHabibi/raspicam_ros2
$ cd ..
$ ament build

# for Linux / OS X
$ source install/local_setup.bash

# for Windows
$ call install/local_setup.bat
```

## Usage
- Make sure the camera module is plugged in
- Enable the camera
    ```
    $ sudo raspi-config
    ```
- Run the node
    ```
    $ ros2 run raspicam_ros2 raspicam_ros2
    ```

## ROS2 Node
