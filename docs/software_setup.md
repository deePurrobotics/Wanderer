# Software Installation Guide for Wanderer Robot.
This guide helps you with Wanderer's software installation. A brand new Jetson TX2 is about to become a fully prepared iRobot rider (Take a deep breathe, let's rock.).

## [Jetpack3.3](https://developer.nvidia.com/embedded/jetpack) Installation
Find out Jetpack download link and related materials [here](https://developer.nvidia.com/embedded/downloads#?search=jetpack%203.3). Full-flash TX2 with Jetpack 3.3.1

## Post Flash
**Caution: DO NOT `$ sudo apt upgrade` anytime!!!**
#### Patch kernel \& install librealsense
1. Open terminal by `Ctrl` `Alt` `t`, clone the repo that contains all the tools we are going to need.
```
cd $HOME
git clone https://github.com/jetsonhacks/buildLibrealsense2TX
cd buildLibrealsense2TX
git checkout v0.9
```
2. Build patched kernel. Touch `buildPatchedKernelTX.sh` with your favorite text editor. Modify *line 11* to `LIBREALSENSE_VERSION=v2.21.0`, then `$ ./buildPatchedKernel.sh` and reboot TX2 after installation.
3. Install *[librealsense](https://github.com/IntelRealSense/librealsense)*. Touch `installLibrealsense.sh` and change *line 10* to `LIBREALSENSE_VERSION=v2.21.0`, then `$ ./installLibrealsense.sh`
> Again, **DO NOT** perform `upgrade` before you build the patched kernel!!!

~~## (Optional) Install emacs~~

~~`$ sudo apt install emacs`
config emacs by going to home directory `$ cd ~` then `$ git clone https://github.com/linZHank/.emacs.d.git`. Fire up emacs `$ emacs` to start installing packages and setting things up.~~

~~## Set static ip address~~

~~You may reference to this [post](https://devtalk.nvidia.com/default/topic/988803/jetson-tx1/how-to-set-tx1-to-use-static-ip-on-ethernet-port/post/5061512/#5061512)~~

## Install ROS-Kinetic
1. `$ cd ~` then `$ git clone https://github.com/linZHank/installROSTX2.git`<br/>
2. `$ cd installROSTX2` then `$ ./installROS.sh`<br/>
This will install ros-kinetic-base and neccessary tools including [catkin-command-line-tools](http://catkin-tools.readthedocs.io/en/latest/)</br>

## Install [ROS wrapper for RealSense]

1. Create a catkin workspace
```console
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/intel-ros/realsense.git
cd realsense
git checkout 2.2.3
cd ~/ros_ws
catkin init
```
2. Install dependencies (may not limited to the following)
```console
sudo apt update
sudo apt install ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-tf ros-kinetic-diagnostic-updater ros-kinetic-image-view ros-kinetic-rviz ros-kinetic-rqt
```
3. Build RealSense ROS wrapper
```console
cd ~/ros_ws
catkin build --cmake-args -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4. Test RealSense ROS
```console
roslaunch realsense2_camera rs_camera.launch
```

## Build [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
> Stay tuned, coming soon...
> **Update:** coming in 1024 years

## Install [create_autonomy](https://github.com/AutonomyLab/create_autonomy.git)
#### Compile from source
1. Create a catkin workspace  
    ```
    $ cd ~
    $ mkdir -p create_ws/src  
    $ cd create_ws  
    $ catkin init  
    ```
2. Clone this repo  
    ```
    $ cd ~/create_ws/src
    $ git clone https://github.com/AutonomyLab/create_autonomy.git  
    ```
3. Install dependencies  
    ```
    $ cd ~/create_ws
    $ rosdep update  
    $ rosdep install --from-paths src -i  
    ```
4. Build  
    ```
    $ cd ~/create_ws
    $ catkin build
    ```
5. In order to connect to Create over USB, ensure your user is in the dialout group
    ```
    $ sudo usermod -a -G dialout nvidia
    ```
6. Logout and login for permission to take effect
#### Running the driver

1. After compiling from source, don't forget to source your workspace:  
    ```
    $ source ~/create_ws/devel/setup.bash
    ```
2. Connect TX2 to Create's 7-pin serial port, plug logitech joy-pad wireless receiver in TX2's usb-hub.
3. For Create 2 (Roomba 600/700 series):
```
$ roslaunch ca_driver create_2.launch
```
**NOTICE:**Make sure the Create is not in charge on the charge-base before excuting this command.
4. Remote control using a Logitech F710 joy-pad
```
$ roslaunch ca_tools joy_teleop.launch [joy_config:=log710]
```
   If use a keybord instead of the joy-pad
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
5. Open the onboard camera
```
$ roslanuch realsense2_camera rs-rgbd.lanuch
```

## (Optional) Install [libfreenect2](https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux)
> You will need this if you were using a Kinect V2 RGB-D camera
* Download libfreenect2 source
    ```
    $ git clone https://github.com/OpenKinect/libfreenect2.git
    $ cd libfreenect2
    ```
* Install build tools
    ```
    $ sudo apt-get install build-essential cmake pkg-config
    ```
* Install libusb. The version must be >= 1.0.20.
    `$ sudo apt-get install libusb-1.0-0-dev`
* Install TurboJPEG
    `$ sudo apt-get install libturbojpeg libjpeg-turbo8-dev`
* Install OpenGL
    `$ sudo apt-get install libglfw3-dev`
* Install OpenNI2 (optional)
    `$ sudo apt-get install libopenni2-dev`
* Build (if you have run `$ cd depends` previously, `$ cd ..` back to the libfreenect2 root directory first.)
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
    make
    make install
    ```
    You need to specify `cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2` for CMake based third-party application to find libfreenect2.
* Set up udev rules for device access: `sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, then replug the Kinect.
* Run the test program: `./bin/Protonect`
* Run OpenNI2 test (optional): `sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, etc to specify the pipeline.

## (Optional) Install [IAI_Kinect 2](https://github.com/code-iai/iai_kinect2#install)
> You will need this if you were using a Kinect V2 RGB-D camera and you want to use it in ROS
1. Clone this repository into your catkin workspace, install the dependencies and build it:
```
cd ~/ros_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/ros_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE="Release"
```
2. Connect your sensor and run kinect2_bridge:
`$ roslaunch kinect2_bridge kinect2_bridge.launch`
3. Calibrate your sensor using the `kinect2_calibration`. Follow the [instructions](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one). Beware, you will have to take hundreds of screenshots manually, which is really time consuming and boring (Ask a girl to help you out XD).

~~## Install [realsense2 ROS wrapper](https://github.com/intel-ros/realsense)~~
~~1. If not using Kinect v2, and no libfreenect2 and iai_kinect2 were configured, then you'll have to install some dependencies:~~
~~`sudo apt install ros-kinetic-cv-bridge ros-kinetic-image-common`~~
~~2. Clone latest realsense2 wrapper~~
~~```~~
~~cd ~/ros_ws/src/~~
~~git clone https://github.com/intel-ros/realsense.git~~
~~cd ..~~
~~catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release~~
~~```~~
~~3. Test~~
~~roslaunch realsense2_camera rs_camera.launch~~

## (Optional) Install [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)
`$ sudo apt-get install ros-kinetic-rtabmap-ros`
