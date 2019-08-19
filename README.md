# ascension_ros
ROS driver for NDI/Ascension trakSTAR/driveBAY2 devices

The package was tested in Ubuntu 16.04.

# Getting started

## Create a workspace in catkin
 
You can follow the instructions on how to create a workspace in catkin here http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Clone the repo to your src directory in the catkin workspace

Build the project through `catkin_make` or `catkin build`

## Copy USB permissions file

Copy the "99-libusb.rules" file into the path "/etc/udev/rules.d/" and reload the udev rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Start the system

Start ROS through terminal: 
```bash
roscore
```

Set the source from the main catkin_ws: 
```bash
source devel/setup.bash
```

Navigate to the launch directory of the project and start the demo: 
```bash
roslaunch demo.launch
```

Or, start the system only and open with it with ImFusion (or any other preferred software).
