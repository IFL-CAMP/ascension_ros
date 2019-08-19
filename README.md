# ascension_ros
ROS driver for NDI/Ascension trakSTAR/driveBAY2 devices
You can run the package in Ubuntu (tested for Ubuntu 16.04)

# Create a workspace in catkin
 
You can follow the instructions on how to create a workspace in catkin here http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Clone the repo to your src directory in the catkin workspace

Build the project through catkin_make

# Copy USB permissions file

Copy the "99-libusb.rules" file into the path "/lib/udev/rules.d/" and reboot your machine

# Start the system

Start ROS through terminal: roscore

Set the source from the main catkin_ws: "source devel/setup.bash"

Navigate to the launch directory of the project and start the demo: "roslaunch demo.launch"

Or, start the system only and open with it with ImFusion (or any other preferred software)
