# The PointCloud Tailor

## 1. Required Libraries

### 1.1. PCL

In order to carry out our project, it is important to have the necessary libraries to work with point clouds, in this case _PCL_:

Install the libraries in Ubuntu:

    sudo apt-get install libpcl-dev

Install the libraries package in ROS:

    sudo apt-get install ros-noetic-pcl-ros

Verify that they has been installed correctly:

    rospack find pcl_ros

### 1.2. Eigen

In addition, it is necessary to install the _Eigen_ library, that is a mathematical library developed to work with geometric transformations and mathematical optimization efficiently:

Install the library:

    sudo apt install libeigen3-dev

Verify that it has been installed correctly:

    ls /usr/include/eigen3

## 2. Project Execution

### 2.1. Cloning the Project

First of all, we have to clone the GitHub repository that contains the final project in our workspace, to later execute it.

Using the terminal, we go to the src directory of our workspace and access the Github repository: https://github.com/albert-tomas/PAE-HP. And we clone it in our workspace using the following command:

    cd ~/catkin_ws/src/
    git clone https://github.com/albert-tomas/PAE-HP.git

Then, we must go to the root of our workspace (_catkin_ws_), making a jump back, and compile again with the command:

    cd ~/catkin_ws/
    catkin_make

At this time, as always before starting to run ROS in a terminal, it would be necessary to update the ROS environment with our workspace, but if we have previously modified our _~/.bashrc_ file adding the configuration of our workspace it will not be necessary, since when opening a new terminal it will do it by default. 

If you have not done, do this:

    source ~/catkin_ws/devel/setup.bash

### 2.2. Execution Steps

Now we have everything ready to start working, we just have to follow the steps that we detail below:

In a terminal, we run the ROS Master: 

    roscore

In another terminal, we can easily run the program through the launch file:

    roslaunch point_cloud_tailor alignment.launch

What we are doing is to execute the _alignment.lauch_ file found in the point_cloud_tailor package in the launch directory. This will run the node correctly and launch an RViz window with all parameters initialized.

Then, we have to place ourselves in the same directory where the dataset bag is located, and decompress the bag file.

We can be located in the rosbags directory or write the full path in the bag’s name.

Finally, in another terminal, we publish the point clouds to the topic using:

    cd ~/catkin_ws/PAE-HP/rosbags/
    rosbag play livox_room.bag

### 3. Results
