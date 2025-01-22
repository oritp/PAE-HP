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

First of all, you have to clone in your workspace the GitHub repository that contains the final project, to later execute it.

Using the terminal, go to the _src_ directory of your _Catkin_ workspace and access the Github repository: https://github.com/albert-tomas/PAE-HP. Clone it into your workspace using the following command:

    cd ~/catkin_ws/src/
    git clone https://github.com/albert-tomas/PAE-HP.git

Then, you must go to the root of your workspace (_catkin_ws_), making a jump back, and compile it again with the command:

    cd ~/catkin_ws/
    catkin_make

At this time, as always before starting to run ROS in a terminal, it would be necessary to update the ROS environment with your workspace, but if you have previously modified your _~/.bashrc_ file adding the configuration of your workspace it will not be necessary, since when opening a new terminal it will do it by default. 

If you have not done, do this:

    source ~/catkin_ws/devel/setup.bash

### 2.2. Execution Steps

Now you have everything ready to start working, you just have to follow the steps that we detail below:

Running the ROS Master is not necessary if you are going to run the program with a launch file, so in a terminal window, you can easily run the program like this:

    roslaunch point_cloud_tailor alignment.launch

What you are doing is to execute the _alignment.lauch_ file found in the point_cloud_tailor package. This will run all the nodes by default, and the ceiling will not be shown, also it will launch an RViz window with all parameters initialized. 

Then, you have to place yourself in the same directory where the dataset bag is located, and decompress the bag file. You can be located in the rosbags directory or write the full path in the bag’s name. Finally, in another terminal, publish the point clouds to the topic using:

    cd ~/catkin_ws/src/PAE-HP/rosbags/
    rosbag play livox_room.bag

### 2.3. Additional Features

Additionally, we have added several ROS services that perform different functions that we found useful to validate the correct operation of the algorithm.

#### a) Save PCD

In another terminal window you can call a service at any time to save the final aligned point cloud result in .pcd format in the directory where the program is running:

    rosservice call /save_pcd

#### b) Publish Trajectory

For this other service we have applied _SLAM_ in order to show the trajectory made by the robot with the LiDAR sensor:

    rosservice call /trajectory

#### c) Show Ceiling

If you want to see the complete detection of the room, including the ceiling, add the argument _show_ceiling:=true_:

    roslaunch point_cloud_tailor alignment.launch show_ceiling:=true

#### d) Don't Publish on RViz

In case you do not want to use or publish on RViz the final aligned cloud, you can add the argument _publish_cloud:=false_, if you do not write anything the program will run by default de visualization:

    roslaunch point_cloud_tailor alignment.launch publish_cloud:=false

#### e) 2D Map (PCD to BMP)

To end, we have created a simple program _2D_map.cpp_, that you can run whenever you want, capable of reading the final point cloud result in .pcd format, which after properly filtering it and collapsing its vertical axis, we obtain a 2D image in .bmp format in order to check the quality of the construction maps:

If it is not compiled, compile the program using the following command that explains its dependencies:

    g++ -o 2D_map 2D_map.cpp -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -lpcl_common -lpcl_io -lpcl_filters

Then, you can run the program by default (without showing the ceiling of the room) like this:

    ./2D_map cloud_name.pcd

If you want to show the ceiling or you are working with taller walls you can add a number argument to define the desired height:

    ./2D_map cloud_name.pcd [MAX_HEIGHT]
