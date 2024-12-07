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
Además, hemos añadido un servicio de ROS al que se puede llamar en cualquier momento para que guarde el resultado de la nube de puntos final alineada en formato .pcd.
Verify that it has been installed correctly:

    ls /usr/include/eigen3

## 2. Project Execution

### 2.1. Cloning the Project

First of all, you have to clone the GitHub repository that contains the final project in your workspace, to later execute it.

Using the terminal, go to the src directory of your _Catkin_ workspace and access the Github repository: https://github.com/albert-tomas/PAE-HP. Clone it into your workspace using the following command:

    cd ~/catkin_ws/src/
    git clone https://github.com/albert-tomas/PAE-HP.git

Then, you must go to the root of your workspace (_catkin_ws_), making a jump back, and compile it again with the command:

    cd ~/catkin_ws/
    catkin_make

At this time, as always before starting to run ROS in a terminal, it would be necessary to update the ROS environment with your workspace, but if you have previously modified our _~/.bashrc_ file adding the configuration of your workspace it will not be necessary, since when opening a new terminal it will do it by default. 

If you have not done, do this:

    source ~/catkin_ws/devel/setup.bash

### 2.2. Execution Steps

Now you have everything ready to start working, you just have to follow the steps that we detail below:

In a terminal, run the ROS Master: 

    roscore

In another terminal, you can easily run the program through the launch file:

    roslaunch point_cloud_tailor alignment.launch

What you are doing is to execute the _alignment.lauch_ file found in the point_cloud_tailor package. This will run the node by default, and the ceiling will not be shown, also it will launch an RViz window with all parameters initialized. 

If you want to see the complete detection of the room, add the argument _show_ceiling:=true_:
    
    roslaunch point_cloud_tailor alignment.launch show_ceiling:=true

Then, you have to place yourself in the same directory where the dataset bag is located, and decompress the bag file. You can be located in the rosbags directory or write the full path in the bag’s name. Finally, in another terminal, publish the point clouds to the topic using:

    cd ~/catkin_ws/src/PAE-HP/rosbags/
    rosbag play livox_room.bag

Additionally, we have added a ROS service that can be called in another terminal window at any time to save the final aligned point cloud result in `.pcd` format in the directory where the program is running:

    rosservice /save_pcd


### 3. Results
