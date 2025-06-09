![University Project](https://img.shields.io/badge/University-UPC-blue)
![Platform](https://img.shields.io/badge/Platform-ROS1%20Noetic-orange)
![Sensor](https://img.shields.io/badge/Sensor-LiDAR-yellow)
![Library](https://img.shields.io/badge/Library-PCL%20%7C%20Eigen-lightgrey)
![C++](https://img.shields.io/badge/C++-62.9%25-red)
![Matlab](https://img.shields.io/badge/Matlab-17.9%25-maroon)
![CMake](https://img.shields.io/badge/CMake-19.2%25-purple)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)

# The PointCloud Tailor
## Advanced Engineering Project subject from _ETSETB_, _Polytechnical University of Catalonia_ (UPC) in collaboration with _Hewlett-Packard_ (HP)

The goal of this project is to explore an extension of the _HP SitePrint_ robot features by adding the ability to check the accuracy of an already built building against specifications by scanning the construction elements using a LiDAR-enabled sensor.

To this end, in this repository there is a program that seeks the correct development of algorithms capable of efficiently aligning consecutive point clouds detected by the LiDAR sensor integrated into the robot based on edges and other geometries of interest, in real-time and with relatively modest hardware, typical of embedded platforms with low memory and processor.

We have developed the algorithms using _C++_ language, and worked mainly with _PCL_ (Point Cloud Library) library, the preferred bridge for 3D applications involving Point Clouds and 3D geometry processing in _ROS_ (Robot Operating System) enviroment, a set of software libraries and tools to build robot applications. In addition, we have benefited from _Matlab_ to carry out various tasks and checks throughout the project.

You will be able to find the following:
- **resources**: User guides for installing and running the program, as well as useful information.
- **rosbags:** Set of files with point cloud data to validate the correct operation of the program.
- **tests:** Tests and prototypes of the different resources implemented in the final version of the project.
- **point_cloud_tailor:** The PointCloud Tailor's final version program.

<p align="center">
  <img src="https://github.com/albert-tomas/PAE-HP/blob/main/point_cloud_tailor/results/aligned_cloud_ceiling_white.png" width="333" />
  <img src="https://github.com/albert-tomas/PAE-HP/blob/main/point_cloud_tailor/results/aligned_cloud_white.png" width="333" />
</p>
