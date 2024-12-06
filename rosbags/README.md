# Rosbags

Here you can find a set of files with point cloud data in `.bag` format to test the correct operation of the program.

The bag file format is very efficient for both recording and replaying messages in ROS, as it works similar to how ROS nodes subscribe and publish via ROS topics. With these files we can collect the point clouds captured by the LiDAR sensor at a given moment, and then reproduce them to simulate that we are doing it in real time.

The point cloud bags that you can find here are the following:
- **livox_room.bag**
- [**mid40_hall_example.bag**](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_hall_example.bag) _(click to download)_

To run these bags it is necessary to first decompress them and then to have the ROS master active and ensure that the main node subscribes to the bag, _/livox/lidar_ in our case. Next we execute the following command in the Linux terminal: 

    rosbag play bag_name.bag
