#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rosbag/bag.h>  // Include rosbag for recording

static ros::Publisher pub_;

// Parameters of the distance filter: squared distances!! (x^2)
float minDistance2 = 0.0; // 0m
float maxDistance2 = 4.0; // 2m 

// Create a rosbag object to write filtered data
rosbag::Bag bag;

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO_ONCE("First sensor image received!");

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Perform the distance filtering
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    cloud_filtered.points.resize(cloud.points.size());
    for (size_t p = 0; p < cloud.points.size(); p++)
    {
        // Do not compute sqrt() to avoid unnecessary computation
        float pointDepth2 = (cloud.points[p].x * cloud.points[p].x) +
                            (cloud.points[p].y * cloud.points[p].y) +
                            (cloud.points[p].z * cloud.points[p].z);

        // Keep point if it's within the threshold range
        if (pointDepth2 >= minDistance2 && pointDepth2 <= maxDistance2)
        {
            cloud_filtered.push_back(cloud.points[p]);
        }
    }

    // Convert back to ROS sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(cloud_filtered, output_msg);
    output_msg.header = cloud_msg->header;

    // Publish the data for visualization
    pub_.publish(output_msg);

    // Save the filtered point cloud to the rosbag
    bag.write("/filtered_point_cloud", ros::Time::now(), output_msg);
    
    ROS_INFO("TEst1");
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "example1_to_rosbag");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Read the external filter parameters
    private_nh.param<float>("min_distance", minDistance2, 0.0);
    private_nh.param<float>("max_distance", maxDistance2, 2.0);

    // Square the parameters to avoid using root sqrt()
    minDistance2 = minDistance2 * minDistance2;
    maxDistance2 = maxDistance2 * maxDistance2;

    // Open the rosbag file
    bag.open("/home/albert/filtered_lidar_data.bag", rosbag::bagmode::Write);  // Specify the path to your rosbag

    // Create the ROS subscriber for reading LiDAR images
    ROS_INFO("Creating sensor subscriber");
    ros::Subscriber sub_ = nh.subscribe("/livox/lidar", 1, lidar_callback);

    // Create a ROS publisher for the output point cloud
    ROS_INFO("Creating filter publisher");
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 10);
    
    ROS_INFO("Test2");

    ROS_INFO("Node ready for filtering");

    
    ros::Rate loop_rate(10); // Adjust rate as needed
    int count = 0;
    while (ros::ok() && count < 1000)  // Process for 100 iterations (or set your own condition)
    {
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    
    // Close the rosbag file when done
    bag.close();
    ROS_INFO("test4");

    return EXIT_SUCCESS;
}
