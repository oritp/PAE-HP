
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>

static ros::Publisher pub_;

pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Parameters of the distance filter: squared distances!! (x^2)
float minDistance2 = 0.0; // 0m
float maxDistance2 = 4.0; // 2m 

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO_ONCE("First sensor image received!");

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Perform the distance filtering
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    for (size_t p = 0; p < cloud.points.size(); p++)
    {
        // Do not compute sqrt() to avoid unnecessary computation
        float pointDepth2 = (cloud.points[p].x * cloud.points[p].x) +
                            (cloud.points[p].y * cloud.points[p].y) +
                            (cloud.points[p].z * cloud.points[p].z);

        // Keep point if it's within the threshold range
        if (pointDepth2 >= minDistance2 && pointDepth2 <= maxDistance2)
        {
            bool is_new_point = true;
            for (const auto& accumulated_point : accumulated_cloud->points)
            {
                float dist2 = std::pow(cloud.points[p].x - accumulated_point.x, 2) +
                              std::pow(cloud.points[p].y - accumulated_point.y, 2) +
                              std::pow(cloud.points[p].z - accumulated_point.z, 2);
                if (dist2 < 0.01) // Umbral 1cm
                {
                    is_new_point = false;
                    break;
                }
            }
            if (is_new_point)
            {
                cloud_filtered.push_back(cloud.points[p]);
            }
        }
    }

    *accumulated_cloud += cloud_filtered;

    // Publish the data for visualization
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*accumulated_cloud, output_msg);
    output_msg.header.frame_id = cloud_msg->header.frame_id;
    pub_.publish(output_msg);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "example2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Read the external filter parameters
    private_nh.param<float>("min_distance", minDistance2, 0.0);
    private_nh.param<float>("max_distance", maxDistance2, 5.0);

    // Square the parameters to avoid using root sqrt()
    minDistance2 = minDistance2 * minDistance2;
    maxDistance2 = maxDistance2 * maxDistance2;

    // Create the ROS subscriber for reading LiDAR images
    ros::Subscriber sub_ = nh.subscribe("/livox/lidar", 1, lidar_callback);

    // Create a ROS publisher for the output point cloud
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

    ROS_INFO("Node ready for filtering");

    accumulated_cloud->clear();

    ros::spin();

    return EXIT_SUCCESS;
}

