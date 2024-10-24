#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> prev_cloud;
pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> current_cloud;
    pcl::fromROSMsg(*cloud_msg, current_cloud);

	// If there is not previous cloud, save and exit
    if (prev_cloud.points.empty()) {
        // Add the first accumulated cloud and publish
        prev_cloud = current_cloud;
        accumulated_cloud += current_cloud;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(accumulated_cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub.publish(output_msg);
        return;
    }

    // Apply ICP between current and previous cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud.makeShared());
    icp.setInputTarget(prev_cloud.makeShared());

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);

	// If ICP has converged publish the accumulated cloud, if not warn
    if (icp.hasConverged()) {
        ROS_INFO_STREAM("ICP converged with score: " << icp.getFitnessScore());
	
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	global_transform = global_transform * transformation;
	pcl::transformPointCloud(current_cloud, current_cloud, global_transform);
	
	// Add the point cloud to the acuumulated one
        accumulated_cloud += aligned_cloud;
		
	// Publish the accumulated cloud
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(accumulated_cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub.publish(output_msg);
    }
    else {
        ROS_WARN("ICP did not converge!");
    }

    // Update the previous cloud
    prev_cloud = current_cloud;
}

int main(int argc, char** argv) {
	// Initialize the ROS node
    ros::init(argc, argv, "icp_mapping");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // Create the ROS Subscriber and subscribe to the Point Cloud
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloud_callback);
    ROS_INFO("Point Cloud subscriber ready!");
    
    // Create the ROS Publisher and publish the accumulate cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 10);
    ROS_INFO("Point Cloud publisher ready!");
    
    // Initialize the accumulated cloud
    accumulated_cloud.clear();
    
    ros::spin();
    
    return EXIT_SUCCESS;
}

