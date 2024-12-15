#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

ros::Publisher pub_aligned_cloud;
ros::Publisher pub_trajectory;

PointCloudT::Ptr input_cloud(new PointCloudT);
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
nav_msgs::Path path;
bool is_first_cloud = true;

// Downsample point cloud
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}

// Filter points by height
void filterByHeight(const PointCloudT::Ptr& input_cloud, PointCloudT::Ptr& output_cloud, float min_height, float max_height) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*output_cloud);
}

// Point cloud callback
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *cloud);

    filterByHeight(cloud, input_cloud, -2, 1.7);
    downsamplePointCloud(input_cloud, 0.15);

    if (is_first_cloud) {
        *accumulated_cloud = *input_cloud;
        is_first_cloud = false;
        ROS_INFO("First cloud stored.");
        return;
    }

    // Align using ICP
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    icp.align(*aligned_cloud, initial_transform);

    if (icp.hasConverged()) {
        ROS_INFO("ICP converged. Fitness score: %f", icp.getFitnessScore());
        initial_transform = icp.getFinalTransformation();

        *accumulated_cloud += *aligned_cloud;

        // Publish accumulated cloud
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub_aligned_cloud.publish(output_msg);

        // Publish the transformation as part of the trajectory
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        // Extract translation and rotation
        Eigen::Matrix3f rotation = initial_transform.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);

        pose.pose.position.x = initial_transform(0, 3);
        pose.pose.position.y = initial_transform(1, 3);
        pose.pose.position.z = initial_transform(2, 3);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        pub_trajectory.publish(path);
    } else {
        ROS_WARN("ICP did not converge.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "combined_node");
    ros::NodeHandle nh;

    // Subscribers and publishers
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 400, pointCloudCallback);
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 400);
    pub_trajectory = nh.advertise<nav_msgs::Path>("trajectory", 10);

    path.header.frame_id = "map";

    ROS_INFO("Combined node ready.");
    ros::spin();
    return 0;
}
