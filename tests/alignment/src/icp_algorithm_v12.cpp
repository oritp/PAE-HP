#include <deque>
#include <vector> 
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudFPFH = pcl::PointCloud<pcl::FPFHSignature33>;

// Aligned cloud publisher and accumulated cloud object
ros::Publisher pub_aligned_cloud;
PointCloudT::Ptr input_cloud(new PointCloudT);
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
PointCloudT::Ptr show_cloud(new PointCloudT);
int N_cloud = 0;
Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
bool is_first_cloud = true;


// Function to downsample point clouds
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}


void filterByHeight(const PointCloudT::Ptr& input_cloud, PointCloudT::Ptr& output_cloud, float min_height, float max_height) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*output_cloud);
}


// Function to filter redundant points based on proximity
void filterRedundantPoints(PointCloudT::Ptr& new_cloud, const PointCloudT::Ptr& accumulated_cloud, float distance_threshold) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(accumulated_cloud);

    PointCloudT::Ptr filtered_cloud(new PointCloudT);

    for (const auto& point : new_cloud->points) {
        std::vector<int> point_indices;
        std::vector<float> point_squared_distances;
        if (kdtree.radiusSearch(point, distance_threshold, point_indices, point_squared_distances) == 0) {
            filtered_cloud->points.push_back(point);
        }
    }
    *new_cloud = *filtered_cloud;
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    /// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *cloud);
    
    filterByHeight(cloud, input_cloud, -2, 1.8);
    
    // Apply downsampling to reduce computational load
    downsamplePointCloud(input_cloud, 0.15);
    
    // If no cloud has been detected before, add it to the buffer and update the accumulated one
    if (is_first_cloud) {
        *accumulated_cloud = *input_cloud;
        is_first_cloud = false;
        ROS_INFO("First Point Cloud stored.");
        return;
    }
    
    // Align the clouds by applying ICP efficiently
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    
    icp.align(*aligned_cloud, initial_transform);
    
    if (icp.hasConverged()) {
        N_cloud ++;
        ROS_INFO("%d: ICP converged, fitness score: %f", N_cloud, icp.getFitnessScore());

        initial_transform = icp.getFinalTransformation();
       
        filterRedundantPoints(aligned_cloud, accumulated_cloud, 0.08f);
       
        ROS_INFO("Accumulated cloud # points: %ld", accumulated_cloud->points.size());
        ROS_INFO("Aligned cloud # points: %ld", aligned_cloud->points.size());
        
        // Update the showing cloud
        *accumulated_cloud += *aligned_cloud;

        // Publish the updated accumulated cloud
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub_aligned_cloud.publish(output_msg);
    } 
    else {
        ROS_WARN("ICP did not converge.");
    }
}


int main(int argc, char** argv) {
    // ROS node initialization
    ros::init(argc, argv, "icp_pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // Subscription to the input topic and publication of the aligned result
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 400, pointCloudCallback);
    ROS_INFO("Point Cloud Subscriber is ready.");
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 400);
    ROS_INFO("Point Cloud Publisher is ready.");

    ros::spin();
    
    return EXIT_SUCCESS;
}
