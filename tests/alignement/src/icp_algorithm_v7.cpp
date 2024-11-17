#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudFPFH = pcl::PointCloud<pcl::FPFHSignature33>;

// Circular buffer and its size, to store the last point clouds
std::deque<PointCloudT::Ptr> point_cloud_buffer;
const size_t BUFFER_SIZE = 10;
// Aligned cloud publisher and accumulated cloud object
ros::Publisher pub_aligned_cloud;
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
PointCloudT::Ptr show_cloud(new PointCloudT);

// Function to downsample point clouds
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size = 0.2) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}

// Function to accumulate point clouds from the buffer
void updateAccumulatedCloud() {
    accumulated_cloud->clear();
    for (const auto& cloud : point_cloud_buffer) {
        *accumulated_cloud += *cloud;
    }
}

// Function to extract FPFH features (Fast Point Feature Histograms)
void extractFPFHFeatures(const PointCloudT::Ptr& cloud, PointCloudFPFH::Ptr& fpfh_features) {
    // Normals estimation
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    // Extract features
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.1);
    fpfh.compute(*fpfh_features);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    /// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr input_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    // Apply downsampling to reduce computational load
    downsamplePointCloud(input_cloud);
    
    // If no cloud has been detected before, add it to the buffer and update the accumulated one
    if (point_cloud_buffer.empty()) {
        point_cloud_buffer.push_back(input_cloud);
        updateAccumulatedCloud();
        *show_cloud += *input_cloud;
        ROS_INFO("First Point Cloud stored.");
        return;
    }
    
    // Extract FPFH features for input and accumulated cloud
    PointCloudFPFH::Ptr source_features(new PointCloudFPFH);
    PointCloudFPFH::Ptr target_features(new PointCloudFPFH);
    extractFPFHFeatures(input_cloud, source_features);
    extractFPFHFeatures(accumulated_cloud, target_features);

    // Configure ICP (Iterative Closest Point) with FPFH based correspondences
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimator;
    estimator.setInputSource(source_features);
    estimator.setInputTarget(target_features);
    
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    estimator.determineCorrespondences(*correspondences);

    // Filter correspondences using RANSAC (Random Sample Consensus)
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
    rejector.setInputSource(input_cloud);
    rejector.setInputTarget(accumulated_cloud);
    rejector.setInputCorrespondences(correspondences);
    pcl::Correspondences inliers;
    rejector.getCorrespondences(inliers);
    
    // Align the clouds by applying ICP efficiently
    icp.setInputSource(input_cloud);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    icp.align(*aligned_cloud);
    
    if (icp.hasConverged() && icp.getFitnessScore() > 0.09) {
        ROS_INFO("ICP exceeds fitness score.");
    }

    else if (icp.hasConverged() && icp.getFitnessScore() < 0.09) {
        ROS_INFO("ICP converged, fitness score: %f", icp.getFitnessScore());

        // Add the aligned cloud to the buffer
        point_cloud_buffer.push_back(aligned_cloud);

        // If the buffer exceeds the maximum size, delete the oldest cloud
        if (point_cloud_buffer.size() > BUFFER_SIZE) {
            point_cloud_buffer.pop_front();
        }

        // Update the accumulated cloud with those in the buffer
        updateAccumulatedCloud();
        
        // Update the showing cloud
        *show_cloud += *accumulated_cloud;

        // Publish the updated accumulated cloud
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*show_cloud, output_msg);
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
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 100, pointCloudCallback);
    ROS_INFO("Point Cloud Subscriber is ready.");
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 100);
    ROS_INFO("Point Cloud Publisher is ready.");

    ros::spin();
    
    return EXIT_SUCCESS;
}

