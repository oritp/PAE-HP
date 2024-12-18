#include <deque>
#include <vector> 
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
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
const size_t BUFFER_SIZE = 30;
// Aligned cloud publisher and accumulated cloud object
ros::Publisher pub_aligned_cloud;
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
PointCloudT::Ptr show_cloud(new PointCloudT);
int N_cloud = 0;
Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();


// Function to downsample point clouds
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}


pcl::PointCloud<pcl::Normal>::Ptr computeNormals(PointCloudT::Ptr cloud, float radius) {
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(radius);
    normal_estimation.compute(*normals);

    return normals;
}


PointCloudFPFH::Ptr computeFPFH(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius) {
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    PointCloudFPFH::Ptr fpfh(new PointCloudFPFH());

    fpfh_estimation.setInputCloud(cloud);
    fpfh_estimation.setInputNormals(normals);
    fpfh_estimation.setSearchMethod(tree);
    fpfh_estimation.setRadiusSearch(radius);
    fpfh_estimation.compute(*fpfh);

    return fpfh;
}


PointCloudT::Ptr selectKeypoints(PointCloudT::Ptr cloud, PointCloudFPFH::Ptr fpfh_features, size_t num_keypoints) {
    std::vector<std::pair<float, int>> entropy_indices;

    for (size_t i = 0; i < fpfh_features->size(); ++i) {
        float entropy = 0.0;
        for (float value : fpfh_features->points[i].histogram) {
            if (value > 0) {
                entropy -= value * log(value);
            }
        }
        entropy_indices.emplace_back(entropy, i);
    }

    // Sort by entropy in descending order
    std::sort(entropy_indices.rbegin(), entropy_indices.rend());

    PointCloudT::Ptr keypoints(new PointCloudT);
    for (size_t i = 0; i < std::min(num_keypoints, entropy_indices.size()); ++i) {
        keypoints->points.push_back(cloud->points[entropy_indices[i].second]);
    }

    return keypoints;
}


// Function to accumulate point clouds from the buffer
void updateAccumulatedCloud(double radius) {
    if (point_cloud_buffer.empty()) {
        ROS_WARN("Buffer is empty, skipping update.");
        return;
    }
    
    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    accumulated_cloud->clear();
    // Create a kdtree to look for neighbours in the actual accumulated cloud
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    // Iteraration among clouds in the buffer.
    for (const auto& cloud : point_cloud_buffer) {        
	if (!accumulated_cloud->points.empty()) {
            kdtree.setInputCloud(accumulated_cloud);
        }

        for (const auto& point : cloud->points) {
            if (!accumulated_cloud->points.empty()) {
                std::vector<int> point_index;
                std::vector<float> point_sqrt_distance;

                if (kdtree.radiusSearch(point, radius, point_index, point_sqrt_distance) == 0) {
                    filtered_cloud->points.push_back(point);
                }
            }
            else {
                filtered_cloud->points.push_back(point);
            }
        }
        *accumulated_cloud += *filtered_cloud;
 	if (!accumulated_cloud->points.empty()) {
            kdtree.setInputCloud(accumulated_cloud);
        }
        filtered_cloud->clear();
    }
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    /// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr input_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    // Apply downsampling to reduce computational load
    downsamplePointCloud(input_cloud, 0.2);
    
    // Normals calculation, FPFH computation, Keypoints selection
    auto normals = computeNormals(input_cloud, 0.5);
    auto fpfh_features = computeFPFH(input_cloud, normals, 0.5);
    PointCloudT::Ptr keypoints = selectKeypoints(input_cloud, fpfh_features, 500);
    
    // If no cloud has been detected before, add it to the buffer and update the accumulated one
    if (point_cloud_buffer.empty()) {
        point_cloud_buffer.push_back(input_cloud);
        *accumulated_cloud = *keypoints;
        *show_cloud += *keypoints;
        ROS_INFO("First Point Cloud stored.");
        return;
    }
    
    // Align the clouds by applying ICP efficiently
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(keypoints);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    
    icp.align(*aligned_cloud, initial_transform);
    
    if (icp.hasConverged()) {
        N_cloud ++;
        ROS_INFO("%d: ICP converged, fitness score: %f", N_cloud, icp.getFitnessScore());

        initial_transform = icp.getFinalTransformation();

        // Add the aligned cloud to the buffer
        point_cloud_buffer.push_back(aligned_cloud);

        // If the buffer exceeds the maximum size, delete the oldest cloud
        if (point_cloud_buffer.size() > BUFFER_SIZE) {
            point_cloud_buffer.pop_front();
        }

        // Update the accumulated cloud with those in the buffer (radius=20cm)
        updateAccumulatedCloud(0.1);
       
        ROS_INFO("Accumulated cloud # points: %ld", accumulated_cloud->points.size());
        ROS_INFO("Aligned cloud # points: %ld", aligned_cloud->points.size());
        
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
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 400, pointCloudCallback);
    ROS_INFO("Point Cloud Subscriber is ready.");
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 400);
    ROS_INFO("Point Cloud Publisher is ready.");

    ros::spin();
    
    return EXIT_SUCCESS;
}

