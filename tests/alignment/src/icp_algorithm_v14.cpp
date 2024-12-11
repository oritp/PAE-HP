#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

ros::Publisher pub_aligned_cloud;
PointCloudT::Ptr input_cloud(new PointCloudT);
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
bool is_first_cloud = true;
int N_cloud = 1;
float min_height = -2.0f;
float max_height = 1.7f;
bool show_ceiling = false;


// Function to downsample point clouds
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}


// Function to filter by height and extract the ceiling
void filterByHeight(const PointCloudT::Ptr& input_cloud, PointCloudT::Ptr& output_cloud, float min_height, float max_height) {
    pcl::PassThrough<PointT> pass;
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
    
    filterByHeight(cloud, input_cloud, min_height, max_height);
    
    // Apply downsampling to reduce computational load
    downsamplePointCloud(input_cloud, 0.15);
    
    // If it is the first cloud update the accumulated one with it
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

        initial_transform = icp.getFinalTransformation();
       
        filterRedundantPoints(aligned_cloud, accumulated_cloud, 0.05f);
        
        ROS_INFO("%d: ICP converged, fitness score: %f", N_cloud, icp.getFitnessScore());       
        ROS_INFO("Accumulated cloud points: %ld", accumulated_cloud->points.size());
        ROS_INFO("Aligned cloud points: %ld", aligned_cloud->points.size());
        
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


// Service callback for saving the point cloud
bool savePointCloudCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
     std::string package_path = ros::package::getPath("alignment");
     std::string file_path = package_path + "/results/aligned_cloud.pcd";
     if (pcl::io::savePCDFileASCII(file_path, *accumulated_cloud) == 0) {
        res.success = true;
        res.message = "Point cloud saved successfully.";
        ROS_INFO("Point cloud saved successfully.");
    } else {
        res.success = false;
        res.message = "Failed to save point cloud.";
        ROS_ERROR("Failed to save point cloud.");
    }
    return true;
}


int main(int argc, char** argv) {
    // ROS node initialization
    ros::init(argc, argv, "alignment_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // Get the parameters from the launch file
    private_nh.param("show_ceiling", show_ceiling, false);
    max_height = (show_ceiling) ? 3.0f : 1.7f;
    
    // Subscription to the input topic and publication of the aligned result
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 400, pointCloudCallback);
    ROS_INFO("Point Cloud Subscriber is ready.");
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 400);
    ROS_INFO("Point Cloud Publisher is ready.");
    
    // Advertise the service to save point cloud
    ros::ServiceServer service = nh.advertiseService("save_pcd", savePointCloudCallback);
    ROS_INFO("Service save_pcd ready.");
    
    ros::spin();
    
    return EXIT_SUCCESS;
}
