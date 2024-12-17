#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>
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
ros::Publisher pub_transform;

PointCloudT::Ptr input_cloud(new PointCloudT);
PointCloudT::Ptr accumulated_cloud(new PointCloudT);
Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();

bool IS_FIRST_CLOUD = true;
bool SHOW_CEILING = false;
bool PUBLISH_CLOUD = true;
float MIN_HEIGHT = -2.0f;
float MAX_HEIGHT = 1.7f;
int N_CLOUD = 1;


// Function to downsample point clouds
void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}


// Function to filter by height and extract the ceiling
void filterByHeight(const PointCloudT::Ptr& input_cloud, PointCloudT::Ptr& output_cloud, float MIN_HEIGHT, float MAX_HEIGHT) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(MIN_HEIGHT, MAX_HEIGHT);
    pass.filter(*output_cloud);
}


// Function to filter redundant points based on proximity
void filterRedundantPoints(PointCloudT::Ptr& new_cloud, const PointCloudT::Ptr& accumulated_cloud, float radius) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(accumulated_cloud);

    PointCloudT::Ptr filtered_cloud(new PointCloudT);

    for (const auto& point : new_cloud->points) {
        std::vector<int> point_indices;
        std::vector<float> point_squared_distances;
        if (kdtree.radiusSearch(point, radius, point_indices, point_squared_distances) == 0) {
            filtered_cloud->points.push_back(point);
        }
    }
    *new_cloud = *filtered_cloud;
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    /// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *cloud);
    
    filterByHeight(cloud, input_cloud, MIN_HEIGHT, MAX_HEIGHT);
    
    // Apply downsampling to reduce computational load
    downsamplePointCloud(input_cloud, 0.2);
    
    // If it is the first cloud update the accumulated one with it
    if (IS_FIRST_CLOUD) {
        *accumulated_cloud = *input_cloud;
        IS_FIRST_CLOUD = false;
        ROS_INFO("First Point Cloud stored.");
        return;
    }

    // Align the clouds by applying ICP efficiently
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-6);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    
    icp.align(*aligned_cloud, initial_transform);
    
    if (icp.hasConverged()) {
        N_CLOUD ++;

        initial_transform = icp.getFinalTransformation();
        
        // Adapt initial transformation to Float32MultiArray format
        std_msgs::Float32MultiArray transform_msgs;
        transform_msgs.data.resize(16);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform_msgs.data[i * 4 + j] = initial_transform(i, j);
            }
        }
        
        pub_transform.publish(transform_msgs);
       
        filterRedundantPoints(aligned_cloud, accumulated_cloud, 0.05f);
        
        ROS_INFO("%d: ICP converged, fitness score: %f", N_CLOUD, icp.getFitnessScore());
        
        *accumulated_cloud += *aligned_cloud;

        // Publish the updated accumulated cloud
        if (PUBLISH_CLOUD) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*accumulated_cloud, output_msg);
            output_msg.header.frame_id = "map";
            pub_aligned_cloud.publish(output_msg);
        }
    } 
    else {
        ROS_WARN("ICP did not converge.");
    }
}


// Service callback for saving the point cloud
bool savePointCloudCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
     std::string package_path = ros::package::getPath("point_cloud_tailor");
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
    private_nh.param("show_ceiling", SHOW_CEILING, false);
    MAX_HEIGHT = (SHOW_CEILING) ? 3.0f : 1.7f;
    private_nh.param("publish_cloud", PUBLISH_CLOUD, true);
    
    // Subscription to the input topic and publication of the aligned result
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 200, pointCloudCallback);
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 200);
    ROS_INFO("Point Cloud Alignment Node is ready.");
    
    pub_transform = nh.advertise<std_msgs::Float32MultiArray>("transformation", 200);

    // Advertise the service to save point cloud
    ros::ServiceServer service = nh.advertiseService("save_pcd", savePointCloudCallback);
    
    ros::spin();
    
    return EXIT_SUCCESS;
}
