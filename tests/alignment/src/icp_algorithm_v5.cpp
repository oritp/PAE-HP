#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transforms.h>


using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
ros::Publisher pub_aligned_cloud;

PointCloudT::Ptr accumulated_cloud(new PointCloudT);
Eigen::Matrix4f fixed_transformation = Eigen::Matrix4f::Identity();
bool is_first_cloud = true;
bool is_first_aligned_cloud = true;

void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size = 0.05) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}

void extractFPFHFeatures(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features) {
    // Normals estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    // FPFH estimation
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.1);
    fpfh.compute(*fpfh_features);
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    PointCloudT::Ptr input_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    downsamplePointCloud(input_cloud);
    
    if (is_first_cloud) {
        *accumulated_cloud = *input_cloud;
        is_first_cloud = false;
        ROS_INFO("Primera nube almacenada.");
        return;
    }
    
     pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    extractFPFHFeatures(input_cloud, source_features);
    extractFPFHFeatures(accumulated_cloud, target_features);

    // ICP with FPFH feature extraction
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    est.determineCorrespondences(*correspondences);

    // Filter correspondences with RANSAC
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(input_cloud);
    rejector.setInputTarget(accumulated_cloud);
    rejector.setInputCorrespondences(correspondences);
    pcl::Correspondences inliers;
    rejector.getCorrespondences(inliers);

    // ICP
    icp.setInputSource(input_cloud);
    icp.setInputTarget(accumulated_cloud);
    //icp.setMaximumIterations(100);
    //icp.setMaxCorrespondenceDistance(0.2);
    //icp.setTransformationEpsilon(1e-6);
    
    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    icp.align(*aligned_cloud);

    if (icp.hasConverged()) {
        ROS_INFO("ICP ha convergido con fitness score: %f", icp.getFitnessScore());

        Eigen::Matrix4f current_transformation = icp.getFinalTransformation();

        if (is_first_aligned_cloud) {
            fixed_transformation = current_transformation;
            is_first_aligned_cloud = false;
        }

        pcl::transformPointCloud(*aligned_cloud, *aligned_cloud, fixed_transformation.inverse() * current_transformation);

        // Update the accumulated cloud and publish
        *accumulated_cloud += *aligned_cloud;
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub_aligned_cloud.publish(output_msg);
    } 
    else {
        ROS_WARN("ICP no ha convergido.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_pointcloud_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 100, pointCloudCallback);
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 100);

    ros::spin();
    
    return EXIT_SUCCESS;
}

