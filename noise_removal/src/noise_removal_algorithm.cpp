
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
// #include <pcl/filters/voxel_grid.h>

// static ros::Publisher pub_;

int main(int argc, char **argv)
{

    /*// Create a PointCloud object to hold the data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the .pcd file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/albert/Downloads/table_scene_lms400.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file \n");
        return -1;
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from the .pcd file with the following fields: "
              << std::endl;

    // Iterate over points and print them (optional)
    for (const auto& point : cloud->points)
        std::cout << "    " << point.x
                  << " " << point.y
                  << " " << point.z << std::endl;

     std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from the .pcd file with the following fields: "
              << std::endl;*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;

    // RPath with the saved file
    reader.read<pcl::PointXYZ>("/home/albert/Downloads/src (copy)/1728910136.386327744.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return 0;
}
