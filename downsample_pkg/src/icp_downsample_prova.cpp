#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>  // C++17 and later
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>


namespace fs = std::filesystem;
// Parameters of the distance filter: squared distances!! (x^2)
float minDistance2 = 0.3; // 0m
float maxDistance2 = 100.0; // 2m 

//Read all files in a folder and returns a vector of the names (strings)
std::vector<std::string> readFolder(){
    std::string folder_path = "/home/albert/Desktop/pcds/pcds_downsampled";

    std::vector<std::string> file_names;
    // Iterate through files in the directory
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename().string());
        }
    }

    // Sort the file names in alphabetical order
    std::sort(file_names.begin(), file_names.end());

    return file_names;
}

//Views the selected pcd in PCL viewer
void openPCLViewer(const std::string& fileName){
    std::string command = "LIBGL_ALWAYS_SOFTWARE=1 pcl_viewer " + fileName + " &";
    int result = std::system(command.c_str()); // Execute the command
}

pcl::PointCloud<pcl::PointXYZ> loadPCD(const std::string& fileName){
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, cloud) == -1) { // Load the PCD file
        PCL_ERROR("Couldn't read the PCD file\n");
        return pcl::PointCloud<pcl::PointXYZ>();  // Return an empty cloud on failure
    }

    // Perform the distance filtering
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    cloud_filtered.points.resize(cloud.points.size());
    for (size_t p=0; p < cloud.points.size(); p++)
    {
        // Do not compute sqrt() to avoid unnecessary computation
        float pointDepth2 = (cloud.points[p].x * cloud.points[p].x) +
                            (cloud.points[p].y * cloud.points[p].y) +
                            (cloud.points[p].z * cloud.points[p].z);

        // Keep point if it's within the threshold range
        if (pointDepth2 >= minDistance2 && pointDepth2 <= maxDistance2)
        {
            cloud_filtered.push_back(cloud.points[p]);
        }
    }

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ> icpAlgorithm(const pcl::PointCloud<pcl::PointXYZ>& currentCloud, pcl::PointCloud<pcl::PointXYZ>& accumulatedCloud){

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(currentCloud.makeShared());
    icp.setInputTarget(accumulatedCloud.makeShared());
    
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (1000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-5);

    // Perform the alignment and store the result in outputCloud
    pcl::PointCloud<pcl::PointXYZ> outputCloud;
    icp.align(outputCloud);
 
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();    

    return outputCloud;
}

int main() {

    //If we are not inside the folder with the pcds, the pcl_viewer crashes
    fs::current_path("/home/albert/Desktop/pcds/pcds_downsampled");

    //Store the names of all the downsapled pcds in order (vector of strings)
    std::vector<std::string> file_names = readFolder();

    const int X = 10;  // Define the number of recent clouds to keep
    std::deque<pcl::PointCloud<pcl::PointXYZ>> recentClouds;

    pcl::PointCloud<pcl::PointXYZ> currentCloud, alignedCloud, accumulatedCloud;

    //cloud1 = loadPCD(file_names[0]);
    //cloud2 = loadPCD(file_names[1]);
    alignedCloud = loadPCD(file_names[0]);
    accumulatedCloud = alignedCloud;
    recentClouds.push_back(alignedCloud);


    //for(int i=1; i<71; i++){
    for(int i=1; i<file_names.size(); i++){
        currentCloud = loadPCD(file_names[i]);

        // Step 1: Create an aligned combination of recent clouds
        pcl::PointCloud<pcl::PointXYZ> recentCombined = *recentClouds.front().makeShared();
        for (size_t j = 0; j < recentClouds.size(); ++j) {
            recentCombined = icpAlgorithm(recentClouds[j], recentCombined);
            //std::cout << recentClouds.size() << std::endl;
        }

        alignedCloud = icpAlgorithm(currentCloud, recentCombined);
        recentClouds.push_back(alignedCloud);  // Add the aligned cloud to the deque
        accumulatedCloud += alignedCloud;
        //std::cout << recentClouds.size() << std::endl;
        // If the deque exceeds size X, remove the oldest cloud
        if (recentClouds.size() > X) {
            recentClouds.pop_front();
            //std::cout << recentClouds.size() << std::endl;
        }

        std::cout << i << std::endl;

        if (i % 10 == 0) {
            pcl::io::savePCDFileASCII("/home/albert/Desktop/pcds/pcds_after_icp/intermediate_" + std::to_string(i) + ".pcd", accumulatedCloud);
        }

    }
    

    //cloud2 = icpAlgorithm(cloud1, cloud2);

    //View the first pcd file
    //openPCLViewer(file_names[0]);
    //openPCLViewer(file_names[1]);


    //We save the aligned point cloud
    ///home/albert/Desktop/pcds/pcds_after_icp

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoise(new pcl::PointCloud<pcl::PointXYZ>(accumulatedCloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    //cloudNoise = accumulatedCloud;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloudNoise);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // Save the cloud to a PCD file
    if (pcl::io::savePCDFileASCII("/home/albert/Desktop/pcds/pcds_after_icp/output.pcd", *cloud_filtered) == -1) {
        PCL_ERROR("Failed to save the point cloud to a .pcd file\n");
        return -1;
    }
   
    fs::current_path("/home/albert/Desktop/pcds/pcds_after_icp");
    openPCLViewer("output.pcd");
    

    return 0;

}