#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>  // C++17 and later
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace fs = std::filesystem;

void voxelGrid(const std::string& filename) {
   // std::cout << "Success" << endl;

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  std::string input = "/home/albert/Desktop/pcds/pcds_full/" + filename;
  reader.read (input, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write (filename + "_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);

  std::string name = "mv " + filename + "_downsampled.pcd /home/albert/Desktop/pcds/pcds_downsampled";
  (void)std::system(name.c_str());

 // return (0);

}

int main() {
    std::string folder_path = "/home/albert/Desktop/pcds/pcds_full";
    //std::cout << "Enter folder path: ";
    //std::getline(std::cin, folder_path);  // Get the folder path from the user

    std::vector<std::string> file_names;
    // Iterate through files in the directory
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            file_names.push_back(entry.path().filename().string());
        }
    }

    // Sort the file names in alphabetical order
    std::sort(file_names.begin(), file_names.end());

    // Display sorted file names
    /*std::cout << "Files in alphabetical order:\n";
    for (const auto& file_name : file_names) {
        std::cout << file_name << '\n';
    }*/

    for (int i = 0; i < file_names.size(); i++) {
        voxelGrid(file_names[i]);
    }

    return 0;
}