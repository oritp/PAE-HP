#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


// Image width and height
#define WIDTH 1280
#define HEIGHT 720

// Point cloud max and min height
const float MIN_HEIGHT = -2.0;
float MAX_HEIGHT = 1.7;

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

PointCloudT::Ptr cloud(new PointCloudT);
PointCloudT::Ptr filtered_cloud(new PointCloudT);

// Struct to represent an RGB pixel in BMP format
struct Pixel {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};


// Function to filter by height
void filterByHeight(const PointCloudT::Ptr& input_cloud, PointCloudT::Ptr& output_cloud, float min_height, float max_height) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*output_cloud);
}


// Función para guardar una matriz de píxeles como una imagen BMP
void saveAsBMP(const std::string &filename, const std::vector<std::vector<Pixel>> &image) {
    std::ofstream file(filename, std::ios::binary);

    // BMP format header (54 bytes)
    unsigned char header[54] = {
        'B', 'M',            // BMP file format identifier
        0, 0, 0, 0,          // File size (computed later)
        0, 0, 0, 0,          // Reserved
        54, 0, 0, 0,         // Data offset of the image
        40, 0, 0, 0,         // Information header size
        static_cast<unsigned char>(WIDTH),         // Image width
        static_cast<unsigned char>(WIDTH >> 8), 
        static_cast<unsigned char>(WIDTH >> 16), 
        static_cast<unsigned char>(WIDTH >> 24), 
        static_cast<unsigned char>(HEIGHT),        // Image height
        static_cast<unsigned char>(HEIGHT >> 8), 
        static_cast<unsigned char>(HEIGHT >> 16), 
        static_cast<unsigned char>(HEIGHT >> 24), 
        1, 0,                // Maps (1)
        24, 0,               // Bits per pixel (24 for RGB)
        0, 0, 0, 0,          // Compression (0 = no compression)
        0, 0, 0, 0,          // Data size of the image (computed later)
        0, 0, 0, 0,          // Horizontal resolution (ignore)
        0, 0, 0, 0,          // Vertical resolution (ignore)
        0, 0, 0, 0,          // Colors on the palette (0 = all)
        0, 0, 0, 0           // Important colors (0 = all)
    };

    int row_padded = (WIDTH * 3 + 3) & (~3);
    int file_size = 54 + row_padded * HEIGHT;

    // Fill file size in the header
    header[2] = file_size;
    header[3] = file_size >> 8;
    header[4] = file_size >> 16;
    header[5] = file_size >> 24;

    // Write the file header
    file.write(reinterpret_cast<char *>(header), 54);

    // Write the data on the image
    for (int y = HEIGHT - 1; y >= 0; --y) {
        for (int x = 0; x < WIDTH; ++x) {
            file.write(reinterpret_cast<const char *>(&image[y][x]), 3);
        }
        // Padding for each row (if it is necessary)
        for (int p = 0; p < row_padded - WIDTH * 3; ++p) {
            file.put(0);
        }
    }

    file.close();
}

int main(int argc, char **argv) {
    // Check that the arguments are provided correctly
    if (argc < 2 || argc > 3) {
        std::cerr << "ERROR: Correct line: " << argv[0] << " <cloud_name.pcd> [MAX_HEIGHT]\n";
        return EXIT_FAILURE;
    }

    // Check that the maximum height is provided correctly and then assign it
    if (argc == 3) {
        std::stringstream ss(argv[2]);
        if (!(ss >> MAX_HEIGHT)) {
            std::cerr << "ERROR: MAX_HEIGHT must be a valid number.\n";
            return EXIT_FAILURE;
        }
    }
    
    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr filtered_cloud(new PointCloudT);

    // Load the point cloud as .pcd file
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
        std::cerr << "ERROR: Cannot load this .pcd file.\n";
        return EXIT_FAILURE;
    }

    // Filter the cloud to not show the ceiling
    filterByHeight(cloud, filtered_cloud, MIN_HEIGHT, MAX_HEIGHT);

    // Find the point cloud limits
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto &point : cloud->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
    }

    // Compute the point cloud center
    float center_x = (min_x + max_x) / 2;
    float center_y = (min_y + max_y) / 2;

    // Compute the scale to center the point cloud in the image
    float scale_x = WIDTH / (max_x - min_x);
    float scale_y = HEIGHT / (max_y - min_y);
    float scale = std::min(scale_x, scale_y);
    
    // Create a white background 2D image (255, 255, 255)
    std::vector<std::vector<Pixel>> image(HEIGHT, std::vector<Pixel>(WIDTH, {255, 255, 255}));

    // Project points onto the XY plane and mark them in the image
    for (const auto &point : filtered_cloud->points) {
        // Center and scale the cloud on the image
        int x = static_cast<int>((point.x - center_x) * scale + WIDTH / 2);
        int y = static_cast<int>(-(point.y - center_y) * scale + HEIGHT / 2);
        // Write the points in black (0, 0, 0)
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) image[y][x] = {0, 0, 0}; 
    }

    // Save the image as .bmp file
    saveAsBMP("aligned_cloud_map.bmp", image);
    std::cout << "SUCCESS: 2D Point Cloud Map saved as 'aligned_cloud_map.bmp'\n";

    return EXIT_SUCCESS;
}
