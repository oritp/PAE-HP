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
#include <pcl/features/principal_curvatures.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
ros::Publisher pub_aligned_cloud;

// Parámetros globales
PointCloudT::Ptr accumulated_cloud(new PointCloudT);  // Nube acumulada para alineación
Eigen::Matrix4f fixed_transformation = Eigen::Matrix4f::Identity();
bool is_first_cloud = true;
bool is_first_aligned_cloud = true;

void downsamplePointCloud(PointCloudT::Ptr& cloud, float leaf_size = 0.05) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
}

void extractFeatures(PointCloudT::Ptr cloud, PointCloudT::Ptr corners, PointCloudT::Ptr planes) {
    // Estimación de normales
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // Estimación de curvaturas principales
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pce.setInputCloud(cloud);
    pce.setInputNormals(normals);
    pce.setRadiusSearch(0.1);
    pce.compute(*curvatures);

    // Clasificación en esquinas y planos según la curvatura
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (curvatures->points[i].pc1 > 0.5) {
            corners->points.push_back(cloud->points[i]);
        } else if (curvatures->points[i].pc1 < 0.2) {
            planes->points.push_back(cloud->points[i]);
        }
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Convertir la nube ROS a formato PCL
    PointCloudT::Ptr input_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    // Filtrar la nube para reducir la carga computacional
    downsamplePointCloud(input_cloud);
    
    if (is_first_cloud) {
        // Inicializar la nube acumulada con la primera nube recibida
        *accumulated_cloud = *input_cloud;
        is_first_cloud = false;
        ROS_INFO("Primera nube almacenada.");
        return;
    }
    
    PointCloudT::Ptr corners(new PointCloudT);
    PointCloudT::Ptr planes(new PointCloudT);
    extractFeatures(input_cloud, corners, planes);
    
    // Aplicar ICP solo con esquinas y planos
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(corners);
    icp.setInputTarget(accumulated_cloud);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    // Aplicar ICP para alinear la nueva nube con la acumulada
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(input_cloud);
    // icp.setInputTarget(accumulated_cloud);
    // icp.setMaximumIterations(50);
    // icp.setTransformationEpsilon(1e-8);
    // icp.setMaxCorrespondenceDistance(0.1);
    
    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    icp.align(*aligned_cloud);

    if (icp.hasConverged()) {
        ROS_INFO("ICP ha convergido con fitness score: %f", icp.getFitnessScore());

       // Obtener la transformación calculada por ICP
        Eigen::Matrix4f current_transformation = icp.getFinalTransformation();

        if (is_first_aligned_cloud) {
            // Almacenar la transformación de la primera nube alineada
            fixed_transformation = current_transformation;
            is_first_aligned_cloud = false;
        }

        // Corregir la nueva nube con la transformación fija
        pcl::transformPointCloud(*aligned_cloud, *aligned_cloud, fixed_transformation.inverse() * current_transformation);

        // Actualizar la nube acumulada y publicar
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

    // Suscriptor para la rosbag de LiDAR
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 100, pointCloudCallback);

    // Publicador para la nube acumulada alineada
    pub_aligned_cloud = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 100);

    ros::spin();
    return 0;
}

