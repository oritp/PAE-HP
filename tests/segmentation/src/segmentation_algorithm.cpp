#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;
ros::Publisher original_pub;

void generateAndSegmentCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Filtrado por rango (por si acaso hay puntos fuera de rango)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5.0); // Filtrar puntos entre 0 y 5 en z (suelo y techo)
    pass.filter(*cloud);

    // Filtro de voxel para reducir la densidad de puntos y acelerar el proceso
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Reducir resolución de los puntos
    voxel_filter.filter(*cloud_filtered);

    // Preparación para segmentación iterativa
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_filtered));
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05); // Umbral de distancia (5 cm)
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0; // Contador de iteraciones
    while (remaining_cloud->points.size() > 50) // Asegurarse de que quede suficiente nube de puntos
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Segmentación de un plano
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            std::cerr << "No se encontraron más planos en la nube." << std::endl;
            break;
        }

        std::cerr << "Plano " << i << " detectado con " << inliers->indices.size() << " puntos." << std::endl;

        // Extraer los puntos del plano detectado
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extrae solo los puntos del plano
        extract.filter(*plane_cloud);

        // Agregar a la nube segmentada
        *segmented_cloud += *plane_cloud;

        // Eliminar los puntos del plano detectado
        extract.setNegative(true); // Elimina los puntos del plano
        extract.filter(*remaining_cloud);

        i++; // Incrementar el contador de iteraciones
    }

    // Publicar la nube segmentada acumulada
    sensor_msgs::PointCloud2 segmented_output;
    pcl::toROSMsg(*segmented_cloud, segmented_output);
    segmented_output.header.frame_id = "map";
    pub.publish(segmented_output);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convertir la entrada de sensor_msgs::PointCloud2 a pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::fromROSMsg(*input, *cloud);

    // Llamar a la función de segmentación
    generateAndSegmentCloud(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planar_segmentation_node");
    ros::NodeHandle nh;

    // Configurar los publishers
    pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    original_pub = nh.advertise<sensor_msgs::PointCloud2>("original_cloud", 1);

    // Suscribirse al rosbag que contiene los datos de nube de puntos
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, cloudCallback);

    ros::spin();

    return 0;
}
