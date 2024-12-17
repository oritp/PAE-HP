#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


std::vector<Eigen::Matrix4f> transformations;
ros::Publisher path_pub;

// Callback to store the transformation matrix
void readTransformCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    Eigen::Matrix4f matrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix(i, j) = msg->data[i * 4 + j];
        }
    }
    transformations.push_back(matrix);
}

// Service to estimate and publish the trajectory
bool computeTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (transformations.empty()) {
        res.success = false;
        res.message = "There is no transformations stored.";
        return true;
    }

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    for (const auto& matrix : transformations) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        pose.pose.position.x = matrix(0, 3);
        pose.pose.position.y = matrix(1, 3);
        pose.pose.position.z = matrix(2, 3);

        Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path.poses.push_back(pose);
    }

    path_pub.publish(path);

    res.success = true;
    res.message = "Trajectory calculated and published.";
    ROS_INFO("Trajectory calculated and published.");
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("transformation", 200, readTransformCallback);
    path_pub = nh.advertise<nav_msgs::Path>("trajectory_path", 200);
    ROS_INFO("Trajectory Estimator Node is ready.");

    // Advertise the service to compute the trajectory
    ros::ServiceServer service = nh.advertiseService("trajectory", computeTrajectory);

    ros::spin();

    return EXIT_SUCCESS;
}

