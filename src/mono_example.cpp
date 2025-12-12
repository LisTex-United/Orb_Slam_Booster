/*
 * Adapted from ROS2-ORB-SLAM3: https://github.com/Mechazo11/ros2_orb_slam3/blob/main/src/common.cpp
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Always the first line, initialize this node

    //* Declare a node object
    auto node = std::make_shared<MonocularMode>();

    // Use MultiThreadedExecutor for concurrent callbacks
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------