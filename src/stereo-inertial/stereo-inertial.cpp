//Author: https://github.com/zang09/ORB_SLAM3_ROS2/tree/humble

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM("/home/rods/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin", "/home/rods/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo/RealSense_Booster.yaml", ORB_SLAM3::System::IMU_STEREO, visualization);

    auto node = std::make_shared<StereoInertialNode>(&pSLAM, "/home/rods/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Stereo/RealSense_Booster.yaml", "false", "false");
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
