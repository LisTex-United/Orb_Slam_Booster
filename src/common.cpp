/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

*/
//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/

    try {
        // Get the install prefix for the package
        packagePath = ament_index_cpp::get_package_prefix("ros2_orb_slam3") + "/";
        RCLCPP_INFO(this->get_logger(), "Package path: %s", packagePath.c_str());
    } catch (const std::exception& e) {
        packagePath = "ros2_test/src/ros2_orb_slam3/";
        RCLCPP_WARN(this->get_logger(), "Could not find package index: %s. Using default path.", e.what());
    }

    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");


    this->declare_parameter("node_name_arg", "slam_cpp");                   // Name of this agent
    this->declare_parameter("voc_file_arg", "file_not_set");                // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file", "file_path_not_set"); // path to settings file
    this->declare_parameter("camera", "RealSense_Booster");                 // name of the settings file
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file");
    settingsFilePath = param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("camera");
    cameraFilePath = param4.as_string();

    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = packagePath + "orb_slam3/config/Monocular/";     // !Assumes monocular
    }

    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    this->initializeVSLAM(cameraFilePath);                      // Initialize VSLAM with camera settings

    subImgMsgName = "/camera/camera/color/image_rect";                          // topic to receive RGB image messages

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
}

//* Destructor
MonocularMode::~MonocularMode()
{

    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}


//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string &configString)
{

    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    //* Build .yaml`s file path
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true;   // Shows OpenCV window output

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image &msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    //* Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);                                 // Local scope
        timeStep = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9; // Get timestep from header
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }

    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    // Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
}
