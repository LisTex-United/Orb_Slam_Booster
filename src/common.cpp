//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
StereoMode::StereoMode() : Node("mono_node_cpp")
{
    std::string packagePath;
    try
    {
        packagePath = ament_index_cpp::get_package_prefix("ros2_orb_slam3") + "/";
        RCLCPP_INFO(this->get_logger(), "Package path: %s", packagePath.c_str());
    }
    catch (const std::exception &e)
    {
        packagePath = "ros2_test/src/ros2_orb_slam3/";
        RCLCPP_WARN(this->get_logger(), "Could not find package index: %s. Using default path.", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");
    //* Declare parameters
    this->declare_parameter("node_name_arg", "slam_cpp");                           // Name of this agent
    this->declare_parameter("voc_file_arg", "orb_slam3/Vocabulary/ORBvoc.txt.bin"); // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file", "orb_slam3/config/Stereo/");           // path to settings file
    this->declare_parameter("camera", "RealSense_Booster");                         // name of the settings file

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = packagePath + param2.as_string();
    // vocFilePath = "/home/rods/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin";
    rclcpp::Parameter param3 = this->get_parameter("settings_file");
    settingsFilePath = packagePath + param3.as_string();
    // settingsFilePath = "/home/rods/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular-Inertial/";
    rclcpp::Parameter param4 = this->get_parameter("camera");
    cameraFilePath = param4.as_string();
    // cameraFilePath = "RealSense_Booster";
    timeStep = 0.0;
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    this->initializeVSLAM(cameraFilePath); // Initialize VSLAM with camera settings
    //* Image subscription
    const std::string infra1Topic = "/camera/camera/infra1/image_rect_raw"; // topic to receive infra1 image messages
    imageSubscriber1 = this->create_subscription<sensor_msgs::msg::Image>(infra1Topic, 1, std::bind(&StereoMode::imageCallback, this, _1));

    const std::string infra2Topic = "/camera/camera/infra2/image_rect_raw"; // topic to receive infra2 image messages
    imageSubscriber2 = this->create_subscription<sensor_msgs::msg::Image>(infra2Topic, 1, std::bind(&StereoMode::imageCallback, this, _1));

    //* IMU subscriptions (GYRO + ACCELEROMETER) in a separate callback group
    // auto imuCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // rclcpp::SubscriptionOptions imuSubOptions;
    // imuSubOptions.callback_group = imuCallbackGroup;

    RCLCPP_INFO(this->get_logger(), "StereoMode node initialized successfully");
}
//* Destructor
StereoMode::~StereoMode()
{
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}

//* Method to bind an initialized VSLAM framework to this node
void StereoMode::initializeVSLAM(std::string &configString)
{
    //* Build .yaml`s file path
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::STEREO;
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true;   // Shows OpenCV window output

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "StereoMode node initialized" << std::endl;
}

//* Callback to process image message and run SLAM node
void StereoMode::imageCallback(const sensor_msgs::msg::Image &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    const std::string frame_id = msg.header.frame_id;
    //* Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
        imageBuffer[frame_id] = cv_ptr;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }
    if (imageBuffer.size() < 2)
        return;
    //* Perform all ORB-SLAM3 operations in Monocular mode
    Sophus::SE3f Tcw = pAgent->TrackStereo(imageBuffer["camera_infra1_optical_frame"]->image, imageBuffer["camera_infra2_optical_frame"]->image, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
    imageBuffer.clear(); // Clear buffer after processing both images
}