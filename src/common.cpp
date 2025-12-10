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

    subImgMsgName = "/camera/camera/color/image_raw";                          // topic to receive RGB image messages

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* Initialize Booster SDK for IMU data
    ChannelFactory::Instance()->Init(0);
    imuThread = std::thread(&MonocularMode::runBoosterSubscriber, this);
    
    RCLCPP_INFO(this->get_logger(), "MonocularMode node initialized successfully");
}
//* Destructor
MonocularMode::~MonocularMode()
{
    // Signal Booster thread to stop
    shutdownMonocular = true;
    
    // Wait for Booster thread to finish
    if (imuThread.joinable()) {
        imuThread.join();
    }

    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}

void MonocularMode::runBoosterSubscriber()
{
    // Create subscriber with lambda that calls member function
    ChannelSubscriber<LowState> channel_subscriber(TOPIC, this->BoosterImuHandler);
    ImuSubscriber->InitChannel();
    
    // Keep thread alive until shutdown
    while (!shutdownBooster) {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    
    RCLCPP_INFO(this->get_logger(), "Booster IMU thread shutting down");
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
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR; // IMU monocular
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true;   // Shows OpenCV window output

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; 
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image &msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas; 
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
    std::cout << "Received image at time: " << timeStep << std::endl;
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    {
        std::lock_guard<std::mutex> lock(imuMutex);
        vImuMeas = imuBuffer;  // Copy the buffer
        imuBuffer.clear();     // Clear for next frame
    }
    
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep, vImuMeas);
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    // Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
}



//EXAMPLE
// IMU callback:
void MonocularMode::BoosterImuHandler(const sensor_msgs::msg::Imu &msg) {
    std::lock_guard<std::mutex> lock(imuMutex);
    auto time = rclcpp::Clock().now();
    double timestamp = time.seconds() + time.nanoseconds() * 1e-9;
    std::cout << "Received IMU message at time: " << timestamp << std::endl;
    ORB_SLAM3::IMU::Point imuPoint(
        low_state_msg->imu_state().acc()[0],      
        low_state_msg->imu_state().acc()[1],      
        low_state_msg->imu_state().acc()[2],      
        low_state_msg->imu_state().gyro()[0],     
        low_state_msg->imu_state().gyro()[1],     
        low_state_msg->imu_state().gyro()[2],     
        timestamp
    );
    
    imuBuffer.push_back(imuPoint);
}