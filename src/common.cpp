//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp")
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
    this->declare_parameter("node_name_arg", "slam_cpp");                             // Name of this agent
    this->declare_parameter("voc_file_arg", "orb_slam3/Vocabulary/ORBvoc.txt.bin");   // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file", "orb_slam3/config/Monocular-Inertial/"); // path to settings file
    this->declare_parameter("camera", "RealSense_Booster");                           // name of the settings file

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = packagePath + param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file");
    settingsFilePath = packagePath + param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("camera");
    cameraFilePath = param4.as_string();

    timeStep = 0.0;
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    this->initializeVSLAM(cameraFilePath); // Initialize VSLAM with camera settings
    //* Image subscription
    const std::string imageTopic = "/camera/camera/color/image_raw"; // topic to receive RGB image messages
    imageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(imageTopic, 1, std::bind(&MonocularMode::imageCallback, this, _1));

    //* IMU subscriptions (GYRO + ACCELEROMETER) in a separate callback group
    auto imuCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imuSubOptions;
    imuSubOptions.callback_group = imuCallbackGroup;

    const std::string gyroTopic = "/camera/camera/gyro/sample";
    gyroSubscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        gyroTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularMode::gyroCallback, this, _1),
        imuSubOptions);

    const std::string accelTopic = "/camera/camera/accel/sample";
    accelSubscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        accelTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularMode::accelCallback, this, _1),
        imuSubOptions);

    RCLCPP_INFO(this->get_logger(), "MonocularMode node initialized successfully");
}
//* Destructor
MonocularMode::~MonocularMode()
{
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
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR; // IMU monocular
    enablePangolinWindow = true;                   // Shows Pangolin window output
    enableOpenCVWindow = true;                     // Shows OpenCV window output

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
}

//* Callback to process image message and run SLAM node
void MonocularMode::imageCallback(const sensor_msgs::msg::Image &msg)
{
    // Constructors of Point struct
    //  Point(const float &acc_x, const float &acc_y, const float &acc_z,
    //           const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
    //           const double &timestamp): a(acc_x,acc_y,acc_z), w(ang_vel_x,ang_vel_y,ang_vel_z), t(timestamp){}
    //  Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp):
    //      a(Acc.x,Acc.y,Acc.z), w(Gyro.x,Gyro.y,Gyro.z), t(timestamp){}
    cv_bridge::CvImagePtr cv_ptr;
    double prev_timeStep = timeStep;
    //* Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
        timeStep = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }
    // Interpolate accel to gyro timestamps
    std::vector<std::pair<cv::Point3f, double>> localGyroBuffer;
    std::vector<std::pair<cv::Point3f, double>> localAccelBuffer;

    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        // Partition gyroBuffer
        auto gyroSplit = std::find_if(
            gyroBuffer.begin(), gyroBuffer.end(),
            [this](const auto &item)
            {
                return item.second > timeStep;
            });
        localGyroBuffer.assign(gyroBuffer.begin(), gyroSplit);
        gyroBuffer.erase(gyroBuffer.begin(), gyroSplit);

        // Partition accelBuffer
        auto accelSplit = std::find_if(
            accelBuffer.begin(), accelBuffer.end(),
            [this](const auto &item)
            {
                return item.second > timeStep;
            });
        localAccelBuffer.assign(accelBuffer.begin(), accelSplit);
        accelBuffer.erase(accelBuffer.begin(), accelSplit);
    }

    if (!localGyroBuffer.empty() && !localAccelBuffer.empty())
    {
        size_t accelIdx = 0;
        for (const auto &gyro : localGyroBuffer)
        {
            double gyroTime = gyro.second;
            if (gyroTime < prev_timeStep)
                continue;
            // Find accel samples before and after gyroTime
            while (accelIdx + 1 < localAccelBuffer.size() && localAccelBuffer[accelIdx + 1].second < gyroTime)
                ++accelIdx;

            // If out of bounds, skip
            if (accelIdx + 1 >= localAccelBuffer.size())
                break;

            const auto &a0 = localAccelBuffer[accelIdx];
            const auto &a1 = localAccelBuffer[accelIdx + 1];
            // Linear interpolation
            double t0 = a0.second, t1 = a1.second;
            cv::Point3f accInterp;
            if (t1 != t0)
            {
                float alpha = (gyroTime - t0) / (t1 - t0);
                accInterp = a0.first * (1.0f - alpha) + a1.first * alpha;
            }
            else
                accInterp = a0.first;

            // Add to imuMeas
            imuMeas.emplace_back(
                accInterp.x, accInterp.y, accInterp.z,
                gyro.first.x, gyro.first.y, gyro.first.z,
                gyroTime);
        }
    }
    if (imuMeas.empty())
    {
        RCLCPP_WARN(this->get_logger(), "IMU measurements are empty, skipping frame.");
        return;
    }
    // std::cout << "Processing image at time: " << timeStep << " with " << imuMeas.size() << " IMU measurements." << std::endl;
    //* Perform all ORB-SLAM3 operations in Monocular mode
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep, imuMeas);
    // Clear buffers after processing
    imuMeas.clear();
}

void MonocularMode::gyroCallback(const sensor_msgs::msg::Imu &msg)
{
    double time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9; // Timestamp
    std::pair<cv::Point3f, double> gyroMeasurement;
    gyroMeasurement.first.x = msg.angular_velocity.x;
    gyroMeasurement.first.y = msg.angular_velocity.y;
    gyroMeasurement.first.z = msg.angular_velocity.z;
    gyroMeasurement.second = time;
    if (time <= timeStep)
        return;
    std::lock_guard<std::mutex> lock(bufferMutex); // Lock mutex for buffer access
    gyroBuffer.push_back(gyroMeasurement);
}

void MonocularMode::accelCallback(const sensor_msgs::msg::Imu &msg)
{
    double time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    std::pair<cv::Point3f, double> accelMeasurement;
    accelMeasurement.first.x = msg.linear_acceleration.x;
    accelMeasurement.first.y = msg.linear_acceleration.y;
    accelMeasurement.first.z = msg.linear_acceleration.z;
    accelMeasurement.second = time;
    if (time <= timeStep)
        return;
    std::lock_guard<std::mutex> lock(bufferMutex); // Lock mutex for buffer access
    accelBuffer.push_back(accelMeasurement);
}
