#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);
    ~StereoSlamNode();

private:
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncStereo();
    std::thread *syncThread_;

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    // Image    
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;
};

#endif
