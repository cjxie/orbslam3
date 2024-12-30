#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/msg/compressed_image.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const bool bRect, const bool bClahe);
    ~StereoInertialNode();

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void GrabImageRight(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
    void GrabImageLeft(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
    void SyncWithImu();
    cv::Mat GetImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg);
    double GetSeconds(builtin_interfaces::msg::Time stamp);

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
    queue<sensor_msgs::msg::CompressedImage::ConstSharedPtr> img0Buf, img1Buf;

    std::mutex mBufMutexImu;
    std::mutex mBufMutex0;
    std::mutex mBufMutex1;
    ORB_SLAM3::System* mpSLAM;

    const bool mbRect;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    std::thread *syncThread_;
    

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr sub_img0_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr sub_img1_;
};

#endif