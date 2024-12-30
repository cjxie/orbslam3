#include "stereo_inertial_node.hpp"

#include<rclcpp/rclcpp.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System* pSLAM, const bool bRect, const bool bClahe) : 
    mpSLAM(pSLAM),
    mbRect(bRect),
    mbClahe(bClahe), 
    Node("Stereo_inertial_node")
{
    if (mbRect)
    {      
        cv::Mat M1l, M2l, M1r, M2r;
        mpSLAM->GetRectMap(M1l, M2l, M1r, M2r);
    }
    declare_parameter("imu", "/zed2/imu");
    declare_parameter("image_left", "/zed2/left/compressed");
    declare_parameter("image_right", "/zed2/right/compressed");
    

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(get_parameter("imu").as_string(), 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    sub_img0_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(get_parameter("image_left").as_string(), 1000, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    sub_img1_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(get_parameter("image_right").as_string(), 1000, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu,this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;


    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Stop all threads
    mpSLAM->Shutdown();

}

void StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  mBufMutexImu.lock();
  imuBuf.push(imu_msg);
  mBufMutexImu.unlock();
  return;
}

void StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img_msg)
{
  mBufMutex0.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex0.unlock();
}

void StereoInertialNode::GrabImageRight(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img_msg)
{
  mBufMutex1.lock();
  if (!img1Buf.empty())
    img1Buf.pop();
  img1Buf.push(img_msg);
  mBufMutex1.unlock();
}

cv::Mat StereoInertialNode::GetImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {    
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

double StereoInertialNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

void StereoInertialNode::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imLeft, imRight;
    double tImg0 = 0, tImg1 = 0;
    double tImu = 0;
    // RCLCPP_INFO(get_logger(), "[DataLoader]: Start processing");
    if (!img0Buf.empty()&& !img1Buf.empty() && !imuBuf.empty())
    {
      tImg0 = this->GetSeconds(img0Buf.front()->header.stamp);
      tImg1 = this->GetSeconds(img1Buf.front()->header.stamp);
      tImu = this->GetSeconds(imuBuf.back()->header.stamp);
      // RCLCPP_INFO(get_logger(), "[DataLoader]: load imu at time %.4f, queue size: %ld", tImu, imuBuf.size());

      this->mBufMutex1.lock();
      // RCLCPP_INFO(get_logger(), "[DataLoader]: right image queue size: %ld", img1Buf.size());
      while ((tImg0 - tImg1) > maxTimeDiff && img1Buf.size() > 1) 
      {
        img1Buf.pop();
        tImg1 = this->GetSeconds(img1Buf.front()->header.stamp);
      }  
      this->mBufMutex1.unlock();
      // RCLCPP_INFO(get_logger(), "[DataLoader]: load right image at time %.4f, queue size: %ld", tImg1, img1Buf.size());

      this->mBufMutex0.lock();
      // RCLCPP_INFO(get_logger(), "[DataLoader]: left image queue size: %ld", img0Buf.size());
      while ((tImg1 - tImg0) > maxTimeDiff && img0Buf.size() > 1) 
      {
        img0Buf.pop();
        tImg0 = this->GetSeconds(img0Buf.front()->header.stamp);
      }  
      this->mBufMutex0.unlock();
      // RCLCPP_INFO(get_logger(), "[DataLoader]: load left image at time %.4f, queue size: %ld", tImg0, img0Buf.size());

      if ((tImg0 - tImg1) > maxTimeDiff || (tImg0 - tImg1) > maxTimeDiff)
      {
        continue;
      }
      if (tImg0 > tImu)
      {
        continue;
      }

      // RCLCPP_INFO(get_logger(), "[SLAM]: start tracking");
      this->mBufMutex0.lock();
      imLeft = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex0.unlock();

      this->mBufMutex1.lock();
      imRight = GetImage(img1Buf.front());
      img1Buf.pop();
      this->mBufMutex1.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutexImu.lock();
      if(!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!imuBuf.empty() && (imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec / 1e9) <=tImg0)
        {
          double t = this->GetSeconds(imuBuf.front()->header.stamp);
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          
          imuBuf.pop();
        }
      }
      mBufMutexImu.unlock();
      
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      if(mbRect)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }
      
      mpSLAM->TrackStereo(imLeft, imRight, tImg0, vImuMeas);

      // int state = mpSLAM->GetTrackingState();
      // vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
      // vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
      // RCLCPP_INFO(get_logger(), "TrackingState: %ld, Tracked MPs: %ld, Tracked KPs: %ld", state, vMPs.size(), vKeys.size());

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}