
#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "communicator.hpp"
#include "ORBextractor.h"
#include <covins/covins_base/config_comm.hpp>
#include "msg_utils/uwb.h"
// ------------------

namespace covins {

class FrontendWrapper {

public:
  using TransformType = TypeDefs::TransformType;
  using PositionType = TypeDefs::PositionType;
  FrontendWrapper();
  ~FrontendWrapper(){};

  // Main Loop
  auto run() -> void;

  bool ParseCamParamFile(cv::FileStorage &fSettings);
  bool ParseORBParamFile(cv::FileStorage &fSettings);

  void convertToMsg(covins::MsgKeyframe &msg, cv::Mat &img, TransformType T_wc,
                    TransformType T_wc_prev, int client_id, int index,
                    double ts);
  
  
  protected:
    auto imageCallbackTF(const sensor_msgs::ImageConstPtr &msgImg,
                         const nav_msgs::OdometryConstPtr &msgOdom) -> void;

#ifdef UWB
    auto imageOdomUwbCallbackTF(const sensor_msgs::ImageConstPtr &msgImg,
                         const nav_msgs::OdometryConstPtr &msgOdom, const msg_utils::uwbConstPtr &msgUwb) -> void;
#endif
    ros::NodeHandle node_, nodeLocal_;
    cv::Mat intrinsic_;
    cv::Mat distCoeff_;

    message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image, nav_msgs::Odometry>> *sync_;

    message_filters::Subscriber<sensor_msgs::Image> *subscriberImg_;
    message_filters::Subscriber<nav_msgs::Odometry> *subscriberOdom_;
#ifdef UWB
    message_filters::Subscriber<msg_utils::uwb> *subscriberUwb_;
    message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image, nav_msgs::Odometry, msg_utils::uwb>> *sync_with_uwb_;
#endif
    

    Eigen::Vector3d prev_pos_;
    Eigen::Vector3d curr_pos_;
    Eigen::Quaterniond prev_quat_;
    Eigen::Quaterniond curr_quat_;

    size_t kf_count_;
    double prev_ts_;
    double curr_ts_;
    cv::Mat K_;
    cv::Mat DistCoef_;
    bool is_fisheye_ = false;

    int n_feat_pr_;
    int n_feat_;
    std::string fType_;

    bool is_odom_imu_frame_ = false;
    double t_min_;
    double r_min_;

    TransformType Tsc_;
    PositionType Psu_;
    TransformType Twc_;
    TransformType Twc_prev_;
    
    std::shared_ptr<covins::ORBextractor> orb_extractor_;
    std::shared_ptr<covins::ORBextractor> orb_extractor_PR_;
    cv::Ptr<cv::xfeatures2d::SIFT> sift_detector_;
    cv::Ptr<cv::ORB> orb_detector_;

    // COVINS integration
    covins::TypeDefs::ThreadPtr thread_comm_;
    std::shared_ptr<covins::Communicator> comm_;
    int client_id_;
     
    // ------------------
};

}