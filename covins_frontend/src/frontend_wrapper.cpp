
#include "frontend_wrapper.hpp"
#include <eigen_conversions/eigen_msg.h>
#include "opencv2/calib3d/calib3d.hpp"

namespace covins {

FrontendWrapper::FrontendWrapper() {
  
  // constructor
}

auto FrontendWrapper::run()-> void {

  ROS_INFO("\nRun Wrapper .....");
  subscriberImg_ = new message_filters::Subscriber<sensor_msgs::Image>;
  subscriberOdom_ = new message_filters::Subscriber<nav_msgs::Odometry>;

  subscriberImg_->subscribe(node_, node_.resolveName("/camera/image_raw"), 5);
  subscriberOdom_->subscribe(node_, node_.resolveName("/cam_odom"), 5);
#ifdef DEBUG_OUTPUT
  std::cout << "----------------------1-----------------------------" << std::endl;
#endif
#ifdef UWB
#ifdef DEBUG_OUTPUT
  std::cout << "----------------------2-----------------------------" << std::endl;
#endif
  subscriberUwb_ = new message_filters::Subscriber<msg_utils::uwb>;
  subscriberUwb_->subscribe(node_, node_.resolveName("/uwb"), 5);
#ifdef DEBUG_OUTPUT
  std::cout << "----------------------3-----------------------------" << std::endl;
#endif
#endif
  std::string config_file;
  ros::param::get("~config_file", config_file);
#ifdef UWB
  sync_with_uwb_ = new message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                      nav_msgs::Odometry, msg_utils::uwb>>(
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                      nav_msgs::Odometry, msg_utils::uwb>(100),
      *subscriberImg_, *subscriberOdom_, *subscriberUwb_);
  sync_with_uwb_->registerCallback(
      boost::bind(&FrontendWrapper::imageOdomUwbCallbackTF, this, _1, _2, _3));
#else
  sync_ = new message_filters::Synchronizer<
          message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                  nav_msgs::Odometry>>(
          message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                  nav_msgs::Odometry>(100),
          *subscriberImg_, *subscriberOdom_);
  sync_->registerCallback(
          boost::bind(&FrontendWrapper::imageCallbackTF, this, _1, _2));
#endif
  prev_pos_ = Eigen::Vector3d::Zero();
  curr_pos_ = Eigen::Vector3d::Zero();
  prev_quat_ = Eigen::Quaterniond::Identity();
  curr_quat_ = Eigen::Quaterniond::Identity();
  Twc_prev_ = TransformType::Identity();
  prev_ts_ = 0;
  curr_ts_ = 0;
  kf_count_ = 0;

  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  

  bool b_parse_cam = ParseCamParamFile(fSettings);
  if(!b_parse_cam)
  {
      std::cout << "*Error with the camera parameters in the config file*" << std::endl;
  }

  // Load ORB parameters
  bool b_parse_orb = ParseORBParamFile(fSettings);
  if(!b_parse_orb)
  {
      std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
  }

  // COVINS Comm Integration
  covins_params::ShowParamsComm();
  comm_.reset(new covins::Communicator(covins_params::GetServerIP(),covins_params::GetPort()));
  thread_comm_.reset(new std::thread(&covins::Communicator::Run,comm_));
  // Get ID from back-end
  while(comm_->GetClientId() < 0){
      usleep(1000); //wait until ID is received from server
  }
  client_id_ = comm_->GetClientId();
  std::cout << "received client ID: " << client_id_ << std::endl;
  // ------------------

  ros::spin();

ros::shutdown();
}

/**
 * @brief Converts image, camera parameters and features to message format.
 * @param msg The message containing the converted data.
 * @param img The image to be converted.
 * @param T_wc The transform from world to camera coordinates.
 * @param T_wc_prev The transform from the previous world to camera coordinates.
 * @param client_id The ID of the client.
 * @param index The index of the image.
 * @param ts The timestamp of the image.
 */

void FrontendWrapper::convertToMsg(covins::MsgKeyframe &msg, cv::Mat &img,
                                   TransformType T_wc, TransformType T_wc_prev,int client_id,
                                   int index, double ts) {

  msg.is_update_msg = false;

  msg.id.first = index;
  msg.id.second = client_id;
  msg.timestamp = ts;

  msg.calibration.T_SC = Tsc_;
  msg.calibration.P_SU = Psu_;
  msg.calibration.cam_model = covins::eCamModel::PINHOLE;
  msg.calibration.dist_model = covins::eDistortionModel::RADTAN;

  covins::TypeDefs::precision_t fx = K_.at<float>(0,0);
  covins::TypeDefs::precision_t fy = K_.at<float>(1,1);
  covins::TypeDefs::precision_t cx = K_.at<float>(0,2);
  covins::TypeDefs::precision_t cy = K_.at<float>(1,2);
  covins::TypeDefs::precision_t k1 = DistCoef_.at<float>(0);
  covins::TypeDefs::precision_t k2 = DistCoef_.at<float>(1);
  covins::TypeDefs::precision_t p1 = DistCoef_.at<float>(2);
  covins::TypeDefs::precision_t p2 = DistCoef_.at<float>(3);
  covins::TypeDefs::DynamicVectorType dist_coeffs;

  dist_coeffs.resize(4);
  dist_coeffs << k1, k2, p1, p2;


  if (is_fisheye_) {
    // Undistorttion
    // Fisheye Undistortion
    dist_coeffs << 0.0,0.0,0.0,0.0;
    cv::Size size = {img.cols, img.rows};
    cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    cv::Mat map1;
    cv::Mat map2;
    cv::fisheye::initUndistortRectifyMap(K_, DistCoef_, E, K_, size, CV_16SC2, map1, map2);

    cv::Mat undistort;
    cv::remap(img, undistort, map1, map2, CV_INTER_LINEAR,
            CV_HAL_BORDER_CONSTANT);

    img = undistort;

    cv::imshow("undist", undistort);
    cv::waitKey(5);
 }


  
  covins::VICalibration calib(
      Tsc_, Psu_, msg.calibration.cam_model, msg.calibration.dist_model,
      dist_coeffs, img.cols, img.rows, fx, fy, cx,
      cy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81,
      Eigen::Vector3d::Zero(), 0, 0.0, 0.0);
  
  msg.calibration = calib;
  msg.img_dim_x_min = 0.0;
  msg.img_dim_y_min = 0.0;
  msg.img_dim_x_max = img.cols;
  msg.img_dim_y_max = img.rows;

  msg.keypoints_aors.reserve(n_feat_pr_);
  msg.keypoints_distorted.reserve(n_feat_pr_);
  msg.keypoints_undistorted.reserve(n_feat_pr_);
  msg.descriptors.reserve(n_feat_pr_);

  msg.keypoints_aors_add.reserve(n_feat_);
  msg.keypoints_distorted_add.reserve(n_feat_);
  msg.keypoints_undistorted_add.reserve(n_feat_);
  msg.descriptors_add.reserve(n_feat_);

  //Place Recognition Features
  std::vector<cv::KeyPoint> cv_keypoints;
  cv_keypoints.reserve(n_feat_pr_);
  
	cv::Mat new_descriptors;
  (*orb_extractor_PR_)(img, cv::Mat(), cv_keypoints, new_descriptors);

  for(size_t i=0;i<cv_keypoints.size();++i) {
      covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
      aors << cv_keypoints[i].angle, static_cast<float>(cv_keypoints[i].octave),
          cv_keypoints[i].response, cv_keypoints[i].size;

      covins::TypeDefs::KeypointType kp_eigen;
      kp_eigen[0] = static_cast<float>(cv_keypoints[i].pt.x);
      kp_eigen[1] = static_cast<float>(cv_keypoints[i].pt.y);
      msg.keypoints_aors.push_back(aors);
      msg.keypoints_distorted.push_back(kp_eigen);
      
  }
  msg.descriptors = new_descriptors.clone();


  // Features to be used for Pose Estimation
  // ORB or SIFT feature

  std::vector<cv::KeyPoint> cv_keypoints_add;
  cv_keypoints_add.reserve(n_feat_);
  cv::Mat new_descriptors_add;

  if (fType_ == "ORB") {
    (*orb_extractor_)(img, cv::Mat(), cv_keypoints_add, new_descriptors_add);
  } else if (fType_ == "SIFT") {
    sift_detector_->detectAndCompute(img, cv::Mat(), cv_keypoints_add,
                                     new_descriptors_add);
  } else {
    std::cout << COUTERROR << "Feature Type Not Supported: Select ORB or SIFT" << std::endl;
        exit(-1);
  }

  for(size_t i=0;i<cv_keypoints_add.size();++i) {
      covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
      aors << cv_keypoints_add[i].angle, static_cast<float>(cv_keypoints_add[i].octave), cv_keypoints_add[i].response, cv_keypoints_add[i].size;

      covins::TypeDefs::KeypointType kp_eigen;
      kp_eigen[0] = static_cast<float>(cv_keypoints_add[i].pt.x);
      kp_eigen[1] = static_cast<float>(cv_keypoints_add[i].pt.y);
      msg.keypoints_aors_add.push_back(aors);
      msg.keypoints_distorted_add.push_back(kp_eigen);
  }

	msg.descriptors_add = new_descriptors_add.clone();

  msg.T_s_c = Tsc_;
  msg.P_s_u = Psu_;
  msg.lin_acc = covins::TypeDefs::Vector3Type::Zero();
  msg.ang_vel = covins::TypeDefs::Vector3Type::Zero();

  TransformType T_w_sref = TransformType::Identity();
  TransformType T_w_s = TransformType::Identity();

  if (is_odom_imu_frame_) {
    T_w_sref = T_wc_prev;
    T_w_s = T_wc;
  } else {
    // If in Camera frame, convert it to IMU frame before sending to backend
    T_w_sref = T_wc_prev * Tsc_.inverse();
    T_w_s = T_wc * Tsc_.inverse();
  }
  
  msg.T_sref_s = T_w_sref.inverse() * T_w_s;

  if (index == 0)
    msg.id_predecessor = defpair;
  else {
    msg.id_predecessor.first = index-1;
    msg.id_predecessor.second = client_id;
    msg.id_reference.first = index-1;
    msg.id_reference.second = client_id;
  }
  msg.id_successor = defpair;
}


/**
 * @brief Callback function for the image and odometry messages received from
 * ROS topics.
 *
 * This function is called when an image and odometry message is received from
 * ROS topics. It converts the odometry message to Eigen format and computes the
 * transformation between the current and previous poses. If the difference
 * between the current and previous poses is above a certain threshold, a new
 * keyframe is created and sent to the server using the comm_ object.
 *
 * @param msgImg    A constant reference to a sensor_msgs::Image message
 * pointer.
 * @param msgOdom   A constant reference to a nav_msgs::Odometry message
 * pointer.
 *
 * @return void
 */

auto FrontendWrapper::imageCallbackTF(const sensor_msgs::ImageConstPtr &msgImg,
                     const nav_msgs::OdometryConstPtr &msgOdom) -> void {

  tf::pointMsgToEigen(msgOdom->pose.pose.position, curr_pos_);
  tf::quaternionMsgToEigen(msgOdom->pose.pose.orientation, curr_quat_);

  auto quat_ang = curr_quat_.angularDistance(prev_quat_);
  auto trans_diff = (curr_pos_ - prev_pos_).norm();
  double curr_ts = double(msgImg->header.stamp.toSec());

  TransformType T_wc = TransformType::Identity();
  TransformType T_wc_prev = TransformType::Identity();

  T_wc.block<3, 1>(0, 3) = curr_pos_;
  T_wc.block<3, 3>(0, 0) = curr_quat_.toRotationMatrix();

  T_wc_prev.block<3, 1>(0, 3) = prev_pos_;
  T_wc_prev.block<3, 3>(0, 0) = prev_quat_.toRotationMatrix();

  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvShare(msgImg);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }


  if (trans_diff > t_min_ || quat_ang > r_min_) {
    std::cout << "Generated New KF with id: " << kf_count_<< std::endl;
    cv::Mat img = cv_ptr->image.clone();

    // Convert KF to Msg
    data_bundle map_chunk;
    MsgKeyframe msg_kf;
    this->convertToMsg(msg_kf, img, T_wc, T_wc_prev, client_id_, kf_count_, curr_ts);
    map_chunk.keyframes.push_back(msg_kf);
    comm_->PassDataBundle(map_chunk);
    // ------------------

    prev_quat_ = curr_quat_;
    prev_pos_ = curr_pos_;
    Twc_prev_ = Twc_;
    prev_ts_ = curr_ts_;
    kf_count_++;
  }
}

#ifdef UWB
auto FrontendWrapper::imageOdomUwbCallbackTF(const sensor_msgs::ImageConstPtr &msgImg,
                                      const nav_msgs::OdometryConstPtr &msgOdom, const msg_utils::uwbConstPtr &msgUwb) -> void {
#ifdef DEBUG_OUTPUT
  std::cout << "--------------------receive image odom uwb message!!!------------------------" << std::endl;
#endif
  tf::pointMsgToEigen(msgOdom->pose.pose.position, curr_pos_);
  tf::quaternionMsgToEigen(msgOdom->pose.pose.orientation, curr_quat_);

  auto quat_ang = curr_quat_.angularDistance(prev_quat_);
  auto trans_diff = (curr_pos_ - prev_pos_).norm();
  double curr_ts = double(msgImg->header.stamp.toSec());

  TransformType T_wc = TransformType::Identity();
  TransformType T_wc_prev = TransformType::Identity();

  T_wc.block<3, 1>(0, 3) = curr_pos_;
  T_wc.block<3, 3>(0, 0) = curr_quat_.toRotationMatrix();

  T_wc_prev.block<3, 1>(0, 3) = prev_pos_;
  T_wc_prev.block<3, 3>(0, 0) = prev_quat_.toRotationMatrix();

  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msgImg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  if (trans_diff > t_min_ || quat_ang > r_min_) {
    std::cout << "Generated New KF with id: " << kf_count_<< std::endl;
    cv::Mat img = cv_ptr->image.clone();

    // Convert KF to Msg
    data_bundle map_chunk;
    MsgKeyframe msg_kf;
    this->convertToMsg(msg_kf, img, T_wc, T_wc_prev, client_id_, kf_count_, curr_ts);
    msg_kf.dist_list = msgUwb->dist;
    msg_kf.id_list = msgUwb->dest_id;
    msg_kf.dist_timestamp = msgUwb->header.stamp.toNSec();
#ifdef DEBUG_OUTPUT
    std::cout << "--------------------------------------------------------" << std::endl;
    std::vector<double> dist_vector =  msgUwb->dist;
    std::vector<signed char> id_vector = msgUwb->dest_id;
    for( int i = 0; i < dist_vector.size(); ++i ) {
      std::cout << "距离第 " << int(id_vector[i]) << " 个节点距离：" << dist_vector[i] << std::endl;
    }
    std::cout << "--------------------------------------------------------" << std::endl;
#endif
    map_chunk.keyframes.push_back(msg_kf);
    comm_->PassDataBundle(map_chunk);
    // ------------------

    prev_quat_ = curr_quat_;
    prev_pos_ = curr_pos_;
    Twc_prev_ = Twc_;
    prev_ts_ = curr_ts_;
    kf_count_++;
  }
}
#endif


bool FrontendWrapper::ParseCamParamFile(cv::FileStorage &fSettings)
{
    DistCoef_ = cv::Mat::zeros(4,1,CV_32F);
    std::cout << std::endl << "Camera Parameters: " << std::endl;
    bool b_miss_params = false;

    is_odom_imu_frame_ = int(fSettings["odom_in_imu_frame"]);

    is_fisheye_ = int(fSettings["is_fisheye"]);

    cv::FileNode node = fSettings["t_min"];
        
    if(!node.empty())
    {
        t_min_ = node.real();
    }
    
    node = fSettings["r_min"];
    if(!node.empty())
    {
        r_min_ = node.real();
    }


    std::string sCameraName = fSettings["Camera.type"];
    
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        // Camera calibration parameters

        cv::FileNode node = fSettings["Camera.fx"];

        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.resize(5);
            DistCoef_.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << DistCoef_.at<float>(0) << std::endl;
        std::cout << "- k2: " << DistCoef_.at<float>(1) << std::endl;

        std::cout << "- p1: " << DistCoef_.at<float>(2) << std::endl;
        std::cout << "- p2: " << DistCoef_.at<float>(3) << std::endl;

        if(DistCoef_.rows==5)
            std::cout << "- k3: " << DistCoef_.at<float>(4) << std::endl;

        K_ = cv::Mat::eye(3,3,CV_32F);
        K_.at<float>(0,0) = fx;
        K_.at<float>(1,1) = fy;
        K_.at<float>(0,2) = cx;
        K_.at<float>(1,2) = cy;
    }
    
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(b_miss_params)
    {
        return false;
    }

    // Params for Imu_Cam Transform

    cv::Mat Tbc;
    node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    std::cout << std::endl;
    std::cout << "Left camera to Imu Transform (Tbc): " << std::endl << Tbc << std::endl;

    cv::cv2eigen(Tbc, Tsc_);
    prev_pos_ = Tsc_.block<3, 1>(3, 0);
    Eigen::Quaterniond temp_quat(Tsc_.block<3,3>(0,0));
    prev_quat_ = temp_quat;


    // Params for Imu_Cam Transform

    cv::Mat Pbu;
    node = fSettings["Pbu"];
    if(!node.empty())
    {
      Pbu = node.mat();
      if(Pbu.rows != 3 || Pbu.cols != 1)
      {
        std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
        b_miss_params = true;
      }
    }
    else
    {
      std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
      b_miss_params = true;
    }

    std::cout << std::endl;
    std::cout << "UWB to Imu Transform (Tbu): " << std::endl << Pbu << std::endl;

    cv::cv2eigen(Pbu, Psu_);
    return true;
}

bool FrontendWrapper::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nFeaturesPR, nLevels, fIniThFAST, fMinThFAST;
    std::string fType;
    float fScaleFactor;

    cv::FileNode node = fSettings["extractor.type"];
    if(!node.empty() && node.isString())
    {
        fType = node.operator std::string();
    }
    else
    {
        std::cerr << "*extractor.type parameter doesn't exist or is not a string*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["extractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*extractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nFeaturesPR"];
    if(!node.empty() && node.isInt())
    {
        nFeaturesPR = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeaturesPR parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    orb_extractor_.reset(new covins::ORBextractor(
        nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
    orb_extractor_PR_.reset(new covins::ORBextractor(
        nFeaturesPR, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));

    orb_detector_ = cv::ORB::create(nFeatures);
    sift_detector_ = cv::xfeatures2d::SIFT::create(nFeatures);

    std::cout << std::endl << "Extractor Parameters: " << std::endl;
    std::cout << "Feature Type: " << fType << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Number of Features Place Rec: " << nFeaturesPR << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    n_feat_ = nFeatures;
    n_feat_pr_ = nFeaturesPR;
    fType_ = fType;

    return true;
}


}
