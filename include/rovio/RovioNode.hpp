/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
//transform broadcaster in tf2
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2/transform_datatypes.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
//Figure out why there are no srv includes
#include "rovio_interfaces/srv/srv_reset_to_pose.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"
#include "rovio_interfaces/msg/health.hpp"
#include "rovio/HealthMonitor.hpp"
#include <ranges>

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode : public rclcpp::Node {
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;
  mtPoseUpdate* mpPoseUpdate_;
  typedef typename std::tuple_element<2,typename mtFilter::mtUpdates>::type mtVelocityUpdate;
  typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
  mtVelocityMeas velocityUpdateMeas_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;
  std::string imu_topic = "/imu0";
  std::string cam0_topic = "/cam0/image_raw";
  std::string cam1_topic = "/cam1/image_raw";
  bool resize_image = false;
  bool scale_camera_mat = true;
  int resize_image_width = 320;
  int resize_image_height = 240;

  rovio_interfaces::msg::Health healthMonitorMsg;
  HealthMonitor<mtState::nMax_, mtState::nLevels_, mtState::patchSize_, mtState::nCam_, mtState::nPose_> healthTracker;


  // Nodes, Subscriber, Publisherss
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg0_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subGroundtruth_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGroundtruthOdometry_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subVelocity_;
  //Services also need to be figured out
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvResetFilter_;
  rclcpp::Service<rovio_interfaces::srv::SrvResetToPose>::SharedPtr srvResetToPoseFilter_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry_;
  //This needs to be a transform, need to figure that out
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pubTransform_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubPoseWithCovStamped_;
  //Another transform that needs to be figured out
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_T_J_W_transform;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tb_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPcl_; /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPatch_;  /**<Publisher: Patch data. */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarkers_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImuBias_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubExtrinsics_[mtState::nCam_];
  rclcpp::Publisher<rovio_interfaces::msg::Health>::SharedPtr healthMonitorPublisher;
  std::vector<image_transport::Publisher> imgVisPublishers_;
  geometry_msgs::msg::TransformStamped transformMsg_;
  geometry_msgs::msg::TransformStamped T_J_W_Msg_;
  nav_msgs::msg::Odometry odometryMsg_;
  geometry_msgs::msg::PoseWithCovarianceStamped estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::msg::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
  sensor_msgs::msg::PointCloud2 pclMsg_;
  sensor_msgs::msg::PointCloud2 patchMsg_;
  visualization_msgs::msg::Marker markerMsg_;
  sensor_msgs::msg::Imu imuBiasMsg_;
  geometry_msgs::msg::TransformStamped rovioTFBroadcastMsg_;
  geometry_msgs::msg::TransformStamped rovioTJWBroadcastMsg_;
  int msgSeq_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  mtOutput imuOutput_;
  MXD imuOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;
  int visFps = 5;
  int visFrameCount = 0;
  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(std::shared_ptr<mtFilter> mpFilter)
      : rclcpp::Node("rovio"), mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_), landmarkOutputImuCT_(&mpFilter->multiCamera_),
        cameraOutputCov_((int)(mtOutput::D_),(int)(mtOutput::D_)), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)), landmarkOutputCov_(3,3),
        featureOutputReadableCov_((int)(FeatureOutputReadable::D_), (int)(FeatureOutputReadable::D_))
  {
#ifndef NDEBUG
    RCLCPP_WARN(this->get_logger(),"====================== Debug Mode ======================");
#endif
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    forceOdometryPublishing_ = false;
    forcePoseWithCovariancePublishing_ = false;
    forceTransformPublishing_ = false;
    forceExtrinsicsPublishing_ = false;
    forceImuBiasPublishing_ = false;
    forcePclPublishing_ = false;
    forceMarkersPublishing_ = false;
    forcePatchPublishing_ = false;
    gotFirstMessages_ = false;

    // Subscribe topics


    srvResetFilter_ = this->create_service<std_srvs::srv::Empty>(
        "rovio/reset",
        std::bind(&RovioNode::resetServiceCallback, this,
          std::placeholders::_1, std::placeholders::_2));
    srvResetToPoseFilter_ =
        this->create_service<rovio_interfaces::srv::SrvResetToPose>(
            "rovio/reset_to_pose",
            std::bind(&RovioNode::resetToPoseServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));
    pubTransform_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("rovio/transform", 1);
    tb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pubOdometry_ =
        this->create_publisher<nav_msgs::msg::Odometry>("rovio/odometry", 1);
    pubPoseWithCovStamped_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "rovio/pose_with_covariance_stamped", 1);
    pubPcl_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("rovio/pcl", 10);
    pubPatch_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("rovio/patch", 1);
    pubMarkers_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "rovio/markers", 10);
    pub_T_J_W_transform = this->create_publisher<geometry_msgs::msg::TransformStamped>("rovio/T_J_W",1);
    healthMonitorPublisher = this->create_publisher<rovio_interfaces::msg::Health>("rovio/health", 1);
    for (int camID = 0; camID < mtState::nCam_; camID++) {
      pubExtrinsics_[camID] =
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "rovio/extrinsics" + std::to_string(camID), 1);
       auto imgVisPublisher_ = image_transport::create_publisher(this,
              "rovio/imgVis" + std::to_string(camID));
        imgVisPublishers_.emplace_back(imgVisPublisher_);
    }



    pubImuBias_ =
        this->create_publisher<sensor_msgs::msg::Imu>("rovio/imu_biases", 1);

    // Handle coordinate frame naming
    map_frame_ = "/map";
    world_frame_ = "/world";
    camera_frame_ = "/camera";
    imu_frame_ = "/imu";
    map_frame_ = readAndDeclareParam<std::string>("map_frame", map_frame_);
    world_frame_ = readAndDeclareParam<std::string>("world_frame", world_frame_);
    camera_frame_ = readAndDeclareParam<std::string>("camera_frame", camera_frame_);
    imu_frame_ = readAndDeclareParam<std::string>("imu_frame", imu_frame_);
    imu_topic = readAndDeclareParam<std::string>("imu_topic", imu_topic);
    cam0_topic = readAndDeclareParam<std::string>("cam0_topic", cam0_topic);
    cam1_topic = readAndDeclareParam<std::string>("cam1_topic", cam1_topic);

    //rclcpp::QoS imuProfile = getQosProfile(imu_topic);
    rclcpp::QoS imuProfile = rclcpp::QoS(1000).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imuProfile,
     std::bind(&RovioNode::imuCallback, this, std::placeholders::_1));
    rclcpp::QoS cam0Profile = rclcpp::QoS(100).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    subImg0_ = this->create_subscription<sensor_msgs::msg::Image>(cam0_topic, cam0Profile,
              std::bind(&RovioNode::imgCallback0, this, std::placeholders::_1));
    rclcpp::QoS cam1Profile = rclcpp::QoS(100).reliability(rclcpp::ReliabilityPolicy::BestEffort);;
    subImg1_ = this->create_subscription<sensor_msgs::msg::Image>(cam1_topic,cam1Profile,
                std::bind(&RovioNode::imgCallback1, this, std::placeholders::_1));
    //subGroundtruth_ =
    // this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 1000,
    //              std::bind(&RovioNode::groundtruthCallback, this,
    //              std::placeholders::_1));
    subGroundtruthOdometry_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 1000,
            std::bind(&RovioNode::groundtruthOdometryCallback, this,
                      std::placeholders::_1));
    subVelocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "abss/twist", 1000,
        std::bind(&RovioNode::velocityCallback, this, std::placeholders::_1));

    resize_image = readAndDeclareParam<bool>("resize_image", resize_image);
    resize_image_width = readAndDeclareParam<int>("resize_image_width", resize_image_width);
    resize_image_height = readAndDeclareParam<int>("resize_image_height", resize_image_height);
    visFps = readAndDeclareParam<int>("vis_fps", visFps);
    healthTracker.pixelCovThreshold = readAndDeclareParam<float>("health_tracker_pix_cov_threshold", healthTracker.pixelCovThreshold);
    healthTracker.accelThreshold = readAndDeclareParam<float>("health_tracker_accel_threshold", healthTracker.accelThreshold);
    healthTracker.velocityThreshold = readAndDeclareParam<float>("health_tracker_velocity_threshold", healthTracker.velocityThreshold);
    odometryMsg_.header.frame_id = world_frame_;
    odometryMsg_.child_frame_id = imu_frame_;
    msgSeq_ = 1;
    for (int camID = 0; camID < mtState::nCam_; camID++) {
      extrinsicsMsg_[camID].header.frame_id = imu_frame_;
    }
    imuBiasMsg_.header.frame_id = world_frame_;
    imuBiasMsg_.orientation.x = 0;
    imuBiasMsg_.orientation.y = 0;
    imuBiasMsg_.orientation.z = 0;
    imuBiasMsg_.orientation.w = 1;
    for (int i = 0; i < 9; i++) {
      imuBiasMsg_.orientation_covariance[i] = 0.0;
    }

    // PointCloud message.
    pclMsg_.header.frame_id = imu_frame_;
    pclMsg_.height = 1;             // Unordered point cloud.
    pclMsg_.width = mtState::nMax_; // Number of features/points.
    const int nFieldsPcl = 18;
    std::string namePcl[nFieldsPcl] = {
        "id",   "camId", "rgb",  "status", "x",    "y",
        "z",    "b_x",   "b_y",  "b_z",    "d",    "c_00",
        "c_01", "c_02",  "c_11", "c_12",   "c_22", "c_d"};
    int sizePcl[nFieldsPcl] = {4, 4, 4, 4, 4, 4, 4, 4, 4,
                               4, 4, 4, 4, 4, 4, 4, 4, 4};
    int countPcl[nFieldsPcl] = {1, 1, 1, 1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1, 1, 1, 1};
    int datatypePcl[nFieldsPcl] = {sensor_msgs::msg::PointField::INT32,
                                   sensor_msgs::msg::PointField::INT32,
                                   sensor_msgs::msg::PointField::UINT32,
                                   sensor_msgs::msg::PointField::UINT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32,
                                   sensor_msgs::msg::PointField::FLOAT32};
    pclMsg_.fields.resize(nFieldsPcl);
    int byteCounter = 0;
    for (int i = 0; i < nFieldsPcl; i++) {
      pclMsg_.fields[i].name = namePcl[i];
      pclMsg_.fields[i].offset = byteCounter;
      pclMsg_.fields[i].count = countPcl[i];
      pclMsg_.fields[i].datatype = datatypePcl[i];
      byteCounter += sizePcl[i] * countPcl[i];
    }
    pclMsg_.point_step = byteCounter;
    pclMsg_.row_step = pclMsg_.point_step * pclMsg_.width;
    pclMsg_.data.resize(pclMsg_.row_step * pclMsg_.height);
    pclMsg_.is_dense = false;

    // PointCloud message.
    patchMsg_.header.frame_id = "";
    patchMsg_.height = 1;             // Unordered point cloud.
    patchMsg_.width = mtState::nMax_; // Number of features/points.
    const int nFieldsPatch = 5;
    std::string namePatch[nFieldsPatch] = {"id", "patch", "dx", "dy", "error"};
    int sizePatch[nFieldsPatch] = {4, 4, 4, 4, 4};
    int countPatch[nFieldsPatch] = {
        1, mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
        mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
        mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
        mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_};
    int datatypePatch[nFieldsPatch] = {sensor_msgs::msg::PointField::INT32,
                                       sensor_msgs::msg::PointField::FLOAT32,
                                       sensor_msgs::msg::PointField::FLOAT32,
                                       sensor_msgs::msg::PointField::FLOAT32,
                                       sensor_msgs::msg::PointField::FLOAT32};
    patchMsg_.fields.resize(nFieldsPatch);
    byteCounter = 0;
    for (int i = 0; i < nFieldsPatch; i++) {
      patchMsg_.fields[i].name = namePatch[i];
      patchMsg_.fields[i].offset = byteCounter;
      patchMsg_.fields[i].count = countPatch[i];
      patchMsg_.fields[i].datatype = datatypePatch[i];
      byteCounter += sizePatch[i] * countPatch[i];
    }
    patchMsg_.point_step = byteCounter;
    patchMsg_.row_step = patchMsg_.point_step * patchMsg_.width;
    patchMsg_.data.resize(patchMsg_.row_step * patchMsg_.height);
    patchMsg_.is_dense = false;

    // Marker message (vizualization of uncertainty)
    markerMsg_.header.frame_id = imu_frame_;
    markerMsg_.id = 0;
    markerMsg_.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerMsg_.action = visualization_msgs::msg::Marker::ADD;
    markerMsg_.pose.position.x = 0;
    markerMsg_.pose.position.y = 0;
    markerMsg_.pose.position.z = 0;
    markerMsg_.pose.orientation.x = 0.0;
    markerMsg_.pose.orientation.y = 0.0;
    markerMsg_.pose.orientation.z = 0.0;
    markerMsg_.pose.orientation.w = 1.0;
    markerMsg_.scale.x = 0.04; // Line width.
    markerMsg_.color.a = 1.0;
    markerMsg_.color.r = 0.0;
    markerMsg_.color.g = 1.0;
    markerMsg_.color.b = 0.0;
  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}

  /** \brief Function to declare and read a parameter.
   * @tparam template type of parameter
   * @param string param name
   * @return paramter value
   */
  template <typename paramType>
  paramType readAndDeclareParam(std::string paramName, paramType defaultValue) {
    paramType value = defaultValue;
    this->declare_parameter(paramName, value);
    this->get_parameter(paramName, value);
    std::stringstream ss;
    ss << value;
    RCLCPP_INFO(this->get_logger(), "Param name: %s has value: %s ", paramName.c_str(), ss.str().c_str());
    return value;
  }

  /**
   * \brief Function to get the qos profile for the specific topic
   * @param topic name
   * @return qos type profile.
   */
  rclcpp::QoS getQosProfile( std::string topicName ) {
    std::vector<rclcpp::TopicEndpointInfo> publishers = this->get_publishers_info_by_topic(topicName, false);
    rclcpp::QoS profile = rclcpp::QoS(rclcpp::KeepLast(10));
    if (!publishers.empty()) {
      profile = publishers[0].qos_profile();
    }
    return profile;
  }


  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest(){
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState = mpFilter_->init_;
    mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    LWF::NormalVectorElement tempNor;
    for(int i=0;i<mtState::nMax_;i++){
      testState.CfP(i).camID_ = 0;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.CfP(i).set_nor(tempNor);
      testState.CfP(i).trackWarping_ = false;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.aux().feaCoorMeas_[i].set_nor(tempNor,true);
      testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
      testState.aux().feaCoorMeas_[i].camID_ = 0;
    }
    testState.CfP(0).camID_ = mtState::nCam_-1;
    mpTestFilterState->fsm_.setAllCameraPointers();

    // Prediction
    std::cout << "Testing Prediction" << std::endl;
    mpFilter_->mPrediction_.testPredictionJacs(testState,predictionMeas_,1e-8,1e-6,0.1);

    // Update
    if(!mpImgUpdate_->useDirectMethod_){
      std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
      for(int i=0;i<(std::min((int)mtState::nMax_,2));i++){
        testState.aux().activeFeature_ = i;
        testState.aux().activeCameraCounter_ = 0;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
        testState.aux().activeCameraCounter_ = mtState::nCam_-1;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
      }
    }

    // Testing CameraOutputCF and CameraOutputCF
    std::cout << "Testing cameraOutputCF" << std::endl;
    cameraOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing imuOutputCF" << std::endl;
    imuOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing attitudeToYprCF" << std::endl;
    rovio::AttitudeToYprCT attitudeToYprCF;
    attitudeToYprCF.testTransformJac(1e-8,1e-6);

    // Testing TransformFeatureOutputCT
    std::cout << "Testing transformFeatureOutputCT" << std::endl;
    transformFeatureOutputCT_.setFeatureID(0);
    if(mtState::nCam_>1){
      transformFeatureOutputCT_.setOutputCameraID(1);
      transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);
    }
    transformFeatureOutputCT_.setOutputCameraID(0);
    transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);

    // Testing LandmarkOutputImuCT
    std::cout << "Testing LandmarkOutputImuCT" << std::endl;
    landmarkOutputImuCT_.setFeatureID(0);
    landmarkOutputImuCT_.testTransformJac(testState,1e-8,1e-5);

    // Getting featureOutput for next tests
    transformFeatureOutputCT_.transformState(testState,featureOutput_);
    if(!featureOutput_.c().isInFront()){
      featureOutput_.c().set_nor(featureOutput_.c().get_nor().rotated(QPD(0.0,1.0,0.0,0.0)),false);
    }

    // Testing FeatureOutputReadableCT
    std::cout << "Testing FeatureOutputReadableCT" << std::endl;
    featureOutputReadableCT_.testTransformJac(featureOutput_,1e-8,1e-5);

    // Testing pixelOutputCT
    rovio::PixelOutputCT pixelOutputCT;
    std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
    pixelOutputCT.testTransformJac(featureOutput_,1e-4,1.0); // Reduces accuracy due to float and strong camera distortion

    // Testing ZeroVelocityUpdate_
    std::cout << "Testing zero velocity update" << std::endl;
    mpImgUpdate_->zeroVelocityUpdate_.testJacs();

    // Testing PoseUpdate
    if(!mpPoseUpdate_->noFeedbackToRovio_){
      std::cout << "Testing pose update" << std::endl;
      mpPoseUpdate_->testUpdateJacs(1e-8,1e-5);
    }

    delete mpTestFilterState;
  }

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0(const sensor_msgs::msg::Image::ConstPtr & img){
    std::lock_guard<std::mutex> lock(m_filter_);
    imgCallback(img,0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback1(const sensor_msgs::msg::Image::ConstPtr & img) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(mtState::nCam_ > 1) imgCallback(img,1);
  }

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const sensor_msgs::msg::Image::ConstPtr & img, const int camID = 0){
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat cv_img;
    cv_ptr->image.copyTo(cv_img);
    if ( resize_image ) {
      cv::resize(cv_img, cv_img, cv::Size(resize_image_width, resize_image_height));
      if ( scale_camera_mat ) {
        double scale_x = static_cast<double>(cv_img.cols)/mpFilter_->multiCamera_.cameras_[camID].image_width_;
        double scale_y = static_cast<double>(cv_img.rows)/mpFilter_->multiCamera_.cameras_[camID].image_height_;
        mpFilter_->multiCamera_.cameras_[camID].scaleCameraMatrix(scale_x, scale_y);
        scale_camera_mat = false;
      }
    }
    if(init_state_.isInitialized() && !cv_img.empty()){
      double msgTime =  rclcpp::Time(img->header.stamp).nanoseconds() * 1e-9;
      msgTime += mpFilter_->camera_timeOffset_;
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        for(int i=0;i<mtState::nCam_;i++){
          if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]){
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

  /** \brief Callback for external groundtruth as TransformStamped
   *
   *  @param transform - Groundtruth message.
   */
  //Figure out transformStamped messages later
  // void groundtruthCallback(const geometry_msgs::msg::TransformStamped::ConstPtr& transform){
  //   std::lock_guard<std::mutex> lock(m_filter_);
  //   if(init_state_.isInitialized()){
  //     Eigen::Vector3d JrJV(transform->transform.translation.x,transform->transform.translation.y,transform->transform.translation.z);
  //     poseUpdateMeas_.pos() = JrJV;
  //     QPD qJV(transform->transform.rotation.w,transform->transform.rotation.x,transform->transform.rotation.y,transform->transform.rotation.z);
  //     poseUpdateMeas_.att() = qJV.inverted();
  //     mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,transform->header.stamp.toSec()+mpPoseUpdate_->timeOffset_);
  //     updateAndPublish();
  //   }
  // }

  /** \brief Callback for external groundtruth as Odometry
   *
   * @param odometry - Groundtruth message.
   */
  void groundtruthOdometryCallback(const nav_msgs::msg::Odometry::ConstPtr& odometry) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()) {
      Eigen::Vector3d JrJV(odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z);
      poseUpdateMeas_.pos() = JrJV;

      QPD qJV(odometry->pose.pose.orientation.w,odometry->pose.pose.orientation.x,odometry->pose.pose.orientation.y,odometry->pose.pose.orientation.z);
      poseUpdateMeas_.att() = qJV.inverted();

      const Eigen::Matrix<double,6,6> measuredCov = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(odometry->pose.covariance.data());
      poseUpdateMeas_.measuredCov() = measuredCov;
      double odomStamp = rclcpp::Time(odometry->header.stamp).nanoseconds() * 1e-9;
      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,odomStamp+mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg){
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    healthTracker.computeAccelDeviation( predictionMeas_. template get<mtPredictionMeas::_acc>());
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_, rclcpp::Time(imu_msg->header.stamp).nanoseconds() * 1e-9);
      updateAndPublish();
    } else {
      switch(init_state_.state_) {
      case FilterInitializationState::State::WaitForInitExternalPose: {
        std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
        mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, rclcpp::Time(imu_msg->header.stamp).nanoseconds() * 1e-9);
        break;
      }
      case FilterInitializationState::State::WaitForInitUsingAccel: {
        std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
        mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),
                                           rclcpp::Time(imu_msg->header.stamp).nanoseconds() * 1e-9);
        break;
      }
      default: {
        std::cout << "Unhandeld initialization type." << std::endl;
        abort();
        break;
      }
      }

      std::cout << std::setprecision(24);
      std::cout << "-- Filter: Initialized at t = " << rclcpp::Time(imu_msg->header.stamp).nanoseconds() * 1e-9 << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }

  /** \brief Callback for external velocity measurements
   *
   *  @param transform - Groundtruth message.
   */
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& velocity){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()){
      Eigen::Vector3d AvM(velocity->twist.linear.x,velocity->twist.linear.y,velocity->twist.linear.z);
      velocityUpdateMeas_.vel() = AvM;
      double velStamp = rclcpp::Time(velocity->header.stamp).nanoseconds() * 1e-9;
      mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_,velStamp);
      updateAndPublish();
    }
  }

  /** \brief ROS service handler for resetting the filter.
   */
  void resetServiceCallback(std_srvs::srv::Empty::Request::SharedPtr /*request*/,
                            std_srvs::srv::Empty::Response::SharedPtr /*response*/){
    requestReset();
    return;
  }

  /** \brief ROS service handler for resetting the filter to a given pose.
   */
   void resetToPoseServiceCallback(rovio_interfaces::srv::SrvResetToPose::Request::SharedPtr request,
                                   rovio_interfaces::srv::SrvResetToPose::Response::SharedPtr /*response*/){
     V3D WrWM(request->t_wm.position.x, request->t_wm.position.y,
              request->t_wm.position.z);
     QPD qWM(request->t_wm.orientation.w, request->t_wm.orientation.x,
             request->t_wm.orientation.y, request->t_wm.orientation.z);
     requestResetToPose(WrWM, qWM.inverted());
     return;
   }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset() {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D& WrWM, const QPD& qMW) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_ = WrWM;
    init_state_.qMW_ = qMW;
    init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
  }

  /**
   * @brief Helper function to convert the filter timestamp in double to rclcpp
   * Time object
   * @param stamp
   * @return rclcpp::Time object
   */
  rclcpp::Time doubleToStamp(double stamp) {
    return rclcpp::Time(static_cast<uint64_t>(stamp));
  }

  /**
   * @brif=ef Function to publish a transform.
   * @param translation Translation offset from frame_id to child_frame_id
   * @param quat rotation offdet from frame_id to child_frame_id
   * @param frame_id origin frame id
   * @param stamp destination frame_id
   * @return bool if transform publishing was sucessful
   */
  bool sendTransform(Eigen::Vector3d &translation, QPD & quat, std::string frame_id, std::string child_frame_id,
    double stamp) {
    geometry_msgs::msg::TransformStamped transformMsg;
    transformMsg.header.stamp = doubleToStamp(stamp);
    transformMsg.header.frame_id = frame_id;
    transformMsg.child_frame_id = child_frame_id;
    transformMsg.transform.translation.x = translation(0);
    transformMsg.transform.translation.y = translation(1);
    transformMsg.transform.translation.z = translation(2);
    transformMsg.transform.rotation.x = quat.x();
    transformMsg.transform.rotation.y = quat.y();
    transformMsg.transform.rotation.z = quat.z();
    transformMsg.transform.rotation.w = -quat.w();
    tb_->sendTransform(transformMsg);
    return true;
  }
  /**
   * @brief Function to compute and populate the healthTracker object fields and
   * then populate the healthMonitoring msg based on th fields.
   * @param mpFilter_ current filter state.
   * @return none
   */
  void computeHealthMessage( const std::shared_ptr<mtFilter> &mpFilter_) {
    healthTracker.computeFeatureDepthCovMedian(mpFilter_);
    healthTracker.computeNISZScoreRMSE(mpImgUpdate_->featureZScores_);
    healthTracker.computePixelCovRatio(mpFilter_);
    healthTracker.computeValidFeatureRatio(mpFilter_);
    healthTracker.computeTrackedFeatureRatio(mpFilter_);
    healthTracker.populateHealthMsg(mpFilter_, healthMonitorMsg, imu_frame_);
  }


  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        mpFilter_->updateSafe(&lastImageTime);
      }
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        RCLCPP_INFO_STREAM(this->get_logger(),
          " == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C);
      }
      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        for(int i=0;i<mtState::nCam_;i++){
          if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
            if ( visFrameCount >= visFps) {
              cv::Mat img = mpFilter_->safe_.img_[i];
              std_msgs::msg::Header header;
              header.frame_id = "cam" + std::to_string(i);
              header.stamp = doubleToStamp(mpFilter_->safe_.t_);
              auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
              imgVisPublishers_[i].publish(msg);
              visFrameCount = 0;
            }
            visFrameCount++;
          }
        }
        if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
	      mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state,imuOutput_);
        // Cout verbose for pose measurements
        if(mpImgUpdate_->verbose_){
          if(mpPoseUpdate_->inertialPoseIndex_  >=0){
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if(mpPoseUpdate_->bodyPoseIndex_ >=0){
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

        // Send Map (Pose Sensor, I) to World (rovio-intern, W) transformation
        if(mpPoseUpdate_->inertialPoseIndex_ >=0){
          Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
          QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
          this->sendTransform(IrIW,
            qWI,
            map_frame_,
            world_frame_,
            mpFilter_->safe_.t_);
        }

        this->sendTransform(imuOutput_.WrWB(),
          imuOutput_.qBW(),
          world_frame_, imu_frame_, mpFilter_->safe_.t_);
        // Send camera pose.
        for (int camID = 0; camID < mtState::nCam_; camID++) {
          this->sendTransform(state.MrMC(camID), state.qCM(camID), imu_frame_,
            camera_frame_ + std::to_string(camID), mpFilter_->safe_.t_);
        }

        // Publish Odometry
        if(pubOdometry_->get_subscription_count() > 0 || forceOdometryPublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);
          odometryMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          odometryMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          odometryMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          odometryMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          odometryMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          odometryMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          odometryMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          odometryMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              odometryMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          odometryMsg_.twist.twist.linear.x = imuOutput_.BvB()(0);
          odometryMsg_.twist.twist.linear.y = imuOutput_.BvB()(1);
          odometryMsg_.twist.twist.linear.z = imuOutput_.BvB()(2);
          odometryMsg_.twist.twist.angular.x = imuOutput_.BwWB()(0);
          odometryMsg_.twist.twist.angular.y = imuOutput_.BwWB()(1);
          odometryMsg_.twist.twist.angular.z = imuOutput_.BwWB()(2);
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
              odometryMsg_.twist.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          pubOdometry_->publish(odometryMsg_);
        }
        healthTracker.computeUnhealthyVelocityDeviation(imuOutput_.BvB());
        if(pubPoseWithCovStamped_->get_subscription_count() > 0 || forcePoseWithCovariancePublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);

          estimatedPoseWithCovarianceStampedMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();

          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              estimatedPoseWithCovarianceStampedMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }

          pubPoseWithCovStamped_->publish(estimatedPoseWithCovarianceStampedMsg_);

        }

        // Send IMU pose message.
         if(pubTransform_->get_subscription_count() > 0 || forceTransformPublishing_){
           transformMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
           transformMsg_.transform.translation.x = imuOutput_.WrWB()(0);
           transformMsg_.transform.translation.y = imuOutput_.WrWB()(1);
           transformMsg_.transform.translation.z = imuOutput_.WrWB()(2);
           transformMsg_.transform.rotation.x = imuOutput_.qBW().x();
           transformMsg_.transform.rotation.y = imuOutput_.qBW().y();
           transformMsg_.transform.rotation.z = imuOutput_.qBW().z();
           transformMsg_.transform.rotation.w = -imuOutput_.qBW().w();
           pubTransform_->publish(transformMsg_);
        }

        if(pub_T_J_W_transform->get_subscription_count() > 0 || forceTransformPublishing_){
           if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
             Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
             QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
             T_J_W_Msg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
             T_J_W_Msg_.transform.translation.x = IrIW(0);
             T_J_W_Msg_.transform.translation.y = IrIW(1);
             T_J_W_Msg_.transform.translation.z = IrIW(2);
             T_J_W_Msg_.transform.rotation.x = qWI.x();
             T_J_W_Msg_.transform.rotation.y = qWI.y();
             T_J_W_Msg_.transform.rotation.z = qWI.z();
             T_J_W_Msg_.transform.rotation.w = -qWI.w();
             pub_T_J_W_transform->publish(T_J_W_Msg_);
           }
         }

        // Publish Extrinsics
        for(int camID=0;camID<mtState::nCam_;camID++){
          if(pubExtrinsics_[camID]->get_subscription_count() > 0 || forceExtrinsicsPublishing_){
            extrinsicsMsg_[camID].header.stamp = doubleToStamp(mpFilter_->safe_.t_);
            extrinsicsMsg_[camID].pose.pose.position.x = state.MrMC(camID)(0);
            extrinsicsMsg_[camID].pose.pose.position.y = state.MrMC(camID)(1);
            extrinsicsMsg_[camID].pose.pose.position.z = state.MrMC(camID)(2);
            extrinsicsMsg_[camID].pose.pose.orientation.x = state.qCM(camID).x();
            extrinsicsMsg_[camID].pose.pose.orientation.y = state.qCM(camID).y();
            extrinsicsMsg_[camID].pose.pose.orientation.z = state.qCM(camID).z();
            extrinsicsMsg_[camID].pose.pose.orientation.w = -state.qCM(camID).w();
            for(unsigned int i=0;i<6;i++){
              unsigned int ind1 = mtState::template getId<mtState::_vep>(camID)+i;
              if(i>=3) ind1 = mtState::template getId<mtState::_vea>(camID)+i-3;
              for(unsigned int j=0;j<6;j++){
                unsigned int ind2 = mtState::template getId<mtState::_vep>(camID)+j;
                if(j>=3) ind2 = mtState::template getId<mtState::_vea>(camID)+j-3;
                extrinsicsMsg_[camID].pose.covariance[j+6*i] = cov(ind1,ind2);
              }
            }
            pubExtrinsics_[camID]->publish(extrinsicsMsg_[camID]);
          }
        }

        // Publish IMU biases
        if(pubImuBias_->get_subscription_count() > 0 || forceImuBiasPublishing_){
          imuBiasMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          imuBiasMsg_.angular_velocity.x = state.gyb()(0);
          imuBiasMsg_.angular_velocity.y = state.gyb()(1);
          imuBiasMsg_.angular_velocity.z = state.gyb()(2);
          imuBiasMsg_.linear_acceleration.x = state.acb()(0);
          imuBiasMsg_.linear_acceleration.y = state.acb()(1);
          imuBiasMsg_.linear_acceleration.z = state.acb()(2);
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.angular_velocity_covariance[3*i+j] = cov(mtState::template getId<mtState::_gyb>()+i,mtState::template getId<mtState::_gyb>()+j);
            }
          }
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.linear_acceleration_covariance[3*i+j] = cov(mtState::template getId<mtState::_acb>()+i,mtState::template getId<mtState::_acb>()+j);
            }
          }
          pubImuBias_->publish(imuBiasMsg_);
        }
        // PointCloud message.
        if(pubPcl_->get_subscription_count() > 0 || pubMarkers_->get_subscription_count() > 0 || forcePclPublishing_ || forceMarkersPublishing_){
          pclMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          markerMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          markerMsg_.points.clear();
          float badPoint = std::numeric_limits<float>::quiet_NaN();  // Invalid point.
          int offset = 0;

          FeatureDistance distance;
          double d,d_minus,d_plus;
          const double stretchFactor = 3;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += pclMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              // Get 3D feature coordinates.
              int camID = filterState.fsm_.features_[i].mpCoordinates_->camID_;
              distance = state.dep(i);
              d = distance.getDistance();
              const double sigma = sqrt(cov(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2));
              distance.p_ -= stretchFactor*sigma;
              d_minus = distance.getDistance();
              if(d_minus > 1000) d_minus = 1000;
              if(d_minus < 0) d_minus = 0;
              distance.p_ += 2*stretchFactor*sigma;
              d_plus = distance.getDistance();
              if(d_plus > 1000) d_plus = 1000;
              if(d_plus < 0) d_plus = 0;
              Eigen::Vector3d bearingVector = filterState.state_.CfP(i).get_nor().getVec();
              const Eigen::Vector3d CrCPm = bearingVector*d_minus;
              const Eigen::Vector3d CrCPp = bearingVector*d_plus;
              const Eigen::Vector3f MrMPm = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPm)).cast<float>();
              const Eigen::Vector3f MrMPp = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPp)).cast<float>();

              // Get human readable output
              transformFeatureOutputCT_.setFeatureID(i);
              transformFeatureOutputCT_.setOutputCameraID(filterState.fsm_.features_[i].mpCoordinates_->camID_);
              transformFeatureOutputCT_.transformState(state,featureOutput_);
              transformFeatureOutputCT_.transformCovMat(state,cov,featureOutputCov_);
              featureOutputReadableCT_.transformState(featureOutput_,featureOutputReadable_);
              featureOutputReadableCT_.transformCovMat(featureOutput_,featureOutputCov_,featureOutputReadableCov_);

              // Get landmark output
              landmarkOutputImuCT_.setFeatureID(i);
              landmarkOutputImuCT_.transformState(state,landmarkOutput_);
              landmarkOutputImuCT_.transformCovMat(state,cov,landmarkOutputCov_);
              const Eigen::Vector3f MrMP = landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();

              // Write feature id, camera id, and rgb
              uint8_t gray = 255;
              uint32_t rgb = (gray << 16) | (gray << 8) | gray;
              uint32_t status = filterState.fsm_.features_[i].mpStatistics_->status_[0];
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID, sizeof(int));  // cam id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb, sizeof(uint32_t));  // rgb
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status, sizeof(int));  // status

              // Write coordinates to pcl message.
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2], sizeof(float));  // z

              // Add feature bearing vector and distance
              const Eigen::Vector3f bearing = featureOutputReadable_.bea().template cast<float>();
              const float distance = static_cast<float>(featureOutputReadable_.dis());
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2], sizeof(float));  // z
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance, sizeof(float)); // d

              // Add the corresponding covariance (upper triangular)
              Eigen::Matrix3f cov_MrMP = landmarkOutputCov_.cast<float>();
              int mCounter = 11;
              for(int row=0;row<3;row++){
                for(int col=row;col<3;col++){
                  memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &cov_MrMP(row,col), sizeof(float));
                  mCounter++;
                }
              }

              // Add distance uncertainty
              const float distance_cov = static_cast<float>(featureOutputReadableCov_(3,3));
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &distance_cov, sizeof(float));

              // Line markers (Uncertainty rays).
              geometry_msgs::msg::Point point_near_msg;
              geometry_msgs::msg::Point point_far_msg;
              point_near_msg.x = float(CrCPp[0]);
              point_near_msg.y = float(CrCPp[1]);
              point_near_msg.z = float(CrCPp[2]);
              point_far_msg.x = float(CrCPm[0]);
              point_far_msg.y = float(CrCPm[1]);
              point_far_msg.z = float(CrCPm[2]);
              markerMsg_.points.push_back(point_near_msg);
              markerMsg_.points.push_back(point_far_msg);
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id, sizeof(int));  // id
              for(int j=1;j<pclMsg_.fields.size();j++){
                memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint, sizeof(float));
              }
            }
          }
          pubPcl_->publish(pclMsg_);
          pubMarkers_->publish(markerMsg_);
        }
        if(pubPatch_->get_subscription_count() > 0 || forcePatchPublishing_){
          patchMsg_.header.stamp = doubleToStamp(mpFilter_->safe_.t_);
          int offset = 0;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += patchMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              // Add patch data
              for(int l=0;l<mtState::nLevels_;l++){
                for(int y=0;y<mtState::patchSize_;y++){
                  for(int x=0;x<mtState::patchSize_;x++){
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[1].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // Patch
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[2].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dx_[y*mtState::patchSize_ + x], sizeof(float)); // dx
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[3].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dy_[y*mtState::patchSize_ + x], sizeof(float)); // dy
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[4].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.mlpErrorLog_[i].patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // error
                  }
                }
              }
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id, sizeof(int));  // id
            }
          }

          pubPatch_->publish(patchMsg_);
        }
        gotFirstMessages_ = true;
        computeHealthMessage( mpFilter_);
        healthMonitorPublisher->publish(healthMonitorMsg);
      }

    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
