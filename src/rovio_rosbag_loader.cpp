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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "rovio/RovioNode.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#define foreach BOOST_FOREACH

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

/**
 * @brief function to deserialize a template sample message, to read from a rosbag.
 * @tparam T
 * @param msg
 * @return T type. Msg type/
 */
template <typename T>
T deserializeMessage(auto msg) {
  rclcpp::SerializedMessage extractedMsg(*msg->serialized_data);
  rclcpp::Serialization<T> deserializer;
  T returnMsg;
  deserializer.deserialize_message(&extractedMsg, &returnMsg);
  return returnMsg;
}


/**
 *
 * @brief function to serialize the message, to store in the bag file.
 * @tparam Message type
 * @param msg to serialize
 * @return generalized serialized msg to store in bag file
 */
template <typename T>
rclcpp::SerializedMessage serializeMessage(T msgToSerialize ) {
  rclcpp::SerializedMessage serializedMsg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msgToSerialize, &serializedMsg);
  return serializedMsg;
}


/**
 * @brief Function to convert the Point stamped message to the pose stamped message
 * @param PointStamped message reference
 * @param PoseStamped message reference
 * @return None
 */
void pointToPose(const geometry_msgs::msg::PointStamped pointMsg,
                geometry_msgs::msg::PoseStamped::SharedPtr poseMsg) {
  poseMsg->header.stamp = pointMsg.header.stamp;
  poseMsg->header.frame_id = pointMsg.header.frame_id;
  poseMsg->pose.position.x = pointMsg.point.x;
  poseMsg->pose.position.y = pointMsg.point.y;
  poseMsg->pose.position.z = pointMsg.point.z;
  poseMsg->pose.orientation.x = 0;
  poseMsg->pose.orientation.y = 0;
  poseMsg->pose.orientation.z = 0;
  poseMsg->pose.orientation.w = 1;
}


/**
 * @brief Function to declare the camera config parameters
 * @param ROVIO node share ptr
 * @return none
 */
void declareParameters(std::shared_ptr<rovio::RovioNode<mtFilter>> node) {
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    node->declare_parameter("camera" + std::to_string(camID)
                            + "_config","");
  }
  node->declare_parameter("filter_config", "");
  node->declare_parameter("record_odometry", true);
  node->declare_parameter("record_pose_with_covariance_stamped", true);
  node->declare_parameter("record_transform", true);
  node->declare_parameter("record_extrinsics", true);
  node->declare_parameter("record_imu_bias", true);
  node->declare_parameter("record_pcl", true);
  node->declare_parameter("record_markers", true);
  node->declare_parameter("record_patch", false);
  node->declare_parameter("reset_trigger", 0.0);
}


/**
 * @brief Function to read the camera calibration parameters. File path are ros params of node.
 * @param mpFilter
 * @param node
 * @return None
 */
void readCameraConfig(std::shared_ptr<mtFilter> mpFilter,
                      std::shared_ptr<rovio::RovioNode<mtFilter>> node) {
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (node->get_parameter("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
                            }
  }
}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::string filter_config;

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  auto rovioNode = std::make_shared<rovio::RovioNode<mtFilter>>(mpFilter);
  declareParameters(rovioNode);
  rovioNode->get_parameter("filter_config", filter_config);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  readCameraConfig(mpFilter, rovioNode);
  mpFilter->refreshProperties();
  
  rovioNode->makeTest();
  double resetTrigger = 0.0;
  rovioNode->get_parameter("record_odometry", rovioNode->forceOdometryPublishing_);
  rovioNode->get_parameter("record_pose_with_covariance_stamped", rovioNode->forcePoseWithCovariancePublishing_);
  rovioNode->get_parameter("record_transform", rovioNode->forceTransformPublishing_);
  rovioNode->get_parameter("record_extrinsics", rovioNode->forceExtrinsicsPublishing_);
  rovioNode->get_parameter("record_imu_bias", rovioNode->forceImuBiasPublishing_);
  rovioNode->get_parameter("record_pcl", rovioNode->forcePclPublishing_);
  rovioNode->get_parameter("record_markers", rovioNode->forceMarkersPublishing_);
  rovioNode->get_parameter("record_patch", rovioNode->forcePatchPublishing_);
  rovioNode->get_parameter("reset_trigger", resetTrigger);

  std::cout << "Recording";
  if(rovioNode->forceOdometryPublishing_) std::cout << ", odometry";
  if(rovioNode->forceTransformPublishing_) std::cout << ", transform";
  if(rovioNode->forceExtrinsicsPublishing_) std::cout << ", extrinsics";
  if(rovioNode->forceImuBiasPublishing_) std::cout << ", imu biases";
  if(rovioNode->forcePclPublishing_) std::cout << ", point cloud";
  if(rovioNode->forceMarkersPublishing_) std::cout << ", markers";
  if(rovioNode->forcePatchPublishing_) std::cout << ", patch data";
  std::cout << std::endl;

  rosbag2_cpp::Reader bagIn;
  std::string rosbag_filename = "dataset.bag";
  rovioNode->declare_parameter("rosbag_filename", rosbag_filename);
  rovioNode->get_parameter("rosbag_filename", rosbag_filename);
  bagIn.open(rosbag_filename);

  rosbag2_cpp::Writer bagOut;
  std::size_t found = rosbag_filename.find_last_of("/");
  std::string file_path = rosbag_filename.substr(0,found);
  std::string file_name = rosbag_filename.substr(found+1);
  if(file_path==rosbag_filename){
    file_path = ".";
    file_name = rosbag_filename;
  }

  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format("%Y-%m-%d-%H-%M-%S");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << rovioNode->get_clock()->now().seconds() << "_" << nMax_ << "_" << nLevels_ << "_" << patchSize_ << "_" << nCam_  << "_" << nPose_;
  std::string filename_out = file_path + "/rovio/" + stream.str();
  rovioNode->declare_parameter("filename_out", filename_out);
  rovioNode->get_parameter("filename_out", filename_out);
  std::string rosbag_filename_out = filename_out + ".bag";
  std::string info_filename_out = filename_out + ".info";
  std::string gt_topic_name = "/gt";
  rovioNode->declare_parameter("gt_topic_name", gt_topic_name);
  rovioNode->get_parameter("gt_topic_name",gt_topic_name);
  std::cout << "GT topic: " << gt_topic_name << std::endl;
  bagOut.open(rosbag_filename_out);

  // Copy info
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();
  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  rovioNode->declare_parameter("imu_topic_name", imu_topic_name);
  rovioNode->get_parameter("imu_topic_name", imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  rovioNode->declare_parameter("cam0_topic_name", cam0_topic_name);
  rovioNode->get_parameter("cam0_topic_name", cam0_topic_name);
  std::string cam1_topic_name = "/cam1/image_raw";
  rovioNode->declare_parameter("cam1_topic_name", cam1_topic_name);
  rovioNode->get_parameter("cam1_topic_name", cam1_topic_name);
  std::string odometry_topic_name = rovioNode->pubOdometry_->get_topic_name();
  //std::string transform_topic_name = rovioNode->pubTransform_->get_topic_name();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
    extrinsics_topic_name[camID] = rovioNode->pubExtrinsics_[camID]->get_topic_name();
  }
  std::string imu_bias_topic_name = rovioNode->pubImuBias_->get_topic_name();
  std::string pcl_topic_name = rovioNode->pubPcl_->get_topic_name();
  std::string u_rays_topic_name = rovioNode->pubMarkers_->get_topic_name();
  std::string patch_topic_name = rovioNode->pubPatch_->get_topic_name();
  std::string health_topic_name = rovioNode->healthMonitorPublisher->get_topic_name();

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  topics.push_back(std::string(cam1_topic_name));


  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  while (bagIn.has_next()) {
    auto serializedMsg = bagIn.read_next();
    if(serializedMsg->topic_name == imu_topic_name){
      sensor_msgs::msg::Imu imuMsg = deserializeMessage<sensor_msgs::msg::Imu>(serializedMsg);
      sensor_msgs::msg::Imu::ConstPtr imuMsgPtr = std::make_shared<sensor_msgs::msg::Imu>(imuMsg);
      if (imuMsgPtr != NULL) rovioNode->imuCallback(imuMsgPtr);
    }
    if(serializedMsg->topic_name == cam0_topic_name){
      sensor_msgs::msg::Image imgMsg =  deserializeMessage<sensor_msgs::msg::Image>(serializedMsg);
      sensor_msgs::msg::Image::ConstPtr imgMsgPtr = std::make_shared<sensor_msgs::msg::Image>(imgMsg);
      if (imgMsgPtr != NULL) rovioNode->imgCallback0(imgMsgPtr);
    }
    if(serializedMsg->topic_name == cam1_topic_name){
      sensor_msgs::msg::Image imgMsg2 = deserializeMessage<sensor_msgs::msg::Image>(serializedMsg);
      sensor_msgs::msg::Image::ConstPtr imgMsg2Ptr = std::make_shared<sensor_msgs::msg::Image>(imgMsg2);
      if (imgMsg2Ptr != NULL) rovioNode->imgCallback1(imgMsg2Ptr);
    }
	if(serializedMsg->topic_name == gt_topic_name) {
		geometry_msgs::msg::PointStamped gtPose = deserializeMessage<geometry_msgs::msg::PointStamped>(serializedMsg);
    geometry_msgs::msg::PoseStamped::SharedPtr poseMsgPtr = std::make_shared<geometry_msgs::msg::PoseStamped>();
	  pointToPose(gtPose, poseMsgPtr);
	  bagOut.write(*poseMsgPtr, gt_topic_name, rovioNode->get_clock()->now());
	}
    rclcpp::spin_some(rovioNode);

    if(rovioNode->gotFirstMessages_){
      static double lastSafeTime = rovioNode->mpFilter_->safe_.t_;
      if(rovioNode->mpFilter_->safe_.t_ > lastSafeTime) {
        if(rovioNode->forceOdometryPublishing_)
        {
          bagOut.write(rovioNode->odometryMsg_, odometry_topic_name, rovioNode->get_clock()->now());
        }
        //if(rovioNode->forceTransformPublishing_) bagOut.write(transform_topic_name,rovioNode->get_clock()->now(),rovioNode->transformMsg_);
        for(int camID=0;camID<mtFilter::mtState::nCam_;camID++)
        {
          if(rovioNode->forceExtrinsicsPublishing_) {
            bagOut.write(rovioNode->extrinsicsMsg_[camID],
              extrinsics_topic_name[camID],rovioNode->get_clock()->now());
          }
        }
        if(rovioNode->forceImuBiasPublishing_) {
          bagOut.write(rovioNode->imuBiasMsg_,imu_bias_topic_name,rovioNode->get_clock()->now());
        }
        if(rovioNode->forcePclPublishing_) {
          bagOut.write(rovioNode->pclMsg_, pcl_topic_name, rovioNode->get_clock()->now());
        }
        if(rovioNode->forceMarkersPublishing_) {
          bagOut.write(rovioNode->odometryMsg_,odometry_topic_name,rovioNode->get_clock()->now());
        }
        if(rovioNode->forcePatchPublishing_) {
          bagOut.write(rovioNode->patchMsg_, patch_topic_name, rovioNode->get_clock()->now());
        }
        bagOut.write(rovioNode->healthMonitorMsg, health_topic_name, rovioNode->get_clock()->now());
        lastSafeTime = rovioNode->mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovioNode->requestReset();
        rovioNode->mpFilter_->init_.state_.WrWM() = rovioNode->mpFilter_->safe_.state_.WrWM();
        rovioNode->mpFilter_->init_.state_.qWM() = rovioNode->mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagOut.close();
  bagIn.close();


  return 0;
}
