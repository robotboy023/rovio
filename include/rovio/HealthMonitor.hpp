/*
* Copyright (c) 2026, Suyash Yeotikar
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
*/
/**
 * @file HealthMonitor.hpp
 * @author Suyash Yeotikar
 * @date Feb 16 2026
 */
#include "CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/PixelOutput.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio_interfaces/msg/health.hpp"

#include <rclcpp/time.hpp>

#ifndef ROVIO_HEALTHMONITOR_HPP
#define ROVIO_HEALTHMONITOR_HPP

template <unsigned int nMax_, int nLevels_, int patchSize_, int nCam_, int nPose_>
class HealthMonitor {
public:
  typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
private:
  rovio::TransformFeatureOutputCT<mtState> featureOutputTransformer_;
  rovio::PixelOutputCT pixelOutputTransformer_;
  rovio::FeatureOutput featureOutput_;
  rovio::PixelOutput pixelOutput_;
  Eigen::MatrixXd pixelOutputCovariance_;
public:
  float trackedFeatureRatio; //< Ratio of tracked features to max features
  float validFeatureRatio; //< Ratio of valid features to max features
  float NISZScoreRMSE; //< RMSE of NIS Z-score
  float featureDepthCovMedian; //< Median of depth covariances for all valid features
  float unhealthyVelocityDeviation; //< Deviation from unhealthy velocity threshold
  float accelDeviation; //< Deviation from the accel threshold
  float pixelCovRatio; //< Ratio of features with pixel covariances above a threshold.
  bool healthMsgValid; //< Boolean variable to control the publishing of the heaalth message.

  float pixelCovThreshold; //< Threshold above which pixel covariance for a feature is considered to be bad.
  float accelThreshold; //< Threshold above which acceleration value from accelerometer is considered to be bad.
  float velocityThreshold; //< Threshold above which velocity value estimated by ROVIO os considered to be bad.

public:

  HealthMonitor();
  /**
   * @brief Function to populate the health message for ROVIO.
   * @param filterState shared ptr to current state vector of ROVIO
   * @param healthMsg health message to be populated
   * @return None
   */
  void populateHealthMsg(const std::shared_ptr<mtFilter> mpFilter_,
    rovio_interfaces::msg::Health &healthMsg, std::string imu_frame) {
    // if ( !this->healthMsgValid) {
    //   return;
    // }
    healthMsg.accel_deviation = this->accelDeviation;
    healthMsg.speed_deviation = this->unhealthyVelocityDeviation;
    healthMsg.pixel_covariance_ratio = this->pixelCovRatio;
    healthMsg.accel_deviation = this->accelDeviation;
    healthMsg.nis_z_score_rmse =  this->NISZScoreRMSE;
    healthMsg.depth_feature_cov_median = featureDepthCovMedian;
    healthMsg.tracked_feature_ratio = this->trackedFeatureRatio;
    healthMsg.valid_feature_ratio = this->validFeatureRatio;
    healthMsg.header.frame_id = imu_frame;
    healthMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(1e9 * mpFilter_->safe_.t_));
  }



  /**
   * @brief Function to compute the median of the features depths covariances.
   * @param state Current state vector of ROVIO
   * @return float value that is the median of the depth covariances
   */
  float computeFeatureDepthCovMedian(const std::shared_ptr<mtFilter> mpFilter_ ) {
    Eigen::MatrixXd stateCovariance;
    stateCovariance = mpFilter_->safe_.cov_;
    auto &featureManager = mpFilter_->safe_.fsm_;
    std::vector<double> featureDepthCovariances;
    for (int i = 0; i < nMax_; i++ ) {
      if ( featureManager.isValid_[i] ) {
        double featureDepthCov = stateCovariance(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2);
        featureDepthCovariances.push_back(featureDepthCov);
      }
    }
    int sizeOfVec = featureDepthCovariances.size();
    if (sizeOfVec == 0 ) return 0;
    std::nth_element(featureDepthCovariances.begin(), featureDepthCovariances.begin() + sizeOfVec/2 , featureDepthCovariances.end());
    if (sizeOfVec % 2 !=  0 ) {
      this->featureDepthCovMedian = static_cast<float>(featureDepthCovariances[sizeOfVec/2]);
      return featureDepthCovMedian;
    } else {
      double val1 = featureDepthCovariances[sizeOfVec/2];
      double val2 = *std::max_element(featureDepthCovariances.begin(), featureDepthCovariances.begin() + sizeOfVec/2);
      featureDepthCovMedian = static_cast<float>((val1 + val2)/2);
      return featureDepthCovMedian;
    }
  }

  /**
   * @brief Function to compute the valid feature ratio
   * @param state Current state vector of ROVIO
   * @return float ratio of valid to max features.
   */
  float computeValidFeatureRatio(const std::shared_ptr<mtFilter> mpFilter_) {
    auto &featureManager = mpFilter_->safe_.fsm_;
    int validCount = 0;
    for (int i = 0; i < nMax_; i++ ) {
      if ( featureManager.isValid_[i] ) {
        validCount++;
      }
    }
    validFeatureRatio = static_cast<float>(validCount) / nMax_;
    return validFeatureRatio;
  }

  /**
   * @brief Function to compute the tracked feature ratio.
   * Feature has to be tracked atleast in 1 camera to be considered to be tracked
   * @param state Current state vector of ROVIO
   * @return float ratio of tracked to max features.
   */
  float computeTrackedFeatureRatio(const std::shared_ptr<mtFilter> mpFilter_) {
    auto &featureManager = mpFilter_->safe_.fsm_;
    int trackedCount = 0;
    for ( int i = 0; i < nMax_; i++ ) {
      bool featureTracked = false;
      if ( featureManager.isValid_[i] && featureManager.features_[i].mpStatistics_ != nullptr ) {
        for (int cam = 0; cam < nCam_; cam++ ) {
          featureTracked = featureTracked || featureManager.features_[i].mpStatistics_->status_[cam] == rovio::TRACKED;

          }
        }
      if ( featureTracked ) {
        trackedCount++;
      }
    }
    trackedFeatureRatio = static_cast<float>(trackedCount) / nMax_;
    return static_cast<float>(trackedCount) /nMax_;
  }

  /**
   * @brief Function to compute the RMSE of NIS z-score
   * @param state Current state vector of ROVIO
   * @return float RMSE of NIS zscore
   */
  float computeNISZScoreRMSE(const std::vector<double> &featureZScores) {
    if ( featureZScores.empty() ) {
      return 0.0;
    }
    double meanScore = std::accumulate(featureZScores.begin(), featureZScores.end(), 0.0)/ featureZScores.size();
    double totalDiffSquared = 0.0;
    for ( double score : featureZScores ) {
      double diff = score - meanScore;
      double diffSquared = diff * diff;
      totalDiffSquared += diffSquared;
    }
    double RMSE = sqrt( totalDiffSquared/ featureZScores.size());
    this->NISZScoreRMSE = sqrt(RMSE);
    return static_cast<float>(RMSE);
  }

  /**
   * @brief Function to compute the ratio of features above a pixel covariance threshold
   * If pixel covariance is greater than threshold in one camera, it is considered for the count.
   * Bad pixel covariance even from one camera can corrupt the filter state in update.
   * @param mtFilter &state
   * @return float ratio of number of features below pixel covariance threshold to max features
   * @note There might be a scope of overcounting features in multi-camera case. Investigate later.
   */
  float computePixelCovRatio( const std::shared_ptr<mtFilter> mpFilter_) {
    auto state = mpFilter_->safe_.state_;
    Eigen::MatrixXd stateCovariance = mpFilter_->safe_.cov_;
    int count = 0;
    featureOutputTransformer_.mpMultiCamera_ = &mpFilter_->multiCamera_;
    Eigen::MatrixXd featureCovariance;
    auto &featureManager = mpFilter_->safe_.fsm_;
    for (int i = 0; i < nMax_; i++ ) {
      bool featureCovarianceAboveThreshold = false;
      for (int camID = 0; camID < nCam_; camID++) {
        if ( !featureManager.isValid_[i]) {
          continue;
        }
        Eigen::MatrixXd featureCovariance;
        featureOutputTransformer_.setFeatureID(i);
        featureOutputTransformer_.setOutputCameraID(camID);
        featureOutputTransformer_.transformState(state, featureOutput_);
        featureOutputTransformer_.transformCovMat(state, stateCovariance, featureCovariance );
        pixelOutputTransformer_.transformState(featureOutput_, pixelOutput_);
        pixelOutputTransformer_.transformCovMat(featureOutput_, featureCovariance, pixelOutputCovariance_);
        Eigen::Vector2d eigValues = pixelOutputCovariance_.eigenvalues().real();
        double eigValueNorm =  eigValues.norm();
        featureCovarianceAboveThreshold = featureCovarianceAboveThreshold || (eigValueNorm > pixelCovThreshold);
      }
      if ( featureCovarianceAboveThreshold ) { count++; }
    }
    pixelCovRatio = static_cast<float>(count) / nMax_;
    return static_cast<float>(count) / nMax_;
  }


  /**
   * @brief Function to compute the deviation of speed from threshold value
   * @param threshold value of velocity
   * @param velocity estimated ROVIO
   * @return double difference of velocity and speed
   */

  double computeUnhealthyVelocityDeviation(Eigen::Vector3d rovioVelocity) {
    double velocityNorm = rovioVelocity.norm();
    unhealthyVelocityDeviation = std::abs(velocityThreshold - velocityNorm);
    return unhealthyVelocityDeviation;
  }

  /**
   * @brief Function to compute the deviation of IMU accelration from threshold value
   * Helpful to detect spikes
   * @param threshold value of acceleration
   * @param IMU accel reading
   */
  double computeAccelDeviation(Eigen::Vector3d IMUAcceleration ) {
        double IMUAccelNorm = IMUAcceleration.norm();
        accelDeviation = std::abs(IMUAccelNorm - accelThreshold);
        return std::abs(accelThreshold - IMUAccelNorm);
  }
};

template <unsigned int nMax_, int nLevels_, int patchSize_, int nCam_, int nPose_>
HealthMonitor<nMax_, nLevels_, patchSize_, nCam_, nPose_>::HealthMonitor()
  : trackedFeatureRatio(0.0f),
    validFeatureRatio(0.0f),
    NISZScoreRMSE(0.0f),
    featureDepthCovMedian(0.0f),
    unhealthyVelocityDeviation(0.0f),
    accelDeviation(0.0f),
    pixelCovRatio(0.0f),
    healthMsgValid(false),
    pixelCovThreshold(0.0f),
    accelThreshold(0.0f),
    velocityThreshold(0.0f),
    featureOutputTransformer_(nullptr){}

#endif // ROVIO_HEALTHMONITOR_HPP

