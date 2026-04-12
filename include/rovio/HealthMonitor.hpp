/** Paste the ETH licence preamble here
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
    healthMsg.total_feature_ratio = this->validFeatureRatio;
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
      return static_cast<float>(featureDepthCovariances[sizeOfVec/2]);
    } else {
      double val1 = featureDepthCovariances[sizeOfVec/2];
      double val2 = *std::max_element(featureDepthCovariances.begin(), featureDepthCovariances.begin() + sizeOfVec/2);
      return static_cast<float>( ( val1 + val2 ) /2);
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
   * @param state Current state vector of ROVIO
   * @return float ratio of tracked to max features.
   */
  float computeTrackedFeatureRatio(const std::shared_ptr<mtFilter> mpFilter_) {
    auto &featureManager = mpFilter_->safe_.fsm_;
    int trackedCount = 0;
    for ( int i = 0; i < nMax_; i++ ) {
      if ( featureManager.isValid_[i] && featureManager.features_[i].mpStatistics_ != nullptr ) {
        for (int cam = 0; cam < nCam_; cam++ ) {
          if ( featureManager.features_[i].mpStatistics_->status_[cam] == rovio::TRACKED ) {
            trackedCount++;
          }
        }
      }
    }
    trackedFeatureRatio = static_cast<float>(trackedCount) / nMax_;
    return static_cast<float>(trackedCount) /nMax_;
  }

  /**
   * @breif Function to compute the RMSE of NIS z-score
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
    return static_cast<float>(RMSE);
  }

  /**
   * @brief Function to compute the ratio of features above a pixel covariance threshold
   * @param mtFilter &state
   * @return float ratio of number of features below pixel covariance threshold to max features
   * @note There might be a scope of overcounting features in multi-camera case. Investigate later.
   */
  float computePixelCovRatio( const std::shared_ptr<mtFilter> mpFilter_) {
    auto state = mpFilter_->safe_.state_;
    Eigen::MatrixXd stateCovariance = mpFilter_->safe_.cov_;
    int count = 0;
    featureOutputTransformer_.mpMultiCamera_ = &mpFilter_->multiCamera_;
    for (int i = 0; i < nMax_; i++ ) {
      for (int camID = 0; camID < nCam_; camID++) {
        Eigen::MatrixXd featureCovariance;
        featureOutputTransformer_.setFeatureID(i);
        featureOutputTransformer_.setOutputCameraID(camID);
        featureOutputTransformer_.transformState(state, featureOutput_);
        featureOutputTransformer_.transformCovMat(state, stateCovariance, featureCovariance );
        Eigen::Vector2d eigValues = featureCovariance.eigenvalues().real();
        double eigValueNorm =  eigValues.norm();
        if (eigValueNorm > pixelCovThreshold ) {
          count++;
        }
      }
    }
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
    return std::abs(velocityThreshold -  velocityNorm);
  }

  /**
   * @brief Function to compute the deviation of IMU accelration from threshold value
   * Helpful to detect spikes
   * @param threshold value of acceleration
   * @param IMU accel reading
   */
  double computeAccelDeviation(Eigen::Vector3d IMUAcceleration ) {
        double IMUAccelNorm = IMUAcceleration.norm();
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

