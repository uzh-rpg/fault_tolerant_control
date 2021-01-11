// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <functional>
#include <fstream>
#include <vector>
#include <memory>

#include <imp/core/image.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/timer_collection.hpp>
#include <ze/common/thread_pool.hpp>
#include <ze/common/thread_safe_fifo.hpp>
#include <ze/common/types.hpp>
#include <ze/common/running_statistics_collection.hpp>
#include <ze/common/ringbuffer.hpp>
#include <ze/vio_frontend/frontend_api.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/motion_type.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/localization.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

DECLARE_bool(vio_activate_backend);
DECLARE_int32(vio_add_every_nth_frame_to_backend);
DECLARE_double(vio_min_depth);
DECLARE_double(vio_max_depth);
DECLARE_double(vio_median_depth);
DECLARE_uint64(vio_min_tracked_features);
DECLARE_uint64(vio_max_landmarks);

namespace ze {

// fwd.
class CameraRig;
using CameraRigPtr = std::shared_ptr<CameraRig>;
class DepthFilter;
class FeatureInitializer;
class FeatureTracker;
class ImuIntegrator;
class NFrame;
class StereoMatcher;
class VioVisualizer;

// callback declarations.
using TrackedNFrameCallback =
  std::function<void(const std::shared_ptr<NFrame>&,
                     const ImuStamps& imu_stamps,
                     const ImuAccGyrContainer& imu_accgyr,
                     const VioMotionType,
                     const std::vector<LandmarkHandle>& lm_opportunistic,
                     const std::vector<LandmarkHandle>& lm_persistent_new,
                     const std::vector<LandmarkHandle>& lm_persistent_continued,
                     const double& height_mea)>;
using UpdateStatesCallback = std::function<bool(const bool wait_for_backend)>;


//! Tracking frontend: Extracts features, tracks them and calls back-end for
//! optimization.
class FrontendBase : Noncopyable
{
protected:
  //! Default constructur loads all data from gflags.
  FrontendBase();

  //! Use provided camera and load other modules from gflags.
  FrontendBase(const CameraRigPtr& rig);

  //! Default destructor.
  virtual ~FrontendBase();

  //! Derived class must implement this functionality.
  virtual void processData(const Transformation& T_Bkm1_Bk) = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! @name Interface
  //! @{
  /// Uses only images.
  // void addData(const std::pair<int64_t, EventArrayPtr>& stamped_events,
  //              const std::vector<ImuStamps>& imu_stamps_vec,
  //              const std::vector<ImuAccGyrContainer>& imu_accgyr_vec,
  //              const double& height_mea);

  /// Uses only events
  void addData(const std::vector<std::pair<int64_t, ImageBase::Ptr>>&
                stamped_images,
               const std::vector<ImuStamps>& imu_stamps_vec,
               const std::vector<ImuAccGyrContainer>& imu_accgyr_vec,
               const double& height_mea);

  // /// Uses both events and images
  // void addData(const std::vector<std::pair<int64_t, ImageBase::Ptr>>&
  //              stamped_images,
  //              const std::pair<int64_t, EventArrayPtr>& stamped_events,
  //              const std::vector<ImuStamps>& imu_stamps_vec,
  //              const std::vector<ImuAccGyrContainer>& imu_accgyr_vec,
  //              const bool &no_motion_prior,
  //              const double& height_mea);

  void reset();

  void shutdown();
  //! @}

  // void processData(
  //     const std::pair<int64_t, EventArrayPtr>& stamped_events,
  //     const std::vector<ImuStamps>& imu_timestamps,
  //     const std::vector<ImuAccGyrContainer>& imu_measurements,
  //     const double& height_mea);

  void processData(
      const std::vector<std::pair<int64_t, ImageBase::Ptr>>& stamped_images,
      const std::vector<ImuStamps>& imu_timestamps,
      const std::vector<ImuAccGyrContainer>& imu_measurements,
      const double& height_mea);

  // void processData(const std::vector<std::pair<int64_t, ImageBase::Ptr>>& stamped_images,
  //     const std::pair<int64_t, EventArrayPtr>& stamped_events,
  //     const std::vector<ImuStamps>& imu_timestamps,
  //     const std::vector<ImuAccGyrContainer>& imu_measurements,
  //     const bool &no_motion_prior,
  //     const double& height_mea);

  std::shared_ptr<NFrame> createNFrame(
      const std::vector<std::pair<int64_t, ImageBase::Ptr>>& stamped_images);

  std::pair<std::vector<real_t>, uint32_t> trackFrameKLT();

  /// \brief Adds given imu measurements to members imu_stamps_since_lkf
  /// and imu_accgyr_since_lkf.
  /// It also removes potential interpolated values, which are not measurements.
  /// \param [in] IMU stamps.
  /// \param [in] IMU measurements, in the same order as the stamps.
  bool addImuMeasurementsBetweenKeyframes(
      const ImuStamps& imu_stamps,
      const ImuAccGyrContainer& imu_accgyr);

  void cleanupInactiveLandmarksFromLastIteration();

  VioMotionType classifyMotion(
      std::vector<real_t>& disparities_sq,
      const uint32_t n_outliers);

  void addImuData(
      int64_t stamp,
      const Vector3& acc,
      const Vector3& gyr,
      const uint32_t imu_idx);

  //! @name Callback registration
  //! @{
  inline void registerTrackedNFrameCallback(const TrackedNFrameCallback& cb)
  {
    tracked_nframe_cb_ = cb;
  }

  inline void registerUpdateStatesCallback(const UpdateStatesCallback& cb)
  {
    update_states_cb_ = cb;
  }

  inline void registerResultCallback(const VisualOdometryCallback& cb)
  {
    result_cb_ = cb;
  }
  //! @}

  // Modules
  std::shared_ptr<const CameraRig> rig_;
  std::shared_ptr<ImuIntegrator> imu_integrator_;
  std::shared_ptr<StereoMatcher> stereo_matcher_;
  std::shared_ptr<FeatureTracker> feature_tracker_;
  std::shared_ptr<FeatureInitializer> feature_initializer_;
  std::shared_ptr<VioVisualizer> visualizer_;
   
  ThreadPool thread_pool_;

  // System State
  int frame_count_ = -1;                //!< Frame counter.
  int imu_meas_count_ = -1;                //!< Frame counter.
  uint32_t num_tracked_ = 0;
  const TransformationVector T_C_B_;    //!< Camera extrinsics.
  Vector3 t_Bkm1_Bk_ = Vector3::Zero(); //! Last relative translation, used for motion model.
  LandmarkTable landmarks_;
  NFrameTable states_;
  FrontendStage stage_ = FrontendStage::AttitudeEstimation;
  ImuStamps imu_stamps_since_lkf_;
  ImuAccGyrContainer imu_accgyr_since_lkf_;
  LocalizationInformation localization_information_;
  real_t scene_depth_;

  // Temporaries
  int attitude_init_count_ = 0;       //!< Number of frames used for attitude estimation.
  VioMotionType motion_type_ = VioMotionType::GeneralMotion;

  // Timing and Statistics
  DECLARE_TIMER(Timer, timers_,
                create_frame, add_landmark_observations, project_landmarks,
                optimize_pose, seed_update_tot, remove_old_landmarks, scene_depth,
                initialize_features, seed_update_initial, optimize_landmarks, compute_scene_depth, retriangulate_all_landmarks,
                track_features, ransac_relative_pose, detect_features, track_selection,
                descriptor_extraction, add_frame_to_backend, total_time, visualization, wait_time,
                draw_events, wait_for_backend, project_events, draw_event_frames, triangulate_new_andmarks
                );
  DECLARE_STATISTICS(Stats, stats_, num_landmarks, img_align_num_iter,
                     img_align_num_measurements, frame_length
                     );

  // Logging.
  void logNumTrackedFeatures(
      const NFrame& nframe,
      const LandmarkTable& landmarks);

private:
  void initModules();

  /// Initialize Dvs related members;
  void initDvs();
  cv::Mat dvs_img_;

  size_t height_;
  size_t width_;
  cv::Size sensor_size_;
  cv::Rect rect_;

  real_t fx_;
  real_t fy_;
  real_t cx_;
  real_t cy_;

  Vector3 v_W_last_;
  bool limitNumberOfFrames(int max_frames);

  bool pollBackend(bool block=false);

  // void drawEvents(const EventArray::iterator& first,
  //     const EventArray::iterator& last,
  //     const int64_t& t0,
  //     const int64_t& t1,
  //     const Transformation& T_1_0,
  //     const bool& do_motion_correction,
  //     cv::Mat& out);

protected:
  TrackedNFrameCallback tracked_nframe_cb_;
  UpdateStatesCallback update_states_cb_;
  VisualOdometryCallback result_cb_;

  Ringbuffer<real_t, 6, 1000> imu_buffer_;
  Odometry odom_;

  //! @name Threading:
  //! @{
  void startThread();

  void stopThread();

  void frontendLoop();

  struct FrontendInputData
  {
    std::vector<std::pair<int64_t, ImageBase::Ptr>> stamped_images;
    std::pair<int64_t, EventArrayPtr> stamped_events;
    std::vector<ImuStamps> imu_stamps_vec;
    std::vector<ImuAccGyrContainer> imu_accgyr_vec;
    double height_mea;
  };

  ThreadSafeFifo<FrontendInputData, 5> input_queue_;
  std::unique_ptr<std::thread> thread_;
  std::atomic_bool stop_thread_ { false };
  //! @}
};

} // namespace ze
