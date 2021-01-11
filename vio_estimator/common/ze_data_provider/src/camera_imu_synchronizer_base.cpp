// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#include <ze/data_provider/camera_imu_synchronizer_base.hpp>

#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/common/time_conversions.hpp>

DEFINE_int32(data_sync_init_skip_n_frames, 0,
             "How many frames should be skipped at the beginning.");

DEFINE_int32(data_sync_stop_after_n_frames, -1,
             "How many frames should be processed?");

namespace ze {

CameraImuSynchronizerBase::CameraImuSynchronizerBase(
                    DataProviderBase& data_provider)
  : num_imus_(data_provider.imuCount())
  , num_cameras_(data_provider.cameraCount())
  , timeshift_cam_imu_(secToNanosec(FLAGS_timeshift_cam_imu))
{
  last_package_stamp_ = -1;

  height_mea_ = 1.0; // hack initial height
  LOG(INFO) << "t_cam = t_imu - " << timeshift_cam_imu_ << " [ns]";
}

void CameraImuSynchronizerBase::registerCameraImuCallback(
    const SynchronizedImageImuCallback& callback)
{
  image_imu_callback_ = callback;
}


void CameraImuSynchronizerBase::addImageData(
    int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx)
{
  CHECK_LT(camera_idx, num_cameras_);

  // Correct for timestamp delay between IMU and Frames.
  stamp += timeshift_cam_imu_;

  // Skip frame processing for first N frames.
  if (sync_frame_count_ < FLAGS_data_sync_init_skip_n_frames)
  {
    if (camera_idx == 0)
    {
      ++sync_frame_count_;
    }
    return;
  }

  // Add image to first available slot in our buffer:
  int slot = -1;
  for (size_t i = 0u; i < image_buffer_.size(); ++i)
  {
    if (image_buffer_[i].empty())
    {
      slot = i;
      break;
    }
  }

  if (slot == -1)
  {
    // No space in buffer to process frame. Delete oldest one. This happens
    // also when the processing is not fast enough such that frames are skipped.
    int64_t min_stamp = std::numeric_limits<int64_t>::max();
    for (size_t i = 0u; i < image_buffer_.size(); ++i)
    {
      if (!image_buffer_[i].empty() && image_buffer_[i].stamp < min_stamp)
      {
        slot = i;
        min_stamp = image_buffer_[i].stamp;
      }
    }
  }

  image_buffer_[slot].stamp = stamp;
  image_buffer_[slot].img = img;
  image_buffer_[slot].camera_idx = camera_idx;

  // Now check, if we have all images from this bundle:
  uint32_t num_imgs = 0u;
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    if (std::abs(stamp - image_buffer_[i].stamp) < millisecToNanosec(2))
    {
      ++num_imgs;
    }
  }

  if (num_imgs != num_cameras_)
  {
    return; // We don't have all frames yet.
  }

  // We have frames with very close timestamps. Put them together in a vector.
  sync_images_ready_to_process_.clear();
  sync_images_ready_to_process_.resize(num_imgs, {-1, nullptr});
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    ImageBufferItem& item = image_buffer_[i];
    if (std::abs(stamp - item.stamp) < c_camera_bundle_time_accuracy_ns)
    {
      DEBUG_CHECK_GT(item.stamp, 0);
      DEBUG_CHECK(item.img);
      sync_images_ready_to_process_.at(item.camera_idx) = { item.stamp, item.img };
    }
  }

  // Double-check that we have all images.
  for (size_t i = 0; i < sync_images_ready_to_process_.size(); ++i)
  {
    if (sync_images_ready_to_process_.at(i).first == -1)
    {
      LOG(ERROR) << "Sync images failed!";
      sync_images_ready_to_process_.clear();
      return;
    }
  }
  sync_imgs_ready_to_process_stamp_ = sync_images_ready_to_process_.front().first;

  checkImuDataAndCallback();
}

void CameraImuSynchronizerBase::addHeightData(
    int64_t /*stamp*/, const double & height)
{
    height_mea_ = height;
}

bool CameraImuSynchronizerBase::validateImuBuffers(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::vector<std::tuple<int64_t, int64_t, bool> >&
    oldest_newest_stamp_vector)
{
  // Check if we have received some IMU measurements for at least one of the imu's.
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
  {
                   if (std::get<2>(oldest_newest_stamp))
  {
                   return true;
}
                   return false;
})
      )
  {
    LOG(WARNING) << "Received all images but no imu measurements!";
    return false;
  }

  // At least one IMU measurements before image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&min_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
  {
                   if (std::get<0>(oldest_newest_stamp) < min_stamp) {
                   return true;
}
                   return false;
})
      )
  {
    LOG(WARNING) << "Oldest IMU measurement is newer than image timestamp.";
    return false;
  }

  // At least one IMU measurements after image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&max_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp) {
                   if (std::get<1>(oldest_newest_stamp) > max_stamp)
  {
                   return true;
}
                   return false;
})
      )
  {
    VLOG(100) << "Waiting for IMU measurements.";
    return false;
  }

  return true;
}

} // namespace ze
