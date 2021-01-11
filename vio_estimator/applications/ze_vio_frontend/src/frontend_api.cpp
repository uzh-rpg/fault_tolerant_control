// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/frontend_api.hpp>

#include <imp/core/image.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/data_provider/camera_imu_synchronizer.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/imu_integrator.hpp>
#include <ze/vio_frontend/frontend_base.hpp>
#include <ze/vio_frontend/frontend_vio.hpp>
#include <ze/imu/imu_model.hpp>
#include <ze/imu/imu_rig.hpp>

namespace ze {

void VisualOdometry::initialize()
{
  // Create frontend.
  frontend_ = std::make_shared<FrontendVio>();
}

void VisualOdometry::subscribeDataProviders()
{
  // Init data provider.
  data_provider_ = ze::loadDataProviderFromGflags(frontend_->rig_->size());

  // Init data synchronization and register callback.
  data_sync_ = std::make_shared<CameraImuSynchronizer>(
        *data_provider_);

  CHECK_GE(frontend_->rig_->size(), 1u);
  data_sync_->registerCameraImuCallback(
    std::bind(
        static_cast<void(FrontendBase::*)(
          const StampedImages&    /*image*/,
          const ImuStampsVector& /*imu_timestamps*/,
          const ImuAccGyrVector& /*imu_measurements*/,
          const double& /*height measurement*/
        )>(&FrontendBase::addData),
      frontend_.get(),
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4));

  data_sync_->registerImuCallback(
        std::bind(&FrontendBase::addImuData, frontend_.get(),
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4));
}

void VisualOdometry::startBlocking()
{
  VLOG(2) << "API Input: Start";
  CHECK(data_provider_) << "Did you call subscribeDataProviders()?";
  frontend_->stage_ = FrontendStage::AttitudeEstimation;
  data_provider_->spin();
}

void VisualOdometry::shutdown()
{
  VLOG(2) << "API Input: Shutdown";
  CHECK(data_provider_);
  CHECK(frontend_);
  data_provider_->shutdown();
  frontend_->shutdown();
}

void VisualOdometry::pause()
{
  VLOG(2) << "API Input: Pause";
  data_provider_->pause();
}

void VisualOdometry::registerResultCallback(const VisualOdometryCallback& cb)
{
  CHECK(frontend_);
  frontend_->registerResultCallback(cb);
}

} // namespace ze
