## Application Modules

#### `applications/ze_ceres_backend`
Visual-Inertial sliding window filter using Ceres (based on OKVIS).

#### `applications/ze_vio_ceres`
Visual-Inertial Odometry based on ze_vio_frontend and ze_ceres_backend.

#### `applications/ze_vio_frontend`
Feature tracking frontend based on KLT tracking + 2pt or 5pt RANSAC

## Common Modules

#### `common/ze_cameras`
Camera models for Pinhole projections with FoV, RadTan, Equidistant distortions.

#### `common/ze_cmake`
Common CMake configurations. Use `include(ze_setup)` in your CMakeLists.txt to set common flags.

#### `common/ze_common`
Contains many utility functions and typedefs.

#### `common/ze_data_provider`
The data provider abstracts several common data sources for images and inertial measurements, such as listening to a ROS topic, reading a rosbag, or a csv file. The data provider further provides synchronization mechanisms.

#### `common/ze_geometry`
Several geometic vision tools such as relative pose RANSAC solvers, nonlinear least squarse refinement of reprojection errors to estimate the pose, triangulation, point alignment.

#### `common/ze_imu`
Implementation of several accelerometer and gyroscope noise and distortion models as the can be calibrated with Kalibr. Imu buffer allows to synchronize separate accel and gyro measurements with different interpolation methods.

#### `common/ze_ros`
Utilities to work with ROS, e.g. message converters.

#### `common/ze_vio_common`
Common containers and types for visual inertial odometry.

## Image Processing Modules

#### `imp/imp_bridge_opencv`
Interface with OpenCV.

#### `imp/imp_bridge_ros`
Interface with ROS.

#### `imp/imp_core`
Main image processing classes: ImageBase, ImageRaw, PixelTypes, Memory.

#### `imp/imp_correspondence`
Sparse stereo matching, Lucas Kanade tracking.

#### `imp/imp_feature_matching`
Feature matching of binary features.

#### `imp/imp_features`
Extraction of BRISK, FAST, ORB features and descriptors.

#### `imp/imp_imgproc`
Image pyramid.

## Visualization Modules

#### `ui/ze_visualization`
Wrapper around RVIZ-based visualization.
