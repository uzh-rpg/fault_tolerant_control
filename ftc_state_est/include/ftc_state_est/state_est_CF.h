// Copyright (C) 2021 Sihao Sun, RPG, University of Zurich, Switzerland
// Copyright (C) 2021 Davide Scaramuzza, RPG, University of Zurich, Switzerland
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <ros/time.h>
#include <sstream>

#define PI 3.14159265

namespace state_est_CF
{  
  class StateEstCF
  {
  public:

    StateEstCF(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    
    StateEstCF():
      StateEstCF(ros::NodeHandle(), ros::NodeHandle("~")){};

    ~StateEstCF();

  private:
    
    // --- Node handlers ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // ---

    // --- Publishers & Subscribers ---
    ros::Subscriber imu_sub_, alt_sub_, odom_sub_, pos_optitrack_sub_, joy_sub_;
    ros::Publisher state_est_pub_;

    // ---
    ros::Time imuTime_last_;
    ros::Time imuTime_;
    ros::Time publishTime_;
    sensor_msgs::Joy joy_msg_;

    double dt_;
    float g_;
    int pub_msg_seq_;
    int N_yaw_;
    int N_yaw_optitrack_;

    //IMU related variables
    bool IMU_read_;
    Eigen::Vector3f rate_, rate_corrected_;
    Eigen::Vector3f acc_, acc_corrected_;
    Eigen::Vector3f gyro_bias_, acc_bias_;

    Eigen::Quaternionf q_est_  = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaternionf q_est0_ = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3f att_err_;
    Eigen::Vector3f att_err_integrated_;

    //CF related gains
    float ka_, kb_;
    float kyaw_, kbyaw_;

    float pitch_, roll_, yaw_;
    float height_, height_mea_, velz_;
    float range_, range_last_;
    float yaw_mea_, yaw_mea_last_;
    float yaw_err_, yaw_err_integrated_;
    float yaw_mea_optitrack_, yaw_mea_optitrack_last_;

    float estimate_rate_;
    bool displacment_compensation_;
    Eigen::Vector3f displacement_;
    bool rotation_compensation_;

    Eigen::Matrix2f P_, F_, Q_;
    Eigen::MatrixXf H_ = Eigen::MatrixXf::Zero(1,2);

    float R_, q_;

    Eigen::Vector3f pos_cg_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel_cg_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f p_bc_   = Eigen::Vector3f::Zero(); 
    Eigen::Quaternionf quat_cb_ = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaternionf quat_ic_ = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);

    bool use_vio_=false;
    bool odometry_called_=false;
    
    double time_vio_;
    bool delay_compensation_;

    Eigen::Matrix4f P_posvel_;
    Eigen::Vector3f pos_est_, vel_est_;
    Eigen::Vector3f gvec_;
    float Q_pos_, Q_vel_;
    float R_pos_, R_vel_;
    
    ///////////////////////////////
    void initialization();
    void initializeHeightEKF();
    

    void ImuCallBack(const sensor_msgs::ImuConstPtr& msg);
    void rangeCallBack(const sensor_msgs::RangeConstPtr& msg);
    void odometryCallBack(const nav_msgs::OdometryConstPtr& msg);
    void poseOptitrackCallBack(const geometry_msgs::PoseStampedConstPtr& msg);
    void joyCallback(const sensor_msgs::JoyConstPtr msg);

    void integrator3(const Eigen::Vector3f& input, Eigen::Vector3f& output, const float dt, const float max);
    void integrator(const float& input, float& output, const float dt, const float max);
    
    void calculateDifference();
    void attIntegration();
    void gyroCorrection();
    Eigen::Vector3f rotationCompensation();
    Eigen::Vector3f quaternion2euler(Eigen::Quaternionf q);    

    void heightEKF();
    void posVelEKF();

    void publishEstimation();

    bool loadParams();
    void run();


  };

} // namespace state_est
