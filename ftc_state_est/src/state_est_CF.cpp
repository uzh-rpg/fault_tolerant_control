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


#include "ftc_state_est/state_est_CF.h"

namespace state_est_CF {
  StateEstCF::StateEstCF(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      :nh_(nh), pnh_(pnh), pub_msg_seq_(0) {    
    loadParams();

    imu_sub_   = nh_.subscribe("imu", 1, &StateEstCF::ImuCallBack,this);
    pos_optitrack_sub_   = nh_.subscribe("optitrack/cuckoo", 1, &StateEstCF::poseOptitrackCallBack, this);
    alt_sub_   = nh_.subscribe("/terarangerone", 1,   &StateEstCF::rangeCallBack, this);
    odom_sub_  = nh_.subscribe("/ze_vio/odometry", 1, &StateEstCF::odometryCallBack, this);  
    joy_sub_ = nh_.subscribe("joy", 1, &StateEstCF::joyCallback, this );

    state_est_pub_ = nh_.advertise<quad_msgs::QuadStateEstimate>("state_est", 1);
    
    initialization();
  }

  StateEstCF::~StateEstCF() {}

  void StateEstCF::initialization() {   

      ROS_INFO("[%s] Initializing...!", pnh_.getNamespace().c_str());

      dt_ = 1.0/estimate_rate_;
      g_ = 9.81;
      gvec_ << 0.0, 0.0, -g_;

      IMU_read_ = false;
      roll_ = 0.0;
      pitch_ = 0.0;

      att_err_integrated_ = Eigen::Vector3f::Zero();
      att_err_ = Eigen::Vector3f::Zero();
      
      height_ = 0.0;
      velz_ = 0.0;
      range_last_ = 0.0;
      yaw_ = 0.0;
      yaw_err_ = 0.0;
      yaw_err_integrated_ = 0.0;
      yaw_mea_ = 0.0;
      yaw_mea_last_ = 0.0;
      N_yaw_ = 0;

      yaw_mea_optitrack_last_ = 0.0;
      yaw_mea_optitrack_ = 0.0;
      N_yaw_optitrack_= 0;

      // EKF setup
      P_ = Eigen::Matrix2f::Identity();
      P_ << 0.1, 0.0,
            0.0, 0.000001;
      H_ << 0.0, 1.0;
            
      Eigen::Vector4f vecp;
      vecp << 0.0, 0.0, 0.0, 0.0;
      P_posvel_ = vecp.asDiagonal();
      
      pos_est_ << 0.0, 0.0, 0.0;
      vel_est_ << 0.0, 0.0, 0.0;
      acc_corrected_ << 0.0, 0.0, 10.0;
      
      ROS_INFO("[%s] Initialized!", pnh_.getNamespace().c_str());
      return;
  }


  bool StateEstCF::loadParams() {

    float dx, dy, dz;
    float bgx, bgy, bgz;
    float bax, bay, baz;
    float q;

    bool check = true;

    pnh_.param("estimate_rate", estimate_rate_, 200.0f);
    pnh_.param("p_gain_cf", ka_, 10.0f);
    pnh_.param("i_gain_cf", kb_, 1.0f);
    pnh_.param("p_gain_cf_yaw", kyaw_, 10.0f);
    pnh_.param("i_gain_cf_yaw", kbyaw_, 1.0f);
    
    pnh_.param("Q", q_, 1.0f);
    pnh_.param("R", R_, 1.0f);
    pnh_.param("Q_pos", Q_pos_, 1.0f);
    pnh_.param("Q_vel", Q_vel_, 1.0f);
    pnh_.param("R_pos", R_pos_, 1.0f);
    pnh_.param("R_vel", R_vel_, 1.0f); 

    pnh_.param("dx", dx, 0.0f);
    pnh_.param("dy", dy, 0.0f);
    pnh_.param("dz", dz, 0.0f);
    pnh_.param("bgx", bgx, 0.0f);
    pnh_.param("bgy", bgy, 0.0f);
    pnh_.param("bgz", bgz, 0.0f);    
    pnh_.param("bax", bax, 0.0f);
    pnh_.param("bay", bay, 0.0f);
    pnh_.param("baz", baz, 0.0f);     
    pnh_.param("displacment", displacment_compensation_, false);
    pnh_.param("rotation", rotation_compensation_, false);
    pnh_.param("delay", delay_compensation_,true);

    displacement_ << dx, dy, dz;
    gyro_bias_ << bgx, bgy, bgz;
    acc_bias_ << bax, bay, baz;

    return check;           
  }
  void StateEstCF::rangeCallBack(const sensor_msgs::RangeConstPtr& msg) {

    range_ = msg->range;

    if (range_last_ > 0.5) {
      if (range_ > 1.05 * range_last_)
        range_ = range_last_ * 1.05;
      else if (range_ < range_last_ / 1.05)
        range_ = range_last_ / 1.05;
    }
    range_last_ = range_;
    
    return;
  }

  void StateEstCF::joyCallback(const sensor_msgs::JoyConstPtr msg) {
    joy_msg_ = *msg;

    if(!use_vio_)
      use_vio_ =   static_cast<bool>(joy_msg_.buttons[3]);
  }

  void StateEstCF::poseOptitrackCallBack(const geometry_msgs::PoseStampedConstPtr& msg) {
    // If the VIO is not avaliable, use the Optitrack provided yaw

    Eigen::Quaternionf orientation_mea;
    orientation_mea.w() = msg->pose.orientation.w;
    orientation_mea.x() = msg->pose.orientation.x;
    orientation_mea.y() = msg->pose.orientation.y;
    orientation_mea.z() = msg->pose.orientation.z;

    auto euler = quaternion2euler(orientation_mea);
    // unwrap yaw from [-pi, pi]
    if ((euler(2) - yaw_mea_optitrack_last_) > PI)
      N_yaw_optitrack_--;
    else if ((euler(2) - yaw_mea_optitrack_last_) < -PI)
      N_yaw_optitrack_++;

    yaw_mea_optitrack_last_ = euler(2);
    yaw_mea_optitrack_ = euler(2) + static_cast<float>(N_yaw_optitrack_) * 2.0 * PI;
  }

  void StateEstCF::ImuCallBack(const sensor_msgs::ImuConstPtr& msg) {

    rate_(0) =  msg->angular_velocity.x;
    rate_(1) =  msg->angular_velocity.y;
    rate_(2) =  msg->angular_velocity.z;

    acc_(0) =  msg->linear_acceleration.x;
    acc_(1) =  msg->linear_acceleration.y;
    acc_(2) =  msg->linear_acceleration.z;
    
    imuTime_ = msg->header.stamp;
    publishTime_ = imuTime_;

    if (IMU_read_)  {
      dt_ = imuTime_.toSec() - imuTime_last_.toSec();
      imuTime_last_ = imuTime_;
    }
    else  {
      imuTime_last_ = imuTime_;
      IMU_read_ = true;
      return;
    }

    if (dt_<0.0)
      return;

    run();

  }

  void StateEstCF::odometryCallBack(const nav_msgs::OdometryConstPtr& msg) {

    if (!odometry_called_)
      odometry_called_ = true;

    ros::Time odometry_time = msg->header.stamp;
    time_vio_ = odometry_time.toSec();
    
    pos_cg_(0) = msg->pose.pose.position.x;
    pos_cg_(1) = msg->pose.pose.position.y;
    pos_cg_(2) = msg->pose.pose.position.z;

    quat_ic_.x() = msg->pose.pose.orientation.x;
    quat_ic_.y() = msg->pose.pose.orientation.y;
    quat_ic_.z() = msg->pose.pose.orientation.z;
    quat_ic_.w() = msg->pose.pose.orientation.w;
    
    quat_ic_.normalize();

    vel_cg_(0) = msg->twist.twist.linear.x;
    vel_cg_(1) = msg->twist.twist.linear.y;
    vel_cg_(2) = msg->twist.twist.linear.z;

    auto euler = quaternion2euler(quat_ic_ * quat_cb_);

    // unwrap yaw from [-pi, pi]
    if ((euler(2) - yaw_mea_last_) > PI)
      N_yaw_--;
    else if ((euler(2) - yaw_mea_last_) < -PI)
      N_yaw_++;

    yaw_mea_last_ = euler(2);
    yaw_mea_ = euler(2) + static_cast<float>(N_yaw_) * 2.0 * PI;
  }

  void StateEstCF::run() {
      gyroCorrection();
      attIntegration(); 
      heightEKF();
      if (odometry_called_)  posVelEKF();
      
      publishEstimation();
      calculateDifference();

      return;
  }
  
  void StateEstCF::gyroCorrection()  {

    integrator3(att_err_, att_err_integrated_, dt_, 10.0);
    integrator(yaw_err_, yaw_err_integrated_, dt_, 10.0);
    rate_corrected_ = rate_ - ka_ * att_err_ - kb_ * att_err_integrated_;    
    rate_corrected_(2) = rate_(2) - kyaw_ * yaw_err_ - kbyaw_ * yaw_err_integrated_;

    rate_corrected_ -= gyro_bias_;

    return;
  }

  void StateEstCF::attIntegration()  {

    Eigen::Matrix4f Fq;
    Eigen::Vector4f q;
    q << q_est0_.w(), q_est0_.x(), q_est0_.y(), q_est0_.z();

    Fq <<                0.0,  -rate_corrected_(0), -rate_corrected_(1),   -rate_(2),
          rate_corrected_(0),                  0.0,  rate_(2),   -rate_corrected_(1),
          rate_corrected_(1),  -rate_(2),                 0.0,    rate_corrected_(0),
          rate_(2),   rate_corrected_(1), -rate_corrected_(0),                   0.0;   //Jacobian for Quaternion

    Fq *= 0.5;

    auto q_dot = Fq * q * dt_;
    q_est0_.w() += q_dot(0);
    q_est0_.x() += q_dot(1);
    q_est0_.y() += q_dot(2);
    q_est0_.z() += q_dot(3);
    q_est0_.normalize();

    auto euler = quaternion2euler(q_est0_);
    
    roll_  = euler(0);
    pitch_ = euler(1);

    yaw_ += dt_ * (rate_corrected_(1) * std::sin(roll_) 
            + rate_corrected_(2) * std::cos(roll_)) / std::cos(pitch_) ;

    if (!use_vio_) {
      q_est_ = Eigen::AngleAxisf(yaw_mea_optitrack_, Eigen::Vector3f::UnitZ())
              * Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX());   
    }
    else {
      q_est_ = Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()); 
    }
    
    return;
  }

  void StateEstCF::posVelEKF() {

    Eigen::Vector4f vecq;
    vecq << Q_pos_, Q_pos_, Q_vel_, Q_vel_;
    Eigen::Matrix4f Q = vecq.asDiagonal();
    Eigen::Vector2f vecr;
    vecr << R_pos_, R_pos_;

    Eigen::Matrix2f R = vecr.asDiagonal(); 

    // assign variables
    Eigen::Vector4f x;
    Eigen::Vector2f z;
    Eigen::Quaternionf q_est;

    q_est = Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX());

    Eigen::Vector3f vel_rot = q_est * (rate_.cross(p_bc_));
    Eigen::Vector3f vel_dot_ = q_est* (acc_corrected_) + gvec_;

    //state update
    pos_est_.x() += (vel_est_.x() + vel_rot.x()) * dt_;
    pos_est_.y() += (vel_est_.y() + vel_rot.y()) * dt_;
    vel_est_.x() += vel_dot_.x() * dt_;
    vel_est_.y() += vel_dot_.y() * dt_;

    x << pos_est_.x(), pos_est_.y(), vel_est_.x(), vel_est_.y();
    z << pos_cg_.x(), pos_cg_.y();

    double timenow;
    if (delay_compensation_) timenow = ros::Time::now().toSec();
    else  timenow = time_vio_;

    if (!odometry_called_) time_vio_ = timenow;

    Eigen::Vector4f x_delay;
    x_delay << pos_est_.x() - (timenow - time_vio_) * vel_est_.x(), 
               pos_est_.y() - (timenow - time_vio_) * vel_est_.y(),
               vel_est_.x(),
               vel_est_.y();

    // Define Jacobian
    Eigen::Matrix4f F;
    F << 1.0, 0.0, dt_, 0.0,
         0.0, 1.0, 0.0, dt_,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0; 
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2,4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    //covariance update
    P_posvel_  = F * P_posvel_* F.transpose() + Q;   
    
    // // update posterior
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(4,2);
    Eigen::Matrix2f S = (H * P_posvel_*H.transpose() + R);    
    K = P_posvel_ * H.transpose() * S.inverse(); 

    Eigen::Vector2f y;

    if (delay_compensation_)   
      y = H * x_delay - z; 
    else
      y = H * x - z; 

    x -=  K * y; 
    P_posvel_ -=  K * H * P_posvel_;
        
    pos_est_.x() = x(0);
    pos_est_.y() = x(1);
    vel_est_.x() = x(2);
    vel_est_.y() = x(3);

    return;
  }

  void StateEstCF::heightEKF() {

    height_mea_ = range_ * cos(pitch_) * cos(roll_);
    float acc_z_cent = rate_(0)*rate_(2)*displacement_(0) 
                     + rate_(1)*rate_(2)*displacement_(1) 
                     + (- rate_(0)*rate_(0) - rate_(1)*rate_(1)) * displacement_(2);
    float acc_z_corr = acc_(2) - acc_z_cent - acc_bias_(2);
    float az = acc_z_corr  * cos(pitch_) * cos(roll_) - g_;
    float height_err, az_corr;

    height_err = height_mea_ - height_;

    F_ << 1.0, 0.0,
          dt_, 1.0; 
    Q_ << dt_ * dt_             , 0.5 * std::pow(dt_,3),
          0.5 * std::pow(dt_,3) , 0.25 * std::pow(dt_,4);
    Q_ = Q_ * q_;

    velz_   += az * dt_;
    height_ += velz_ * dt_;
  
    P_ = F_ * P_ * F_.transpose()  + Q_;
    
    float y = height_mea_ - height_;

    float S = P_(1,1);
    S += R_;
    auto K = P_ * H_.transpose() / S;
    P_ -= K * H_ * P_;

    velz_   += K(0) * y;
    height_ += K(1) * y;   
      
    return;
  }

  void StateEstCF::publishEstimation() {
    quad_msgs::QuadStateEstimate msg;

    msg.position.x = pos_est_(0);
    msg.position.y = pos_est_(1);
    msg.velocity.x = vel_est_(0);
    msg.velocity.y = vel_est_(1);      
    msg.position.z = height_;
    msg.velocity.z = velz_;

    msg.orientation.w = q_est_.w();
    msg.orientation.x = q_est_.x();
    msg.orientation.y = q_est_.y();
    msg.orientation.z = q_est_.z();

    msg.bodyrates.x = rate_(0);
    msg.bodyrates.y = rate_(1);
    msg.bodyrates.z = rate_(2);

    msg.header.stamp = publishTime_;
    msg.header.seq = pub_msg_seq_++;  

    state_est_pub_.publish(msg);
    return;
  }

  void StateEstCF::calculateDifference() {
    // compensate for displacement from imu to cg
    if(displacment_compensation_) {
      Eigen::Vector3f tmp;
      tmp = rate_.cross(rate_.cross(displacement_));

      acc_corrected_ = acc_ - tmp;
    }
    else  acc_corrected_ = acc_;
    
    // compensate for centripetal acceleration
    if(rotation_compensation_) {
      Eigen::Vector3f tmp;
      tmp = rotationCompensation();
      acc_corrected_ = acc_corrected_ - tmp;
    }
    acc_corrected_ -= acc_bias_;

    Eigen::Vector3f zB;
    zB = q_est_.inverse() * Eigen::Vector3f::UnitZ();

    att_err_ = zB.cross(acc_corrected_);
    
    // compensate for the latency of yaw estimate from VIO
    double timenow;
    if (delay_compensation_)  timenow = ros::Time::now().toSec();
    else timenow = time_vio_;

    if (!odometry_called_) time_vio_ = timenow;
    yaw_err_ = yaw_ - (timenow - time_vio_) * rate_(2) - yaw_mea_;

    return;
  }
  
  template <typename T>
  void limit(T& v, T down, T up) {
    v = (v >= up) ? up:v;
    v = (v <= down) ? down:v;

    return; 
  }

  void StateEstCF::integrator3(const Eigen::Vector3f& input, Eigen::Vector3f& output, const float dt, const float max) {
      output = dt * input + output;

      limit(output(0), -max, max);
      limit(output(1), -max, max);

      return;
  }

  void StateEstCF::integrator(const float& input, float& output, const float dt, const float max) {
      output = dt * input + output;

      limit(output, -max, max);

      return;
  }

  Eigen::Vector3f StateEstCF::rotationCompensation() {
      Eigen::Vector3f f = Eigen::Vector3f::Zero();

      float alpha = std::acos(std::cos(pitch_)*std::cos(roll_));

      if (std::abs(rate_(2)) >= 10.0) {
        f(0) =   - g_ * rate_(0) / rate_(2) * std::cos(alpha);
        f(1) =   - g_ * rate_(1) / rate_(2) * std::cos(alpha);
        f(2) =     g_ * std::tan(alpha) * std::sin(alpha);
      }

      return f;
  }   

  Eigen::Vector3f StateEstCF::quaternion2euler(Eigen::Quaternionf q) {  
    auto R = q.toRotationMatrix();

    Eigen::Vector3f euler;
    euler(0)   = std::atan(R(2,1)/R(2,2)); //roll
    euler(1)  = -std::asin(R(2,0)); // pitch
    euler(2)  = std::atan(R(1,0)/R(0,0)); // yaw

    if ((R(1,0) / R(0,0) >= 0) && (R(1,0) / std::cos(euler(1)) < 0))
         euler(2) -= PI;
    if ((R(1,0) / R(0,0) <= 0) && (R(1,0) / std::cos(euler(1)) > 0))
         euler(2) += PI;

    return euler;
  }  

} // namespace state_est_CF
