#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "ftc_ctrl/ftc_ctrl.h"

namespace ftc {
  NDICtrl::NDICtrl
  (const ros::NodeHandle& nh, const ros::NodeHandle& pnh){
    nh_ = nh;
    pnh_ = pnh;

    // Fetch parameters
    ROS_INFO("[%s] Initializing...!", pnh_.getNamespace().c_str());
    if (!loadParams())  {
      ROS_ERROR("[%s] Could not load parameters.",
        pnh_.getNamespace().c_str());
      ros::shutdown();
      return;
    }
    ROS_INFO("[%s] Parameters loaded",
      ros::this_node::getName().c_str());    
    armed_out_.data = false;

    control_timer_    = nh_.createTimer(ros::Duration(1.0/ctrl_rate_), &NDICtrl::controlUpdateCallback, this);
    start_rotors_sub_ = nh_.subscribe("start_rotors", 1, &NDICtrl::startRotorsCallback, this);
    stop_rotors_sub_  = nh_.subscribe("stop_rotors", 1, &NDICtrl::stopRotorsCallback, this);
    state_est_sub_    = nh_.subscribe("state_est", 1, &NDICtrl::stateUpdateCallback, this);
    reference_sub_    = nh_.subscribe("reference_pos", 1, &NDICtrl::referenceUpdateCallback, this );
    yaw_rate_sub_     = nh_.subscribe("heading_design", 1, &NDICtrl::headingCallback, this );
    joy_sub_          = nh_.subscribe("joy", 1, &NDICtrl::joyCallback, this );

    motor_command_pub_ = nh_.advertise<quad_msgs::ControlCommand>("control_command", 1); 
    arm_pub_      = nh_.advertise<std_msgs::Bool>("control_active", 1);
    kill_pub_     = nh_.advertise<std_msgs::Empty>("emergency_kill", 1);

    yaw_rate_err_int_ = 0.0;
    motor_command_msg_.control_mode = 3;
    motor_command_msg_.mot_throttle = {0.0, 0.0, 0.0, 0.0};

    if (sqrt(nx_b_*nx_b_+ny_b_*ny_b_) >= 0.5) {
      ROS_ERROR("[%s] Wrong primary axis setting.",
       pnh_.getNamespace().c_str());
      ros::shutdown();
      return;
    }
    n_b_ << nx_b_, ny_b_, sqrt(1 - nx_b_*nx_b_ - ny_b_*ny_b_);

    Inertia_ << Ix_,  0.0,  0.0,
                0.0,  Iy_,  0.0,
                0.0,  0.0,  Iz_;

    pos_design_ << 0.0, 0.0, -1.0;

    time_fail_ = 0.0;
    time_traj_update_ = 0.0;

    heading_target_ = 0.0;
    // calculate control effective matrix G and it's inverse

    use_vio_ = true;
    calculateControlEffectiveness();
  
    ROS_INFO("[%s] Controller initialized", ros::this_node::getName().c_str());  
  }

  NDICtrl::~NDICtrl(){}

  void NDICtrl::calculateControlEffectiveness() {    
  
    Eigen::Matrix4d G;

    G <<   lever_arm_y_[0]  + cg_bias_[0],   lever_arm_y_[1] + cg_bias_[1],  lever_arm_y_[2] + cg_bias_[1],   lever_arm_y_[3] + cg_bias_[1],
           -lever_arm_x_[0] - cg_bias_[0],   -lever_arm_x_[1] - cg_bias_[0], -lever_arm_x_[2] - cg_bias_[0],  -lever_arm_x_[3] - cg_bias_[0],
           drag_coeff_[0],  drag_coeff_[1],  drag_coeff_[2],  drag_coeff_[3],
           1.0, 1.0, 1.0, 1.0;
    G_inv_ = Eigen::MatrixXd::Zero(4,4);

    //
    if (f_id_ > -1 && f_id_ < 4) {
      Eigen::MatrixXd G3_tmp(3,4);
      G3_tmp << G.row(0),
                G.row(1),
                G.row(3);
      Eigen::Matrix3d G3;

      size_t j=0;
      for (size_t i=0; i<4; i++) {
        if(i != f_id_) {
          G3.col(j) << G3_tmp.col(i);
          j++ ;
        }
      }
      Eigen::Matrix3d G3_inv = G3.inverse();
      
      Eigen::MatrixXd G_tmp = Eigen::MatrixXd::Zero(4,3);
      if(f_id_ > -1) {
        G_tmp.topRows(f_id_) = G3_inv.topRows(f_id_);
        if (f_id_ != 3) {
          G_tmp.bottomRows(3-f_id_) = G3_inv.bottomRows(3-f_id_);
        }        
      }  

      G_inv_.leftCols(2) = G_tmp.leftCols(2);
      G_inv_.rightCols(1) = G_tmp.rightCols(1);
    }
    else {
      G_inv_ = G.inverse(); 
    }
  }

  void NDICtrl::startRotorsCallback
  (const std_msgs::Empty::ConstPtr& msg)  {
    armed_out_.data = true;
  }

  void NDICtrl::stopRotorsCallback
  (const std_msgs::Empty::ConstPtr& msg)  {
    failure_mode_ = true;
    failIdCallback();
  }

  void NDICtrl::joyCallback(const sensor_msgs::JoyConstPtr msg)  {
    joy_msg_ = *msg;

    if(!joy_called_)
      joy_called_ = true;

    if (!manual_mode_)
      manual_mode_ = static_cast<bool>(joy_msg_.buttons[5]);
      
    if (!kill_mode_)
      kill_mode_ = static_cast<bool>(joy_msg_.buttons[4]);

    if (!failure_mode_) {
      failure_mode_ = static_cast<bool>(joy_msg_.buttons[0]);

      if (failure_mode_)
        failIdCallback();
    }

    if(!use_vio_) {
      use_vio_ =   static_cast<bool>(joy_msg_.buttons[3]);

      if (use_vio_) {
        pos_design_(0) = pos_design_current_position_.x();
        pos_design_(1) = pos_design_current_position_.y();
      }
    }
      
  }

  void NDICtrl::failIdCallback()  {

    f_id_ = failure_id_;
    double lever_arm = std::sqrt(lever_arm_x_[f_id_]*lever_arm_x_[f_id_]+lever_arm_y_[f_id_]*lever_arm_y_[f_id_]);
    nx_b_ = - n_primary_axis_ * lever_arm_x_[f_id_] / lever_arm;
    ny_b_ = - n_primary_axis_ * lever_arm_y_[f_id_] / lever_arm;
    
    time_fail_ = ros::Time::now().toNSec();

    calculateControlEffectiveness();
  }

  void NDICtrl::headingCallback
  (const std_msgs::Float64ConstPtr& msg) {
    heading_target_ = msg->data;
  }


  void NDICtrl::stateUpdateCallback(const quad_msgs::QuadStateEstimate::ConstPtr& msg)  {       
     
      if(estimation_received_ == false)
        estimation_received_ = true;
      
      state_.timestamp = msg->header.stamp;

      state_.position.x()     =   msg->position.x;     
      state_.position.y()     =   msg->position.y;     
      state_.position.z()     =   msg->position.z;     

      state_.velocity.x()     =   msg->velocity.x;     
      state_.velocity.y()     =   msg->velocity.y; 
      state_.velocity.z()     =   msg->velocity.z;         

      pos_design_current_position_.x() = msg->position.x;
      pos_design_current_position_.y() = msg->position.y;     

      state_.orientation.x()  =   msg->orientation.x;    
      state_.orientation.y()  =   msg->orientation.y;    
      state_.orientation.z()  =   msg->orientation.z;          
      state_.orientation.w()  =   msg->orientation.w;          

      state_.bodyrates.x()    =   msg->bodyrates.x;        
      state_.bodyrates.y()    =   msg->bodyrates.y;        
      state_.bodyrates.z()    =   msg->bodyrates.z;      
  }

  void NDICtrl::referenceUpdateCallback(const geometry_msgs::PointConstPtr& msg)  {
      
      position_at_update_ = pos_design_;

      pos_design_(0) = msg->x;
      pos_design_(1) = msg->y;
      pos_design_(2) = msg->z;

      referenceUpdated_ = true;
      time_traj_update_ = ros::Time::now().toSec();

      return;
  }

  void NDICtrl::controlUpdateCallback(const ros::TimerEvent&)  {
    
    if (armed_out_.data && estimation_received_)
        arm_pub_.publish(armed_out_); 
    else return;

    if (kill_mode_) {
      std_msgs::Empty emsg;
      kill_pub_.publish(emsg);
    }
    controller_outerloop();
    controller_innerloop();
    sendMotorCommand();

    return;
  }

  template <typename T>
  void limit(T& v, T down, T up)
  {
    v = (v >= up) ? up:v;
    v = (v <= down) ? down:v;
    return; 
  }

  void NDICtrl::sendMotorCommand()  {
    int throttle[4];

    for (int i=0; i<thrust_.size();++i) {
      limit(thrust_(i), 0.0, 8.0);
      throttle[i] = static_cast<int>((-coeff2_ + sqrt(coeff2_ * coeff2_ - 4.0 * coeff1_ * (coeff3_ - thrust_(i)))) / (2.0 * coeff1_));
      limit(throttle[i], kMinMotCom_DShot_, kMaxMotCom_DShot_);
      motor_command_msg_.mot_throttle[i] = throttle[i];
      motor_command_msg_.rotor_thrust[i] = thrust_(i);
    } 

    if (f_id_>=0 && f_id_ <=3) {
      throttle[f_id_] = 0;
      motor_command_msg_.mot_throttle[f_id_] = 0;
      motor_command_msg_.rotor_thrust[f_id_] = 0.0;
    }

    motor_command_pub_.publish(motor_command_msg_);

    return;
  }

  void NDICtrl::controller_outerloop() {


    // Set position control gains
    Eigen::DiagonalMatrix<double,3> Kp_pos;
    Eigen::DiagonalMatrix<double,3> Kd_pos;
    Eigen::DiagonalMatrix<double,3> Ki_pos;
    if (!failure_mode_) {
      Kp_pos.diagonal() << Kp_pos_vec_[0], Kp_pos_vec_[1], Kp_pos_vec_[2];
      Kd_pos.diagonal() << Kd_pos_vec_[0], Kd_pos_vec_[1], Kd_pos_vec_[2];
      Ki_pos.diagonal() << Ki_pos_vec_[0], Ki_pos_vec_[1], Ki_pos_vec_[2];
    } else {
      Kp_pos.diagonal() << Kp_pos_vec_fail_[0], Kp_pos_vec_fail_[1], Kp_pos_vec_fail_[2];
      Kd_pos.diagonal() << Kd_pos_vec_fail_[0], Kd_pos_vec_fail_[1], Kd_pos_vec_fail_[2];
      Ki_pos.diagonal() << Ki_pos_vec_fail_[0], Ki_pos_vec_fail_[1], Ki_pos_vec_fail_[2];
    }

    // set used position and velocity for control.
    Eigen::Vector3d position_used, velocity_used;
    if (!filter_pos_.initialized() || !filter_vel_.initialized()) {
      //@ hack: These are notch filter parameters for 100 Hz control frequency.
      Eigen::VectorXd num_notch = (Eigen::Matrix<double,5,1>() 
            <<    0.935526706991070, -3.708891622841157, 5.547024652212278, 
            -3.708891622841157,  0.935526706991070).finished();
      Eigen::VectorXd den_notch = (Eigen::Matrix<double,5,1>() 
                  << 1.000000000000000,  -3.832569470084841,   5.542863517942953, 
                   -3.585213775599692,   0.875214548253684).finished();

      filter_pos_.init(num_notch, den_notch, state_.position);
      filter_vel_.init(num_notch, den_notch, state_.velocity);
    }
    filter_pos_.update(pos_filtered_notch_, state_.position);
    filter_vel_.update(vel_filtered_notch_, state_.velocity);

    if (!failure_mode_) {
      position_used = state_.position;
      velocity_used = state_.velocity;
    }
    else {
      if (use_notch_filter_){
        position_used << pos_filtered_notch_(0), pos_filtered_notch_(1), pos_filtered_notch_(2);
        velocity_used << vel_filtered_notch_(0), vel_filtered_notch_(1), vel_filtered_notch_(2);        
      }
      else
      {
        position_used << state_.position(0), state_.position(1), state_.position(2);
        velocity_used << state_.velocity(0), state_.velocity(1), state_.velocity(2);
      }
    }

    // Sigmoid reference trajectory
    double time = ros::Time::now().toSec() - time_traj_update_;
    if(!referenceUpdated_)
      time = 0.0;

    if (!sigmoid_traj_ || (time<0)) {
      Eigen::Vector3d pos_err = (pos_design_ - position_used);
      integrator3(pos_err, pos_err_int_, 1.0/ctrl_rate_, max_pos_err_int_);
      a_des_ = Kp_pos * pos_err - Kd_pos * velocity_used + Ki_pos * pos_err_int_ - g_vect_;
    }
    else {
      Eigen::Vector3d pos_des, vel_des, acc_des;
      
      sigmoidTraj(pos_design_(0), position_at_update_(0), pos_des(0), vel_des(0), acc_des(0), time);
      sigmoidTraj(pos_design_(1), position_at_update_(1), pos_des(1), vel_des(1), acc_des(1), time);
  
      pos_des(2) = pos_design_(2);
      vel_des(2) = 0.0;
      acc_des(2) = 0.0;

      Eigen::Vector3d pos_err = (pos_des - position_used);      
      integrator3(pos_err, pos_err_int_, 1.0/ctrl_rate_, max_pos_err_int_);

      a_des_ = acc_des + Kd_pos * (vel_des - velocity_used) 
                + Kp_pos * pos_err  + Ki_pos * pos_err_int_ - g_vect_;            
    }

    // acceleration command hedging
    if(use_vio_) { 
      if (!failure_mode_) {
        limit(a_des_(0), -max_horz_acc_, max_horz_acc_);
        limit(a_des_(1), -max_horz_acc_, max_horz_acc_);
        limit(a_des_(2), g_ - max_vert_acc_, g_ + max_vert_acc_);
      }
      else {
        limit(a_des_(0), -max_horz_acc_fail_, max_horz_acc_fail_);
        limit(a_des_(1), -max_horz_acc_fail_, max_horz_acc_fail_);
        limit(a_des_(2), g_ - max_vert_acc_, g_ + max_vert_acc_);
      }

      if (manual_mode_) {
        a_des_(0) = joy_msg_.axes[4] * max_horz_acc_;
        a_des_(1) = joy_msg_.axes[3] * max_horz_acc_; 
        a_des_(2) = joy_msg_.axes[1] * max_vert_acc_ + g_;   
      }
    }
    else {
      if (joy_called_) {
        a_des_(0) = joy_msg_.axes[4] * max_horz_acc_;
        a_des_(1) = joy_msg_.axes[3] * max_horz_acc_;  
      }
      else {
        a_des_(0) = 0.0;
        a_des_(1) = 0.0;
      }
      if(manual_mode_) a_des_(2) = joy_msg_.axes[1] * max_vert_acc_ + g_;   
      else limit(a_des_(2), g_ - max_vert_acc_, g_ + max_vert_acc_);
    }
    return;
  };
  
  void NDICtrl::controller_innerloop()  { 

    // change nb after failure happens. Keep it unchanged at the 1st second.
    time_now_ = ros::Time::now().toNSec();
    if ((time_now_ - time_fail_) <= 1.0*1e9)
      n_b_ << 0.0, 0.0, 1.0;
    else
      n_b_ << nx_b_, ny_b_, sqrt(1 - nx_b_*nx_b_ - ny_b_*ny_b_);

    Eigen::Vector3d n_des_i  = a_des_ / a_des_.norm();
    Eigen::Vector3d n_des_b = state_.orientation.inverse() * n_des_i;
    
    Eigen::Vector3d z_body = state_.orientation.inverse() * Eigen::Vector3d::UnitZ();
    z_body = z_body/z_body.norm()  ;
    double theta = std::acos(((-g_vect_).dot(z_body))/g_);
    double theta_1 = 1.00; // deg2rad(60)
    double thrust_design(0.0);
    thrust_design = (m_*a_des_(2))/cos(std::min(theta,theta_1));

    // reduced attitude controller
    Eigen::Vector2d nu_out;
    Eigen::Vector3d nb_err = n_b_ - n_des_b;

    if(!referenceUpdated_) {
      // Reset all integral terms once take-off command is sent
      nb_err_int_ << 0.0, 0.0, 0.0;
      pos_err_int_ << 0.0, 0.0, 0.0;
      yaw_rate_err_int_ = 0.0;
    }
    integrator3(nb_err, nb_err_int_, 1.0/ctrl_rate_, max_nb_err_int_);
    motor_command_msg_.thrust = thrust_design;

    Eigen::Vector3d omega_dot_design(0.0,0.0,0.0);
    double kph, kdh, kih;  
    if (!failure_mode_) {
      kph = K_att_[0];
      kdh = K_att_[1];
      kih = K_att_[2];
    }
    else {
      kph = K_att_fail_[0];
      kdh = K_att_fail_[1];
      kih = K_att_fail_[2];
    }

    omega_dot_design(0) = (kph * (n_b_(1) - n_des_b(1)) 
          + kdh * ( n_des_b(0) * state_.bodyrates.z() -  n_des_b(2) * state_.bodyrates.x())
          + kih * nb_err_int_(1)
          - (n_des_b(2) * state_.bodyrates.y() - n_des_b(1) * state_.bodyrates.z()) * state_.bodyrates.z()) / ( n_des_b(2));

    omega_dot_design(1) = (kph * (n_b_(0) - n_des_b(0)) 
          + kdh * (-n_des_b(1) * state_.bodyrates.z() +  n_des_b(2) * state_.bodyrates.y())
          + kih * nb_err_int_(0)
          + (n_des_b(0) * state_.bodyrates.z() - n_des_b(2) * state_.bodyrates.x()) * state_.bodyrates.z()) / (-n_des_b(2));


    // In the nominal condition, switch on the yaw controller
    if (!failure_mode_) {
      double omega_z_design;
      if (use_vio_) {
        omega_z_design = getDesignedYawRate();
        if (manual_mode_)
            omega_z_design = joy_msg_.axes[0] * 5.0;        
      }
      else {
        if (joy_called_) omega_z_design = joy_msg_.axes[0] * 5.0;  
        else  omega_z_design = 0.0;        
      }
      double yaw_rate_err = omega_z_design - state_.bodyrates.z();
      integrator(yaw_rate_err, yaw_rate_err_int_, 1.0/ctrl_rate_, 30.0);
      omega_dot_design(2) =  K_yaw_[1]  * yaw_rate_err + K_yaw_[2] * yaw_rate_err_int_;
    }

    // Control allocation
    Eigen::Vector3d mu3;
    mu3 = Inertia_ * omega_dot_design + state_.bodyrates.cross(Inertia_ * state_.bodyrates); 
    Eigen::Vector4d mu;
    mu << mu3, thrust_design;

    thrust_ = G_inv_ * mu;

    return;
  };

  bool NDICtrl::loadParams() {
    bool check = true;

    check &= pnh_.param("ctrl_rate"  , ctrl_rate_,  200.0);
    check &= pnh_.param("failed_prop", failure_id_, 0);
    check &= pnh_.param("mass",       m_,          0.5); 
    check &= pnh_.param("n_primary_axis",  n_primary_axis_,  0.0);
    check &= pnh_.param("minMot",     kMinMotCom_DShot_, 100);
    check &= pnh_.param("maxMot",     kMaxMotCom_DShot_, 2000);
    check &= pnh_.param("maxVertAcc", max_vert_acc_, 3.0);
    check &= pnh_.param("maxHorzAcc", max_horz_acc_, 3.0);    
    check &= pnh_.param("maxHorzAccFail", max_horz_acc_fail_, 2.0);    
    check &= pnh_.param("maxNbErrInt", max_nb_err_int_, 30.0);
    check &= pnh_.param("maxPosErrInt", max_pos_err_int_, 30.0);
    check &= pnh_.param("mot_coeff1", coeff1_, 0.000008492875480);
    check &= pnh_.param("mot_coeff2", coeff2_, -0.002777454295627);
    check &= pnh_.param("mot_coeff3", coeff3_, 0.108400169428593);
    check &= pnh_.param("use_sigmoid_traj", sigmoid_traj_, false);
    check &= pnh_.param("use_notch_filter", use_notch_filter_, false);
    check &= pnh_.param("Ix",         Ix_,         1.45e-3);
    check &= pnh_.param("Iy",         Iy_,         1.26e-3);
    check &= pnh_.param("Iz",         Iz_,         2.52e-3);
    check &= pnh_.getParam("lever_arm_x",   lever_arm_x_);
    check &= pnh_.getParam("lever_arm_y",   lever_arm_y_);
    check &= pnh_.getParam("drag_coeff",    drag_coeff_);
    check &= pnh_.getParam("cg_bias",    cg_bias_);
    check &= pnh_.getParam("kp_pos", Kp_pos_vec_);
    check &= pnh_.getParam("kd_pos", Kd_pos_vec_);
    check &= pnh_.getParam("ki_pos", Ki_pos_vec_);
    check &= pnh_.getParam("k_att", K_att_);
    check &= pnh_.getParam("k_yaw", K_yaw_);
    check &= pnh_.getParam("kp_pos_fail", Kp_pos_vec_fail_);
    check &= pnh_.getParam("kd_pos_fail", Kd_pos_vec_fail_);
    check &= pnh_.getParam("ki_pos_fail", Ki_pos_vec_fail_);
    check &= pnh_.getParam("k_att_fail", K_att_fail_);

    return check; 
  }

  void NDICtrl::integrator3(const Eigen::Vector3d& input, Eigen::Vector3d& output, const double dt, const double max) {
      output = dt * input + output;

      limit(output(0), -max, max);
      limit(output(1), -max, max);

      return;
  }

  void NDICtrl::integrator(const double &input, double& output, const double dt, const double max) {
      output = dt * input + output;

      limit(output, -max, max);

      return;    
  }

  double NDICtrl::getDesignedYawRate() {
      Eigen::Vector3d Target_vec(cos(heading_target_),sin(heading_target_),0.0); 
      Eigen::Vector3d vecTmp = state_.orientation.inverse() * Target_vec;
      Eigen::Vector3d Xwbxy;
      Xwbxy << vecTmp(0), vecTmp(1), 0.0;
      Xwbxy /= Xwbxy.norm();
      Eigen::Vector3d rot_vec = Eigen::Vector3d::UnitX().cross(Xwbxy);
      rot_vec /= rot_vec.norm();

      return K_yaw_[0] * std::acos(Xwbxy(0)) * rot_vec(2);
  }


  void NDICtrl::sigmoidTraj(const double pos_end, const double pos_begin,
                           double& pos_des, double& vel_des, double& acc_des, double time)  {
    double a, b, c, tend;

    a = pos_end - pos_begin;

    tend = std::abs(a) / 0.2; // manual designed
    if (tend < 1.0)
      tend = 1.0;

    if (time > tend)
      time = 2.0*tend;

    c = 6.0 / tend;
    b = 3.0 / c;

    pos_des = pos_begin + (a*exp(-c*(b - time)))/(exp(-c*(b - time)) + 1);
    vel_des = (a*c*exp(-c*(b - time)))/(exp(-c*(b - time)) + 1) 
              - (a*c*exp(-2*c*(b - time)))/((exp(-c*(b - time)) + 1)*(exp(-c*(b - time)) + 1));
    acc_des = (a*c*c*exp(-c*(b - time)))/(exp(-c*(b - time)) + 1) 
              - (3*a*c*c*exp(-2*c*(b - time)))/((exp(-c*(b - time)) + 1)*(exp(-c*(b - time)) + 1)) 
              + (2*a*c*c*exp(-3*c*(b - time)))/(exp(-c*(b - time)) + 1)/(exp(-c*(b - time)) + 1)/(exp(-c*(b - time)) + 1);

    return;
  }

} //namespace ftc

