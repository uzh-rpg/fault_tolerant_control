#pragma once

#include "ros/ros.h"


#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "quad_msgs/ControlCommand.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "sensor_msgs/Joy.h"
// #include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <Eigen/Eigen>
#include "filter.h"

#define PI 3.14159265

namespace ftc
{
  struct QuadState
  {
    ros::Time timestamp;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d bodyrates;
  };

  class NDICtrl
  {
  public:

    NDICtrl(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    NDICtrl():
      NDICtrl(ros::NodeHandle(), ros::NodeHandle("~")){};
    ~NDICtrl();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber state_est_sub_;
    ros::Subscriber reference_sub_; 
    ros::Subscriber start_rotors_sub_;
    ros::Subscriber stop_rotors_sub_;
    ros::Subscriber yaw_rate_sub_;
    ros::Subscriber joy_sub_;
    ros::Publisher motor_command_pub_;
    ros::Publisher arm_pub_;
    ros::Publisher kill_pub_;
    ros::Timer control_timer_;
    std_msgs::Bool armed_out_;
    quad_msgs::ControlCommand motor_command_msg_;
    sensor_msgs::Joy joy_msg_;

    QuadState state_;

    int kMinMotCom_DShot_;
    int kMaxMotCom_DShot_;
    int f_id_=-1;     
    int failure_id_;
    
    double ctrl_rate_;
    double time_traj_update_;
    static constexpr double g_=9.81;
    Eigen::Vector3d g_vect_=Eigen::Vector3d(0.0,0.0, -g_);
    double nx_b_=0;     
    double ny_b_=0;
    double n_primary_axis_;  
    double max_vert_acc_;
    double max_pos_err_int_;
    double max_nb_err_int_;
    double max_horz_acc_;
    double max_horz_acc_fail_;

    Eigen::Vector3d position_at_update_;
    Eigen::Vector3d n_b_;
    Eigen::Vector3d nb_err_int_;
    Eigen::Vector3d pos_err_int_;   
    Eigen::Vector3d a_des_;
    Eigen::Matrix4d G_inv_;
    Eigen::Vector4d thrust_;
    Eigen::Vector3d pos_design_;
    Eigen::Vector3d pos_design_current_position_;
    Eigen::Matrix3d Inertia_;

    double yaw_rate_err_int_;
    double m_, Ix_, Iy_, Iz_;   
    double coeff1_, coeff2_, coeff3_;
    double time_now_;
    double time_fail_;
    double heading_target_;
    
    std::vector<double> Kp_pos_vec_, Kd_pos_vec_, Ki_pos_vec_, K_att_;
    std::vector<double> Kp_pos_vec_fail_, Kd_pos_vec_fail_, Ki_pos_vec_fail_, K_att_fail_;
    std::vector<double> K_yaw_;

    bool sigmoid_traj_;
    bool position_design_updated_=false;
    bool use_notch_filter_;
    bool estimation_received_=false;
    bool referenceUpdated_=false;
    bool use_vio_     = false;
    bool joy_called_  = false;
    bool manual_mode_ = false;
    bool kill_mode_   = false;
    bool failure_mode_= false;

    Eigen::Vector3d pos_filtered_notch_;
    Eigen::Vector3d vel_filtered_notch_;    

    filter::Filter filter_pos_;
    filter::Filter filter_vel_;

    std::vector<double> lever_arm_x_, lever_arm_y_, drag_coeff_;
    std::vector<double> cg_bias_;

    bool loadParams();
    void startRotorsCallback (const std_msgs::Empty::ConstPtr& msg);
    void stopRotorsCallback (const std_msgs::Empty::ConstPtr& msg);
    void controlUpdateCallback(const ros::TimerEvent&);
    void joyCallback(const sensor_msgs::JoyConstPtr msg);
    void stateUpdateCallback(const quad_msgs::QuadStateEstimate::ConstPtr& msg);
    void headingCallback(const std_msgs::Float64ConstPtr& msg);
    void referenceUpdateCallback(const geometry_msgs::PointConstPtr& msg);
    void calculateControlEffectiveness();
    void sendMotorCommand();
    void controller_outerloop();
    void controller_innerloop();
    double getDesignedYawRate();
    void failIdCallback();
    void integrator3(const Eigen::Vector3d& input, Eigen::Vector3d& output, const double dt, const double max);
    void integrator(const double &input, double& output, const double dt, const double max);
    void sigmoidTraj(const double pos_end, const double pos_begin,
                           double& pos_des, double& vel_des, double& acc_des, double time);


  };

} // namespace ftc
