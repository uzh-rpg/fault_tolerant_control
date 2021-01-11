#pragma once
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mav_msgs/Actuators.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_msgs/ControlCommand.h"

namespace rotors_interface
{
    class RotorSInterface
    {
        public:
            RotorSInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
            RotorSInterface():
                RotorSInterface(ros::NodeHandle(), ros::NodeHandle("~")){};
            ~RotorSInterface();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher  rotors_desired_motor_speed_pub_;
            ros::Publisher  state_est_pub_;
            ros::Subscriber rotors_odometry_sub_;
            ros::Subscriber motor_command_sub_;
            
            double coeff1_, coeff2_, coeff3_;
            double rotor_thrust_coeff_;
            
            bool loadParams();
            void rotorsOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void ftcMotorCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg);
    };

} // namespace rotors_interface