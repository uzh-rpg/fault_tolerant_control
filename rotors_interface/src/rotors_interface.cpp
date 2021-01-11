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


#include "rotors_interface/rotors_interface.hpp"
#include <iostream>
#include <stdlib.h>
#include <Eigen/Eigen>

namespace rotors_interface{

RotorSInterface::RotorSInterface  
(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)  {
    nh_ = nh;
    pnh_ = pnh;

    ROS_INFO("[%s] Initializing...!", pnh_.getNamespace().c_str());
    if (!loadParams())  {
      ROS_ERROR("[%s] Could not load parameters. Using default Params!",
        pnh_.getNamespace().c_str());
    }

    rotors_desired_motor_speed_pub_ = nh_.advertise<mav_msgs::Actuators>(
        "command/motor_speed", 1);
    state_est_pub_ = nh_.advertise<quad_msgs::QuadStateEstimate>(
        "state_est", 1);

    rotors_odometry_sub_ = nh_.subscribe(
        "ground_truth/odometry", 1, &RotorSInterface::rotorsOdometryCallback, this);  
    motor_command_sub_ = nh_.subscribe(
        "control_command", 1, &RotorSInterface::ftcMotorCommandCallback, this);  
}

RotorSInterface::~RotorSInterface(){}

void RotorSInterface::rotorsOdometryCallback(
    const nav_msgs::Odometry::ConstPtr& msg)
{

  quad_msgs::QuadStateEstimate msg_pub;

  msg_pub.header = msg->header;
  msg_pub.position.x    = msg->pose.pose.position.x;
  msg_pub.position.y    = msg->pose.pose.position.y;
  msg_pub.position.z    = msg->pose.pose.position.z;
  msg_pub.orientation.w = msg->pose.pose.orientation.w;  
  msg_pub.orientation.x = msg->pose.pose.orientation.x;
  msg_pub.orientation.y = msg->pose.pose.orientation.y;
  msg_pub.orientation.z = msg->pose.pose.orientation.z;

  Eigen::Vector3d Vb = (Eigen::Vector3d() 
                    << msg->twist.twist.linear.x, 
                        msg->twist.twist.linear.y, 
                        msg->twist.twist.linear.z
                        ).finished();
  Eigen::Quaterniond q;
  q.w() = msg->pose.pose.orientation.w;  
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d Vi = q * Vb;
  msg_pub.velocity.x    = Vi(0);
  msg_pub.velocity.y    = Vi(1);
  msg_pub.velocity.z    = Vi(2);
  msg_pub.bodyrates.x   = msg->twist.twist.angular.x;
  msg_pub.bodyrates.y   = msg->twist.twist.angular.y;
  msg_pub.bodyrates.z   = msg->twist.twist.angular.z;

  state_est_pub_.publish(msg_pub);

  return;
}

void RotorSInterface::ftcMotorCommandCallback(
    const quad_msgs::ControlCommand::ConstPtr& msg)
{
    mav_msgs::Actuators desired_motor_speed;
    
    for (int i=0; i<4; i++)
    {
        desired_motor_speed.angular_velocities.push_back(0.0);

        double rotorSpeed = msg->mot_throttle[i];
        if(rotorSpeed < 300) rotorSpeed = 0.0;
        else rotorSpeed = rotorSpeed * rotorSpeed * coeff1_
                    + rotorSpeed * coeff2_ + coeff3_;
               
        rotorSpeed /= rotor_thrust_coeff_;

        if (rotorSpeed < 0.0) rotorSpeed = 0.0;
                
        desired_motor_speed.angular_velocities[i] = std::sqrt(rotorSpeed);
    }
    rotors_desired_motor_speed_pub_.publish(desired_motor_speed);
    
    return;
}

bool RotorSInterface::loadParams() 
{
    bool check = true;
    check &= pnh_.param("mot_coeff1", coeff1_, 0.000008492875480);
    check &= pnh_.param("mot_coeff2", coeff2_, -0.002777454295627);
    check &= pnh_.param("mot_coeff3", coeff3_, 0.108400169428593);
    check &= pnh_.param("rotor_thrust_coeff", rotor_thrust_coeff_, 8.54858e-06);

    return check; 
}

}//namespace rotors_interface