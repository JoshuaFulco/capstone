/*
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Code based on joint_position_controller.cpp, authored by:
 Author: Vijay Pradeep
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
*/

#include "PID_Controller.h"
#include <pluginlib/class_list_macros.h>

namespace artbot_control{

PID_Controller::PID_Controller() //constructor
  : loop_count_(0) //initialize counter via Member initializer list
{}

PID_Controller::~PID_Controller() //destructor
{
  sub_command_.shutdown(); //shutdown node subscribed to joint command messages
}

  bool PID_Controller::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    //Load PID controller (if a separate class has the logic)

    //Start controller state publisher (if publishing control state)

    // Start command subscriber			topic, queue length, callback function pointer, callback object instance
    sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &PID_Controller::setCommandCB, this);

    // get the joint object (from the hardware interface) to use in the realtime loop
    joint_ = robot->getHandle(my_joint);  // throws on failure

    // Initialize PID gains
    gain_struct_.p_gain_ = 30.0;
    gain_struct_.i_gain_ = 0.0;
    gain_struct_.d_gain_ = 10.0;

    command_struct_.position_ = 0.0;
    command_struct_.velocity_ = 0.0;
  
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;

    return true;
  }

  void PID_Controller::update(const ros::Time& time, const ros::Duration& period)
  {

//  command_struct_ = *(command_.readFromRT());

// Need to get command_struct_ values from sub_command_
  double command_position = command_struct_.position_;
  double command_velocity = command_struct_.velocity_;
  
  double p_gain = gain_struct_.p_gain_;
  double i_gain = gain_struct_.i_gain_;
  double d_gain = gain_struct_.d_gain_;

  double commanded_effort;

  double current_position = joint_.getPosition();
  double current_velocity = joint_.getVelocity();

  double p_term, i_term, d_term;

/*
  // Compute position error
   angles::shortest_angular_distance_with_limits(
      current_position,
      command_position,
      joint_urdf_->limits->lower,
      joint_urdf_->limits->upper,
      error);
*/

  // Compute the current position error, integral sum, P and I command terms
  p_error_ = command_position - current_position;
  
  p_term = p_gain * p_error_;

  i_error_ += period.toSec() * p_error_; // add to running integral summation
  i_term = i_gain * i_error_;

  if (command_struct_.has_velocity_)
  {
    // Compute velocity error if a non-zero velocity command was given
    d_error_ = command_velocity - current_velocity;

    // Compute the commanded effort using PD gains
    d_term = d_error_*d_gain;
  }
  else
  {
    // Set the PID error and compute the command output
    d_term = -1.0*current_velocity*d_gain;
  }

  commanded_effort = p_term + i_term + d_term;

  joint_.setCommand(commanded_effort);

  }

void PID_Controller::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

// Set the joint position command
void PID_Controller::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
//  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void PID_Controller::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

//  command_.writeFromNonRT(command_struct_);
}

/*
  if (loop_count_ % 10 == 0)
  {
	//publish state
  }
  loop_count_++;
  }
*/

  void PID_Controller::starting(const ros::Time& time) {
  double pos_command = joint_.getPosition();
  }

}//namespace
PLUGINLIB_EXPORT_CLASS(artbot_control::PID_Controller, controller_interface::ControllerBase);
