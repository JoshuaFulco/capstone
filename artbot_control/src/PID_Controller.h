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
 Code based on joint_position_controller.h, authored by:
 Author: Vijay Pradeep
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
*/

#ifndef ARTBOT_CONTROL__PID_CONTROLLER_H
#define ARTBOT_CONTROL__PID_CONTROLLER_H

/**
   @class artbot_control::PID_Controller
   @brief pid_controller
   This class controls joint position using a pid loop.
   @section ROS ROS interface
   @param type Must be "artbot_control::PID_Control"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.
   Subscribes to:
   - @b command (std_msgs::Float64) : The joint position to achieve.
   Publishes:
   - @b state (control_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.
*/

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
//#include <urdf/model.h>
//#include <control_toolbox/pid.h>
//#include <boost/scoped_ptr.hpp>
//#include <boost/thread/condition.hpp>
//#include <realtime_tools/realtime_publisher.h>
//#include <control_msgs/JointControllerState.h>
//#include <realtime_tools/realtime_buffer.h>

namespace artbot_control{

class PID_Controller: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
  };

  struct Gains 
  {
    double p_gain_;  /**< Proportional gain. */
    double i_gain_;  /**< Integral gain. */
    double d_gain_;  /**< Derivative gain. */
//  double i_max_;   /**< Maximum allowable integral term. */
//  double i_min_;   /**< Minimum allowable integral term. */
  };

  PID_Controller();
  ~PID_Controller();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */  
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double pos_target);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(double pos_target, double vel_target);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);
  
  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the PID parameters
   */

  hardware_interface::JointHandle joint_;
//  boost::shared_ptr<const urdf::Joint> joint_urdf_;
//  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

  Gains gain_struct_;

private:
  int loop_count_;

  ros::Subscriber sub_command_;

//  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_; /**< Position error. */
  double i_error_; /**< Integral of position error. */
  double d_error_; /**< Derivative of position error. */

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);

//  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */

/*  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;
*/

  /**
   * \brief Check that the command is within the hard limits of the joint. Checks for joint
   *        type first. Sets command to limit if out of bounds.
   * \param command - the input to test
   */
//  void enforceJointLimits(double &command);

}; // class

} // namespace

#endif
