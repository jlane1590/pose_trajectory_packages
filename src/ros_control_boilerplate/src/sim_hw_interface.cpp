/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#include <ros_control_boilerplate/sim_hw_interface.h>

// ROS parameter loading
//#include <rosparam_shortcuts/rosparam_shortcuts.h>

#define sim 1

namespace ros_control_boilerplate
{
SimHWInterface::SimHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : GenericHWInterface(nh, urdf_model)
  , name_("sim_hw_interface")
{
  // Load rosparams
/*  ros::NodeHandle rpnh(nh_, "hardware_interface");
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "sim_control_mode", sim_control_mode_);
  if (error)
  {
    ROS_WARN_STREAM_NAMED(name_, "SimHWInterface now requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   sim_control_mode: 0 # 0: position, 1: velocity");
  }
  rosparam_shortcuts::shutdownIfError(name_, error);
*/
}

void SimHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // Resize vectors
  joint_position_prev_.resize(num_joints_, 0.0);
#if sim==1
  //setup polstro serial port
  std::string portName = "/dev/ttyACM0";
  unsigned int baudRate = 57600;
  maestro = Polstro::SerialInterface::createSerialInterface(portName, baudRate);
  if(maestro == NULL)
    ROS_INFO("Failed to create maestro pointer");
  //initialize servos to center position
  std::vector<double> positions;
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    positions.push_back(0.0);
    joint_position_[joint_id] = 0.0;
  }
  double *posptr = &positions[0];
  if(!maestro->setMultiplePos(num_joints_, channelBlockStart, posptr, true))
    ROS_DEBUG("Cannot write servo positions");
#endif
  ROS_INFO_NAMED(name_, "SimHWInterface Ready.");
}

void SimHWInterface::read(ros::Duration &elapsed_time)
{
  // No need to read since our write() command populates our state for us
#if sim==0
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    unsigned short raw_pos = 0;
//    if(!maestro->getPosition(channelBlockStart + joint_id, raw_pos))
//      ROS_WARN("Cannot get joint%d position",joint_id);
    //currently joint position is in 1/4usecs from servo
    //joint_position_[joint_id] = raw_pos;
    ROS_INFO("joint%d command: [%d]",joint_id, joint_position_command_[joint_id]);
    joint_position_[joint_id] = joint_position_command_[joint_id];

    if (elapsed_time.toSec() > 0)
    {
      joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
    }
    else
      joint_velocity_[joint_id] = 0;

    // Save last position
    joint_position_prev_[joint_id] = joint_position_[joint_id];
  }
#endif
}

void SimHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);
#if sim==1
  std::vector<double> targets;
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    targets.push_back(joint_position_command_[joint_id]);
    //ROS_INFO("command: [%f]", joint_position_command_[joint_id]);
    //ROS_WARN_COND(joint_position_command_[joint_id] > 1.5, "Detected PWM Spike: %f", joint_position_command_[joint_id]);
    joint_position_[joint_id] = joint_position_command_[joint_id];

    // Calculate velocity based on change in positions
    if (elapsed_time.toSec() > 0)
    {
      joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
    }
    else
      joint_velocity_[joint_id] = 0;

    // Save last position
    joint_position_prev_[joint_id] = joint_position_[joint_id];
  }
  double *tarptr = &targets[0];
  if(!maestro->setMultiplePos(num_joints_, channelBlockStart, tarptr, true))
    ROS_DEBUG("Cannot write servo positions");
#endif

  // NOTE: the following is a "simuation" example so that this boilerplate can be run without being
  // connected to hardware
  // When converting to your robot, remove the built-in PID loop and instead let the higher leverl
  // ros_control controllers take
  // care of PID loops for you. This P-controller is only intended to mimic the delay in real
  // hardware, somewhat like a simualator
#if sim==0
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    positionControlSimulation(elapsed_time, joint_id);
    /*switch (sim_control_mode_)
    {
      case 0:  // hardware_interface::MODE_POSITION:
        positionControlSimulation(elapsed_time, joint_id);
        break;

      case 1:  // hardware_interface::MODE_VELOCITY:

        // // Move all the states to the commanded set points slowly
        // joint_position_[joint_id] += joint_velocity_[joint_id] * elapsed_time.toSec();

        // v_error_ = joint_velocity_command_[joint_id] - joint_velocity_[joint_id];

        // // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
        // joint_velocity_[joint_id] += v_error_ * VELOCITY_STEP_FACTOR;

        // Naive
        joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
        joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec();

        break;

      case 2:  // hardware_interface::MODE_EFFORT:
        ROS_ERROR_STREAM_NAMED(name_, "Effort not implemented yet");
        break;
    }*/
  }
#endif

}

void SimHWInterface::enforceLimits(ros::Duration &period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

void SimHWInterface::positionControlSimulation(ros::Duration &elapsed_time, const std::size_t joint_id)
{
  const double max_delta_pos = joint_velocity_limits_[joint_id] * elapsed_time.toSec();

  // Move all the states to the commanded set points at max velocity
  p_error_ = joint_position_command_[joint_id] - joint_position_[joint_id];

  const double delta_pos = std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
  joint_position_[joint_id] += delta_pos;

  // Bypass max velocity p controller:
  //joint_position_[joint_id] = joint_position_command_[joint_id];

  // Calculate velocity based on change in positions
  if (elapsed_time.toSec() > 0)
  {
    joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
  }
  else
    joint_velocity_[joint_id] = 0;

  // Save last position
  joint_position_prev_[joint_id] = joint_position_[joint_id];
}


}  // namespace
