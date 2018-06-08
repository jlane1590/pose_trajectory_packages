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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Generate a random trajectory to test the ros_control controller
*/

// ROS
#include <ros/ros.h>
#include <pose_trajectory_controller/FollowPoseTrajectoryAction.h>
#include <pose_trajectory_controller/PoseTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace ros_control_boilerplate
{
static const double SEC_PER_TRAJ_POINT = 0.5;  // time between points
static const std::size_t TRAJ_POINTS = 8;     // number of points to generate

class TestTrajectory
{
public:
  /**
   * \brief Constructor
   */
  TestTrajectory()
    : nh_private_("~")
  {
    std::string action_topic;
    nh_private_.getParam("action_topic", action_topic);
    if (action_topic.empty())
    {
      ROS_FATAL_STREAM_NAMED(
          "test_trajectory",
          "No follow pose trajectory action topic found on the parameter server");
      exit(-1);
    }
    ROS_INFO_STREAM_NAMED("test_trajectory", "Connecting to action " << action_topic);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<pose_trajectory_controller::FollowPoseTrajectoryAction> action_client(
        action_topic, true);

    ROS_INFO_NAMED("test_trajetory", "Waiting for action server to start.");
    // wait for the action server to start
    action_client.waitForServer();  // will wait for infinite time

    ROS_INFO_NAMED("test_trajetory", "Action server started, sending goal.");

    // send a goal to the action
    pose_trajectory_controller::FollowPoseTrajectoryGoal goal;
    goal.trajectory = createTrajectory();
    std::cout << "Trajectory:\n" << goal.trajectory << std::endl;
    action_client.sendGoal(goal);

    // Wait for the action to return
    double wait_extra_padding = 2;  // time to wait longer than trajectory itself
    bool finished_before_timeout = action_client.waitForResult(
        ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = action_client.getState();
      ROS_INFO_NAMED("test_trajetory", "Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO_NAMED("test_trajetory", "Action did not finish before the time out.");

    ROS_INFO_STREAM_NAMED("test_trajectory", "TestTrajectory Finished");
  }

  /**
   * \brief Create random trajectory
   */
  pose_trajectory_controller::PoseTrajectory createTrajectory()
  {
    std::vector<std::string> axis_names;
    double min_joint_value = -1.570796327;
    double max_joint_value = 1.570796327;

    // Get joint names
    nh_private_.getParam("neck_controller/axes", axis_names);
    if (axis_names.size() == 0)
    {
      ROS_FATAL_STREAM_NAMED(
          "init",
          "No axes found on parameter server for controller, did you load the proper yaml file?"
              << " Namespace: " << nh_private_.getNamespace() << "/neck_controller/axes");
      exit(-1);
    }

    nh_private_.getParam("min_joint_value", min_joint_value);
    nh_private_.getParam("max_joint_value", max_joint_value);
    ROS_DEBUG_STREAM_NAMED("test_trajectory", "Creating trajectory with joint values from "
                                                  << min_joint_value << " to " << max_joint_value);

    // Create header
    pose_trajectory_controller::PoseTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.axis_names = axis_names;

    // Create trajectory with x points
    trajectory.points.resize(TRAJ_POINTS);
    for (std::size_t i = 0; i < TRAJ_POINTS; ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
			trajectory.points[i].velocities.resize(axis_names.size());
      // for each joint
      for (std::size_t j = 0; j < axis_names.size(); ++j)
      {
        //trajectory.points[i].positions[j] = dRand(min_joint_value, max_joint_value);
        //if(j == 2)
          //trajectory.points[i].positions[j] = i%2==0 ? -0.5 : 0.5;
        //else
        trajectory.points[i].positions[j] = 0;
				trajectory.points[i].velocities[j] = 0;
        trajectory.points[i].time_from_start = ros::Duration(i * SEC_PER_TRAJ_POINT + 1);
      }
    }

    trajectory.points[0].positions[0] = 0.0;
    trajectory.points[0].velocities[0] = 0.0;
    trajectory.points[0].time_from_start = ros::Duration(1.0);

    trajectory.points[1].positions[0] = 0.15645;
    trajectory.points[1].velocities[0] = 0.140693;
    trajectory.points[1].time_from_start = ros::Duration(3.0);

    trajectory.points[2].positions[0] = 0.48217;
    trajectory.points[2].velocities[0] = 0.161184;
    trajectory.points[2].time_from_start = ros::Duration(3.5);

    trajectory.points[3].positions[0] = 0.06666;
    trajectory.points[3].velocities[0] = 0.175815;
    trajectory.points[3].time_from_start = ros::Duration(4.0);

    trajectory.points[4].positions[0] = 0.65701;
    trajectory.points[4].velocities[0] = 0.184587;
    trajectory.points[4].time_from_start = ros::Duration(4.5);

    trajectory.points[5].positions[0] = 0.250272;
    trajectory.points[5].velocities[0] = 0.1875;
    trajectory.points[5].time_from_start = ros::Duration(5.0);

    trajectory.points[6].positions[0] = 0.59353;
    trajectory.points[6].velocities[0] = 0.184553;
    trajectory.points[6].time_from_start = ros::Duration(5.5);

    trajectory.points[7].positions[0] = 1.0;
    trajectory.points[7].velocities[0] = 0.0;
    trajectory.points[7].time_from_start = ros::Duration(9.0);

/*
    trajectory.points[0].positions[0] = 0.0;
    trajectory.points[0].positions[1] = 0.5;
    trajectory.points[0].velocities[0] = 0.0;
    trajectory.points[0].velocities[1] = 0.0;

    trajectory.points[1].positions[0] = 0.5;
    trajectory.points[1].positions[1] = 0.0;
    trajectory.points[1].velocities[0] = 0.0;
    trajectory.points[1].velocities[1] = -0.7854;

    trajectory.points[2].positions[0] = 0.0;
    trajectory.points[2].positions[1] = -0.5;
    trajectory.points[2].velocities[0] = -0.7854;
    trajectory.points[2].velocities[1] = 0.0;

    trajectory.points[3].positions[0] = -0.5;
    trajectory.points[3].positions[1] = 0.0;
    trajectory.points[3].velocities[0] = 0.0;
    trajectory.points[3].velocities[1] = 0.7854;

    trajectory.points[4].positions[0] = 0.0;
    trajectory.points[4].positions[1] = 0.5;
    trajectory.points[4].velocities[0] = 0.7854;
    trajectory.points[4].velocities[1] = 0.0;

    trajectory.points[5].positions[0] = 0.0;
    trajectory.points[5].positions[1] = 0.0;
    trajectory.points[5].velocities[0] = 0.0;
    trajectory.points[5].velocities[1] = 0.0;
*/
/*
		trajectory.points.resize(3);
		for (std::size_t i = 0; i < 3; ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
			trajectory.points[i].velocities.resize(axis_names.size());
		}
		trajectory.points[0].positions[0] = -1.5;
		trajectory.points[0].velocities[0] = 0.0;
		trajectory.points[0].positions[1] = -1.5;
		trajectory.points[0].velocities[1] = 0.0;
		trajectory.points[0].time_from_start = ros::Duration(1);

		trajectory.points[1].positions[0] = 1.5;
		trajectory.points[1].velocities[0] = 0.0;
		trajectory.points[1].positions[1] = 1.5;
		trajectory.points[1].velocities[1] = 0.0;
		trajectory.points[1].time_from_start = ros::Duration(5);

		trajectory.points[2].positions[0] = -1.5;
		trajectory.points[2].velocities[0] = 0.0;
		trajectory.points[2].positions[1] = -1.5;
		trajectory.points[2].velocities[1] = 0.0;
		trajectory.points[2].time_from_start = ros::Duration(9);
*/

    return trajectory;
  }

  /** \brief Get random number */
  double dRand(double dMin, double dMax)
  {
    double d = (double)rand() / RAND_MAX;
    return dMin + d * (dMax - dMin);
  }

private:
  // A shared node handle
  ros::NodeHandle nh_private_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestTrajectory> TestTrajectoryPtr;
typedef boost::shared_ptr<const TestTrajectory> TestTrajectoryConstPtr;

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ROS_INFO_STREAM_NAMED("test_trajectory", "Starting TestTrajectory...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros_control_boilerplate::TestTrajectory server;

  ROS_INFO_STREAM_NAMED("test_trajectory", "Shutting down.");
  ros::shutdown();

  return 0;
}
