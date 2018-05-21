#include <ros/ros.h>
#include "ryan_trajectory_merger/ryan_trajectory_merger.h"
#include "pose_trajectory_controller/PoseTrajectory.h"
#include "ryan_gesture_generation/dynamic_gesture_generators.h"
#include <pose_trajectory_controller/FollowPoseTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ryan_trajectory_merger");
  ros::NodeHandle nodeHandle;

  //ros::Duration(3.0).sleep();
  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string action_topic;
  nodeHandle.getParam("neck_controller/action_topic", action_topic);
  if (action_topic.empty())
  {
    ROS_FATAL_STREAM_NAMED(
        "test_gesture",
        "No follow pose trajectory action topic found on the parameter server");
    exit(-1);
  }
  ROS_INFO_STREAM_NAMED("test_gesture", "Connecting to action " << action_topic);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<pose_trajectory_controller::FollowPoseTrajectoryAction> action_client(
      action_topic, true);

  ROS_INFO_NAMED("test_gesture", "Waiting for action server to start.");
  // wait for the action server to start
  action_client.waitForServer();  // will wait for infinite time

  ROS_INFO_NAMED("test_gesture", "Action server started, sending goal.");

  // send a goal to the action
  pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

  pose_trajectory_controller::PoseTrajectory merged;
  pose_trajectory_controller::PoseTrajectory posture;
  pose_trajectory_controller::PoseTrajectory gesture;

  ryan_gesture_generation::RyanGestureGenerator ryanGestureGenerator(nodeHandle);

  posture = ryanGestureGenerator.createShakeTrajectory(10.0, 1.0, 0.06, 0.0, false);
  gesture = ryanGestureGenerator.createSurpriseTrajectory(3.0, 1.0, 0.5, 3.0, false);

  int res = ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, gesture);

  goal.trajectory = merged;

  std::cout << "Trajectory:\n" << goal.trajectory << std::endl;
  action_client.sendGoal(goal);

  // Wait for the action to return
  double wait_extra_padding = 2;  // time to wait longer than trajectory itself
  bool finished_before_timeout = action_client.waitForResult(
      ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = action_client.getState();
    ROS_INFO_NAMED("test_gesture", "Action finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO_NAMED("test_gesture", "Action did not finish before the time out.");

  ROS_INFO_STREAM_NAMED("test_gesture", "TestGesture Finished");

  return 0;
}
