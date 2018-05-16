#include <ros/ros.h>
#include <ryan_gesture_generation/dynamic_gesture_generators.h>
#include <pose_trajectory_controller/PoseTrajectory.h>
#include <pose_trajectory_controller/FollowPoseTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <sstream>

namespace ryan_gesture_generation
{
class TestGesture
{
public:
  TestGesture(char type, double duration, double intensity, double rate, double start_delay, bool invert)
    : nh_()
  {
    std::string action_topic;
    nh_.getParam("neck_controller/action_topic", action_topic);
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
    ryan_gesture_generation::RyanGestureGenerator ryanGestureGenerator(nh_);

    ROS_INFO("Using inputs:\nduration: %f, intensity: %f, rate: %f, start_delay: %f, invert: %s",
             duration, intensity, rate, start_delay, invert ? "true" : "false");

    switch(type)
    {
    case 'b':
      ROS_INFO("to construct a headbang gesture");
      goal.trajectory = ryanGestureGenerator.createHeadbangTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 'n':
      ROS_INFO("to construct a nod gesture");
      goal.trajectory = ryanGestureGenerator.createNodTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 'e':
      ROS_INFO("to construct an exaggerated nod gesture");
      goal.trajectory = ryanGestureGenerator.createExaggeratedNodTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 't':
      ROS_INFO("to construct a tilt gesture");
      goal.trajectory = ryanGestureGenerator.createTiltTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 's':
      ROS_INFO("to construct a surprise gesture");
      goal.trajectory = ryanGestureGenerator.createSurpriseTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 'h':
      ROS_INFO("to construct a hold surprise gesture");
      goal.trajectory = ryanGestureGenerator.createHoldSurpriseTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 'k':
      ROS_INFO("to construct a shake gesture");
      goal.trajectory = ryanGestureGenerator.createShakeTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    case 'c':
      ROS_INFO("to construct a circle gesture");
      goal.trajectory = ryanGestureGenerator.createCircleTrajectory(duration, intensity, rate, start_delay, invert);
      break;
    default:
      ROS_FATAL_STREAM_NAMED(
                "test_gesture",
                "Gesture type does not match a known gesture");
      exit(-1);
      break;
    }

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
  }

private:
  ros::NodeHandle nh_;
};

}

int main(int argc, char **argv)
{
  if(argc != 7)
  {
    ROS_ERROR("Usage is: \"name of gesture\", duration, intensity, rate, start_delay, invert");
    return 0;
  }
  std::stringstream ss;
  double gesture_inputs[5];
  for(int i = 2; i < 7; ++i)
  {
    ss << argv[i];
    //ss = new std::istringstream(argv[i]);
    if(!(ss >> gesture_inputs[i-2]))
    {
      ROS_ERROR("Invalid input %d: %s", i, argv[i]);
      return 0;
    }
    ss.clear();
  }

  ros::init(argc, argv, "dynamic_gesture_tester");
  //ros::NodeHandle nh;

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ryan_gesture_generation::TestGesture test_client((argv[1])[0], gesture_inputs[0], gesture_inputs[1],
      gesture_inputs[2], gesture_inputs[3], gesture_inputs[4]);
/*
  ryan_gesture_generation::RyanGestureGenerator ryanGestureGenerator(nh);
  pose_trajectory_controller::PoseTrajectory trajectory;

  std::cout << ros::Time::now() << std::endl;
  //trajectory = ryanGestureGenerator.createNodTrajectory(2.0,1.0,0.1,0.0,true);
  //trajectory = ryanGestureGenerator.createExaggeratedNodTrajectory(1.0,1.0,0.1,0.0,true);
  //trajectory = ryanGestureGenerator.createTiltTrajectory(3.0,1.0,0.0,0.0,false);
  trajectory = ryanGestureGenerator.createSurpriseTrajectory(2.0,1.0,1.0,0.0,false);
  //trajectory = ryanGestureGenerator.createHoldSurpriseTrajectory(2.0,1.0,0.0,0.0,false);
  //trajectory = ryanGestureGenerator.createShakeTrajectory(1.0,1.0,0.1,0.0,false);
  //trajectory = ryanGestureGenerator.createCircleTrajectory(1.0,1.0,1.0,0.0,true);
  std::cout << ros::Time::now() << std::endl;
  std::cout << "Trajectory:\n" << trajectory << std::endl;
*/

  ROS_INFO_STREAM_NAMED("dynamic_gesture_tester", "Shutting down.");
  ros::shutdown();

  return 0;
}
