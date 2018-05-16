#include <ros/ros.h>
#include "ryan_trajectory_merger/ryan_trajectory_merger.h"
#include "pose_trajectory_controller/PoseTrajectory.h"
#include "ryan_gesture_generation/dynamic_gesture_generators.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ryan_trajectory_merger");
  ros::NodeHandle nodeHandle;

  pose_trajectory_controller::PoseTrajectory merged;
  pose_trajectory_controller::PoseTrajectory posture;
  pose_trajectory_controller::PoseTrajectory gesture;

  ryan_gesture_generation::RyanGestureGenerator ryanGestureGenerator(nodeHandle);

  posture = ryanGestureGenerator.createShakeTrajectory(5.0,1.0,0.1,0.0,false);
  gesture = ryanGestureGenerator.createNodTrajectory(3.0, 0.5, 0.3, 1.0, false);

  int res = ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, gesture);

  std::cout << "Trajectory:\n" << merged << std::endl;

  return 0;
}
