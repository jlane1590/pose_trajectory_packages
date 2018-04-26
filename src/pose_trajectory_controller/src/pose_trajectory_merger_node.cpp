#include <ros/ros.h>
#include "pose_trajectory_merger/pose_trajectory_merger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_trajectory_merger");
  ros::NodeHandle nodeHandle("~");

  pose_trajectory_merger::PoseTrajectoryMerger <trajectory_interface::QuinticSplineSegment<double> > poseTrajectoryMerger(nodeHandle);

  ros::spin();
  return 0;
}
