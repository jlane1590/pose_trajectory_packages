#ifndef POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_H
#define POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_H

#include <pose_trajectory_controller/init_pose_trajectory.h>
//#include <pose_trajectory_controller/pose_trajectory_segment.h>

#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>

#include <ros/ros.h>
#include <pose_trajectory_controller/MergeTwoPoseTraj.h>

#include <iterator>
#include <sstream>

namespace pose_trajectory_merger {

template <class SegmentImpl>
class PoseTrajectoryMerger
{
public:
  PoseTrajectoryMerger(ros::NodeHandle& nodeHandle);

  virtual ~PoseTrajectoryMerger();

private:
  bool trajectoryMergerService(pose_trajectory_controller::MergeTwoPoseTraj::Request& request,
                               pose_trajectory_controller::MergeTwoPoseTraj::Response& response);

  bool sortAscendingDuration(const ros::Duration first, const ros::Duration second) {return first < second; }

  // ROS node handle
  ros::NodeHandle& nodeHandle_;

  // ROS service server
  ros::ServiceServer serviceServer_;

  double dur_epsilon_;

  typedef pose_trajectory_controller::PoseTrajectorySegment<SegmentImpl> Segment;

  Segment current_pose_segment_;

};

}

#include <pose_trajectory_merger/pose_trajectory_merger_impl.h>

#endif // POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_H
