
namespace pose_trajectory_merger {

template <class SegmentImpl>
PoseTrajectoryMerger<SegmentImpl>::PoseTrajectoryMerger(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle), dur_epsilon_(0.1)
{
  serviceServer_ = nodeHandle_.advertiseService("merge_trajectories",
  &PoseTrajectoryMerger::trajectoryMergerService, this);
}

template <class SegmentImpl>
PoseTrajectoryMerger<SegmentImpl>::~PoseTrajectoryMerger()
{
}

template <class SegmentImpl>
bool PoseTrajectoryMerger<SegmentImpl>::trajectoryMergerService(pose_trajectory_controller::MergeTwoPoseTraj::Request& request,
                             pose_trajectory_controller::MergeTwoPoseTraj::Response& response)
{
  pose_trajectory_controller::PoseTrajectory resultTrajectory;
  //check the validity of the request trajectories

  //handle empty input trajectories
  if(request.traj1.points.empty())
  {
    if(request.traj2.points.empty())
    {
      //both input trajectories are empty, nothing to be done, respond with empty trajectory
      ROS_INFO("input trajectories are empty, returning empty trajectory");
      response.mergedTraj = resultTrajectory;
      response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::EMPTY_TRAJECTORIES;
      return true;
    }
    //traj1 is empty so return traj2
    response.mergedTraj = request.traj2;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::SUCCESSFUL;
    return true;
  }
  else if(request.traj2.points.empty())
  {
    //traj2 is empty but traj1 is not
    response.mergedTraj = request.traj1;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::SUCCESSFUL;
    return true;
  }
  //Otherwise if the trajectories are not empty, they should consist of at least 2 waypoints to create a valid segment
  if(request.traj1.points.size() < 2 || request.traj2.points.size() < 2)
  {
    ROS_ERROR("Input trajectories must contain at least 2 waypoints.");
    response.mergedTraj = resultTrajectory;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::INVALID_TRAJECTORIES;
    return true;
  }

  // Non strictly-monotonic waypoints
  if (!isTimeStrictlyIncreasing(request.traj1) || !isTimeStrictlyIncreasing(request.traj2))
  {
    ROS_ERROR("Input trajectory contains waypoints that are not strictly increasing in time.");
    response.mergedTraj = resultTrajectory;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::INVALID_TRAJECTORIES;
    return true;
  }
  // Non matching controlled axes
  const std::vector<std::string> axis_names = request.traj1.axis_names;
  // Mapping vector contains the map between the traj1 axis order and traj2 axis order
  // The vector is empty if the axes don't match
  using pose_trajectory_controller::internal::mapping;

  std::vector<unsigned int> mapping_vector = mapping(request.traj2.axis_names,axis_names);
  //make a mapping vector for each trajectory
  if (mapping_vector.empty())
  {
    ROS_ERROR("Cannot merge given trajectories. The controlled axes do not match.");
    response.mergedTraj = resultTrajectory;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::INVALID_TRAJECTORIES;
    return true;
  }

  //Check that the sizes of the trajectory members are consistent
  if(!isValid(request.traj1) || !isValid(request.traj2))
  {
    ROS_ERROR("Input trajectory members are not of consistent size.");
    response.mergedTraj = resultTrajectory;
    response.error_code = pose_trajectory_controller::MergeTwoPoseTraj::Response::INVALID_TRAJECTORIES;
    return true;
  }

  //At this point we have two valid trajectories to merge

  //the start time of the merged trajectory will be the earlier start time of the input trajectories
  resultTrajectory.header.stamp = (request.traj1.header.stamp < request.traj2.header.stamp ?
                                     request.traj1.header.stamp : request.traj2.header.stamp);

  std::vector<ros::Duration> resultWayTimes;

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt1_it = request.traj1.points.begin();
  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt2_it = request.traj2.points.begin();

  while((std::distance(pt1_it, request.traj1.points.end()) >= 1) && (std::distance(pt2_it, request.traj2.points.end()) >= 1))
  {
    ros::Duration pt1TrueTime = (request.traj1.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start;
    ros::Duration pt2TrueTime = (request.traj2.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start;

    if((pt1TrueTime - pt2TrueTime).toSec() < -dur_epsilon_)
    {
      resultWayTimes.push_back(pt1TrueTime);
      pt1_it++;
    }
    else if((pt1TrueTime - pt2TrueTime).toSec() > dur_epsilon_)
    {
      resultWayTimes.push_back(pt2TrueTime);
      pt2_it++;
    }
    else
    {
      resultWayTimes.push_back(pt1TrueTime);
      pt1_it++;
      pt2_it++;
    }
  }

  if(std::distance(pt1_it, request.traj1.points.end()) < 1)
  {
    ROS_INFO("Adding remainder of traj2");
    for(; pt2_it != request.traj2.points.end(); ++pt2_it)
    {
      resultWayTimes.push_back((request.traj2.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start);
    }
  }
  else if(std::distance(pt2_it, request.traj2.points.end()) < 1)
  {
    ROS_INFO("Adding remainder of traj1");
    for(; pt1_it != request.traj1.points.end(); ++pt1_it)
    {
      resultWayTimes.push_back((request.traj1.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start);
    }
  }

  std::stringstream sstr;
  sstr << "Merged Trajectory Way Point time_from_starts: ";
  for(int i = 0; i < resultWayTimes.size(); ++i)
  {
    sstr << resultWayTimes[i].toSec() << " ";
  }
  sstr << std::endl;

  ROS_INFO_STREAM(sstr.str());

  resultTrajectory.axis_names = axis_names;
  resultTrajectory.points.resize(resultWayTimes.size());

  typename Segment::State segment_start_state = typename Segment::State(1);
  typename Segment::State segment_end_state = typename Segment::State(1);

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator msg_pt_it;

  //for each axis in the trajectory
  for(unsigned int axis_it=0; axis_it < axis_names.size(); axis_it++)
  {//for each way point time in the result
    for(std::vector<ros::Duration>::const_iterator point_it = resultWayTimes.begin(); point_it != resultWayTimes.end(); ++point_it)
    {//for each waypoint in the trajectory
      for(msg_pt_it = request.traj1.points.begin(); msg_pt_it != request.traj1.points.end(); ++msg_pt_it)
      {
        //check the time_from_start of each point in the input trajectory for the given axis,
        //the first point after the current result way point is the end of the segment to sample
        if(msg_pt_it->time_from_start > *point_it)
        {
          break;
        }
      }
      //the resulting way point occurs before the start of this trajectory
      if(msg_pt_it == request.traj1.points.begin())
      {
        //segment_start_state =
      }
    }
  }
/*
  //sample each trajectory over all axes for each waypoint time to determine the merged state
  for(std::vector<pose_trajectory_controller::PoseTrajectory::ConstPtr>::const_iterator traj_it = inputTrajectoryPtrs.begin();
      traj_it != inputTrajectoryPtrs.end(); ++traj_it) //for all input trajectories
  {

  }
*/


  response.mergedTraj = resultTrajectory;

  return true;
}

}
