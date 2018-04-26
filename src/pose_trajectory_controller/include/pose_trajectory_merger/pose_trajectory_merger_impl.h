#ifndef POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_IMPL_H
#define POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_IMPL_H

//#include <pose_trajectory_merger/pose_trajectory_merger.h>

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

  std::vector<ros::Duration> result_way_times;

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt1_it = request.traj1.points.begin();
  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt2_it = request.traj2.points.begin();

  while((std::distance(pt1_it, request.traj1.points.end()) >= 1) && (std::distance(pt2_it, request.traj2.points.end()) >= 1))
  {
    ros::Duration pt1RelativeTime = (request.traj1.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start;
    ros::Duration pt2RelativeTime = (request.traj2.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start;

    if((pt1RelativeTime - pt2RelativeTime).toSec() < -dur_epsilon_)
    {
      result_way_times.push_back(pt1RelativeTime);
      pt1_it++;
    }
    else if((pt1RelativeTime - pt2RelativeTime).toSec() > dur_epsilon_)
    {
      result_way_times.push_back(pt2RelativeTime);
      pt2_it++;
    }
    else
    {
      result_way_times.push_back(pt1RelativeTime);
      pt1_it++;
      pt2_it++;
    }
  }

  if(std::distance(pt1_it, request.traj1.points.end()) < 1)
  {
    //ROS_INFO("Adding remainder of traj2");
    for(; pt2_it != request.traj2.points.end(); ++pt2_it)
    {
      result_way_times.push_back((request.traj2.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start);
    }
  }
  else if(std::distance(pt2_it, request.traj2.points.end()) < 1)
  {
    //ROS_INFO("Adding remainder of traj1");
    for(; pt1_it != request.traj1.points.end(); ++pt1_it)
    {
      result_way_times.push_back((request.traj1.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start);
    }
  }

  std::stringstream sstr;
  sstr << "Merged Trajectory Way Point time_from_starts: ";
  for(int i = 0; i < result_way_times.size(); ++i)
  {
    sstr << result_way_times[i].toSec() << " ";
  }
  sstr << std::endl;

  ROS_INFO_STREAM(sstr.str());

  //ROS_INFO("wall time = %f \n traj start time = %f \n", ros::Time::now().toSec(), resultTrajectory.header.stamp.toSec());

  resultTrajectory.axis_names = axis_names;
  resultTrajectory.points.resize(result_way_times.size());

  for(int i = 0; i < result_way_times.size(); ++i)
  {
    resultTrajectory.points[i].time_from_start = result_way_times[i];
  }

  std::vector<pose_trajectory_controller::PoseTrajectory*> request_trajectories;
  request_trajectories.push_back(&(request.traj1));
  request_trajectories.push_back(&(request.traj2));

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator traj_point_it, prev_traj_point_it;

for(std::vector<pose_trajectory_controller::PoseTrajectory*>::iterator traj_it = request_trajectories.begin();
    traj_it != request_trajectories.end(); ++traj_it)
{
  //for each way point time in the result
  for(std::vector<ros::Duration>::iterator time_it = result_way_times.begin(); time_it != result_way_times.end(); ++time_it)
  {//for each waypoint in the trajectory
    for(traj_point_it = (*traj_it)->points.begin(); traj_point_it != (*traj_it)->points.end(); ++traj_point_it)
    {
      //check the time_from_start of each point in the input trajectory,
      //the first point after the current result way point is the end of the segment to sample
      ros::Duration relative_time = ((*traj_it)->header.stamp - resultTrajectory.header.stamp) + traj_point_it->time_from_start;
      if(relative_time > *time_it)
      {
        break;
      }
    }
    //the previous requested time occurred during a different segment, so update the segment to sample
    if(traj_point_it != prev_traj_point_it)
    {
      //the resulting way point occurs before the start of this trajectory,
      //so sample a segment from the start point to the start point
      if(traj_point_it == (*traj_it)->points.begin())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*traj_point_it);

        //create a pose trajectory segment going to and from the starting point
        double end_time = ((*traj_it)->header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment_.init(end_time, segment_state, end_time, segment_state);
      }
      //the resulting way point occurs after the end of this trajectory,
      //so sample a segment from the end point to the end point
      else if(traj_point_it == (*traj_it)->points.end())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to and from the starting point
        double end_time = ((*traj_it)->header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();

        current_pose_segment_.init(end_time, segment_state, end_time, segment_state);
      }
      //otherwise the resulting way point occurs somewhere along the trajectory,
      //so sample a segment to this point from the previous point
      else
      {
        //create a segment state out of this pose point
        typename Segment::State segment_end_state(*traj_point_it);
        //create a segment state out of the previous pose point
        typename Segment::State segment_start_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to the end state from the start state
        double start_time = ((*traj_it)->header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();
        double end_time = ((*traj_it)->header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment_.init(start_time, segment_start_state, end_time, segment_end_state);
      }
      //update the previous trajectory point iterator
      prev_traj_point_it = traj_point_it;
    }
      //sample the trajectory segment at the result trajectory way point time
      typename Segment::State way_point_state = typename Segment::State(axis_names.size());
      current_pose_segment_.sample(time_it->toSec(), way_point_state);

      int result_pt_index = std::distance(result_way_times.begin(), time_it);

      //resize the arrays of the result trajectory point if necessary, no-op if already the right size
      if (!traj_point_it->positions.empty()) {resultTrajectory.points[result_pt_index].positions.resize(axis_names.size());}
      if (!traj_point_it->velocities.empty()) {resultTrajectory.points[result_pt_index].velocities.resize(axis_names.size());}
      if (!traj_point_it->accelerations.empty()) {resultTrajectory.points[result_pt_index].accelerations.resize(axis_names.size());}

      //add the state data to the result trajectory
      for (unsigned int i = 0; i < axis_names.size(); ++i)
      {
        if (!traj_point_it->positions.empty())     {resultTrajectory.points[result_pt_index].positions[i] += way_point_state.position[i];}
        if (!traj_point_it->velocities.empty())    {resultTrajectory.points[result_pt_index].velocities[i] += way_point_state.velocity[i];}
        if (!traj_point_it->accelerations.empty()) {resultTrajectory.points[result_pt_index].accelerations[i] += way_point_state.acceleration[i];}
      }
  }
}
  response.mergedTraj = resultTrajectory;

  return true;
}

}
#endif // POSE_TRAJECTORY_CONTROLLER_POSE_TRAJECTORY_MERGER_IMPL_H
