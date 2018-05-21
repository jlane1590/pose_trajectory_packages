#ifndef RYAN_TRAJECTORY_MERGER_RYAN_TRAJECTORY_MERGER_H
#define RYAN_TRAJECTORY_MERGER_RYAN_TRAJECTORY_MERGER_H

#include <pose_trajectory_controller/init_pose_trajectory.h>

#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>

#include <iterator>
#include <sstream>

namespace ryan_trajectory_merger {

namespace internal {

void getFixedYPR(tf2::Matrix3x3& rot, double& yaw, double& pitch, double& roll)
{
  pitch = tf2Asin(rot.getRow(0).getZ());
  roll = -tf2Atan2(rot.getRow(1).getZ(),rot.getRow(2).getZ());
  yaw = -tf2Atan2(rot.getRow(0).getY(),rot.getRow(0).getX());
}

void rotateV3(tf2::Matrix3x3& rot, tf2::Vector3& in, tf2::Vector3& out)
{
  out.setX(rot[0].dot(in));
  out.setY(rot[1].dot(in));
  out.setZ(rot[2].dot(in));
}

} //namespace internal

/**
 *  returns error_code (SUCCESSFUL = 0, EMPTY_TRAJECTORIES = -1, INVALID_TRAJECTORIES = -2)
 */
template <class SegmentImpl>
int createMergedTrajectory(pose_trajectory_controller::PoseTrajectory& merged,
                            pose_trajectory_controller::PoseTrajectory& posture,
                            pose_trajectory_controller::PoseTrajectory& gesture,
                            double epsilon = 0.1)
{
  pose_trajectory_controller::PoseTrajectory resultTrajectory;
  //check the validity of the posture and gesture trajectories

  //handle empty input trajectories
  if(posture.points.empty())
  {
    if(gesture.points.empty())
    {
      //both input trajectories are empty, nothing to be done, respond with empty trajectory
      ROS_INFO("input trajectories are empty, returning empty trajectory");
      merged = resultTrajectory;
      return -1;
    }
    else
    {
      //posture is empty so return gesture
      merged = gesture;
      return 0;
    }
  }
  else if(gesture.points.empty())
  {
    //gesture is empty so return posture
    merged = posture;
    return 0;
  }
  //Otherwise if the trajectories are not empty, they should consist of at least 2 waypoints to create a valid segment
  if(posture.points.size() < 2 || gesture.points.size() < 2)
  {
    ROS_ERROR("Input posture and gesture must contain at least 2 waypoints.");
    merged = resultTrajectory;
    return -2;
  }

  // Non strictly-monotonic time waypoints
  if (!isTimeStrictlyIncreasing(posture) || !isTimeStrictlyIncreasing(gesture))
  {
    ROS_ERROR("Input trajectory contains waypoints that are not strictly increasing in time.");
    merged = resultTrajectory;
    return -2;
  }
  // Non matching controlled axes
  const std::vector<std::string> axis_names = posture.axis_names;
  // Mapping vector contains the map between the posture axis order and gesture axis order
  // The vector is empty if the axes don't match
  std::vector<unsigned int> mapping_vector = pose_trajectory_controller::internal::mapping(gesture.axis_names,axis_names);

  if (mapping_vector.empty())
  {
    ROS_ERROR("Cannot merge given posture and gesture. The controlled axes do not match.");
    merged = resultTrajectory;
    return -2;
  }

  //Check that the sizes of the trajectory members are consistent
  if(!isValid(posture) || !isValid(gesture))
  {
    ROS_ERROR("Posture or gesture trajectory members are not of consistent size.");
    merged = resultTrajectory;
    return -2;
  }

  //At this point we have valid posture and gesture trajectories to merge

  //the start time of the merged trajectory will be the earlier start time of the input trajectories
  resultTrajectory.header.stamp = (posture.header.stamp < gesture.header.stamp ?
                                     posture.header.stamp : gesture.header.stamp);

  std::vector<ros::Duration> result_way_times;

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt1_it = posture.points.begin();
  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator pt2_it = gesture.points.begin();

  while((std::distance(pt1_it, posture.points.end()) >= 1) && (std::distance(pt2_it, gesture.points.end()) >= 1))
  {
    ros::Duration posture_rel_time = (posture.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start;
    ros::Duration gesture_rel_time = (gesture.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start;

    if((posture_rel_time - gesture_rel_time).toSec() < -epsilon)
    {
      result_way_times.push_back(posture_rel_time);
      pt1_it++;
    }
    else if((posture_rel_time - gesture_rel_time).toSec() > epsilon)
    {
      result_way_times.push_back(gesture_rel_time);
      pt2_it++;
    }
    else
    {
      result_way_times.push_back(posture_rel_time);
      pt1_it++;
      pt2_it++;
    }
  }
  /* posture ends first, add remaining gesture waypoint times */
  if(std::distance(pt1_it, posture.points.end()) < 1)
  {
    for(; pt2_it != gesture.points.end(); ++pt2_it)
    {
      result_way_times.push_back((gesture.header.stamp - resultTrajectory.header.stamp) + pt2_it->time_from_start);
    }
  }
  /* gesture ends first, add remaining posture waypoint times */
  else if(std::distance(pt2_it, gesture.points.end()) < 1)
  {
    for(; pt1_it != posture.points.end(); ++pt1_it)
    {
      result_way_times.push_back((posture.header.stamp - resultTrajectory.header.stamp) + pt1_it->time_from_start);
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

  /* always use the posture axis order in the merged trajectory */
  resultTrajectory.axis_names = axis_names;
  resultTrajectory.points.resize(result_way_times.size());
  /* move the waypoint times into the merged trajectory */
  for(int i = 0; i < result_way_times.size(); ++i)
  {
    resultTrajectory.points[i].time_from_start = result_way_times[i];
  }

  std::vector<pose_trajectory_controller::PoseTrajectoryPoint>::iterator traj_point_it, prev_traj_point_it;

  typedef pose_trajectory_controller::PoseTrajectorySegment<SegmentImpl> Segment;
  Segment current_pose_segment;

  /* initialize prev_traj_point_it to the end of the gesture so we know the first traj_point_it cannot equal it */
  prev_traj_point_it = gesture.points.end();

  /* first sample the posture at each way point time */
  //for each way point time in the result
  for(std::vector<ros::Duration>::iterator time_it = result_way_times.begin();
      time_it != result_way_times.end(); ++time_it)
  {
    //for each waypoint in the posture
    for(traj_point_it = posture.points.begin(); traj_point_it != posture.points.end(); ++traj_point_it)
    {
      //check the time_from_start of each point in the posture,
      //the first point after the current result way point is the end of the segment to sample
      ros::Duration relative_time = (posture.header.stamp - resultTrajectory.header.stamp) + traj_point_it->time_from_start;
      if(relative_time > *time_it)
      {
        break;
      }
    }
    //if the previous requested time occurred during a different segment, update the segment to sample
    if(traj_point_it != prev_traj_point_it)
    {
      /* if the resulting way point occurs before the start of the posture trajectory,
       * sample a segment from the start point to the start point */
      if(traj_point_it == posture.points.begin())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*traj_point_it);

        //create a pose trajectory segment going to and from the starting point
        double end_time = (posture.header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment.init(end_time, segment_state, end_time, segment_state);
      }

      /* if the resulting way point occurs after the end of the posture trajectory,
       * sample a segment from the end point to the end point */
      else if(traj_point_it == posture.points.end())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to and from the end point
        double end_time = (posture.header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();

        current_pose_segment.init(end_time, segment_state, end_time, segment_state);
      }

      /* otherwise the resulting way point occurs somewhere along the posture trajectory,
       * so sample a segment to this point from the previous point */
      else
      {
        //create a segment state out of this pose point
        typename Segment::State segment_end_state(*traj_point_it);
        //create a segment state out of the previous pose point
        typename Segment::State segment_start_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to the end state from the start state
        double start_time = (posture.header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();
        double end_time = (posture.header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment.init(start_time, segment_start_state, end_time, segment_end_state);
      }

      //update the previous trajectory point iterator
      prev_traj_point_it = traj_point_it;
    }

    //sample the posture segment at the result trajectory way point time
    typename Segment::State way_point_state = typename Segment::State(axis_names.size());
    current_pose_segment.sample(time_it->toSec(), way_point_state);

    int result_pt_index = std::distance(result_way_times.begin(), time_it);

    /* resize the arrays of the result trajectory point if necessary, no-op if already the right size */
    if (!posture.points.begin()->positions.empty()) {resultTrajectory.points[result_pt_index].positions.resize(axis_names.size());}
    if (!posture.points.begin()->velocities.empty()) {resultTrajectory.points[result_pt_index].velocities.resize(axis_names.size());}
    if (!posture.points.begin()->accelerations.empty()) {resultTrajectory.points[result_pt_index].accelerations.resize(axis_names.size());}

    /* move the posture state data to the result trajectory */
    for (unsigned int i = 0; i < axis_names.size(); ++i)
    {
      if (!posture.points.begin()->positions.empty())     {resultTrajectory.points[result_pt_index].positions[i] = way_point_state.position[i];}
      if (!posture.points.begin()->velocities.empty())    {resultTrajectory.points[result_pt_index].velocities[i] = way_point_state.velocity[i];}
      if (!posture.points.begin()->accelerations.empty()) {resultTrajectory.points[result_pt_index].accelerations[i] = way_point_state.acceleration[i];}
    }
  }

  /* now we have the posture state at each way point and we can transform
   * the gesture trajectory back to the neck frame for each point */

  /* initialize prev_traj_point_it to the end of the posture so we know the first traj_point_it cannot equal it */
  prev_traj_point_it = posture.points.end();

  //for each way point time in the result
  for(std::vector<ros::Duration>::iterator time_it = result_way_times.begin();
      time_it != result_way_times.end(); ++time_it)
  {

    //for each waypoint in the gesture
    for(traj_point_it = gesture.points.begin(); traj_point_it != gesture.points.end(); ++traj_point_it)
    {
      //check the time_from_start of each point in the gesture,
      //the first point after the current result way point is the end of the segment to sample
      ros::Duration relative_time = (gesture.header.stamp - resultTrajectory.header.stamp) + traj_point_it->time_from_start;
      if(relative_time > *time_it)
      {
        break;
      }
    }

    //if the previous requested time occurred during a different segment, update the segment to sample
    if(traj_point_it != prev_traj_point_it)
    {
      /* if the resulting way point occurs before the start of the gesture trajectory,
       * sample a segment from the start point to the start point */
      if(traj_point_it == gesture.points.begin())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*traj_point_it);

        //create a pose trajectory segment going to and from the starting point
        double end_time = (gesture.header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment.init(end_time, segment_state, end_time, segment_state);
      }

      /* if the resulting way point occurs after the end of the gesture trajectory,
       * sample a segment from the end point to the end point */
      else if(traj_point_it == gesture.points.end())
      {
        //create a segment state out of this pose point
        typename Segment::State segment_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to and from the end point
        double end_time = (gesture.header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();

        current_pose_segment.init(end_time, segment_state, end_time, segment_state);
      }

      /* otherwise the resulting way point occurs somewhere along the gesture trajectory,
       * so sample a segment to this point from the previous point */
      else
      {
        //create a segment state out of this pose point
        typename Segment::State segment_end_state(*traj_point_it);
        //create a segment state out of the previous pose point
        typename Segment::State segment_start_state(*(std::prev(traj_point_it)));

        //create a pose trajectory segment going to the end state from the start state
        double start_time = (gesture.header.stamp - resultTrajectory.header.stamp).toSec() + std::prev(traj_point_it)->time_from_start.toSec();
        double end_time = (gesture.header.stamp - resultTrajectory.header.stamp).toSec() + traj_point_it->time_from_start.toSec();

        current_pose_segment.init(start_time, segment_start_state, end_time, segment_end_state);
      }

      //update the previous trajectory point iterator
      prev_traj_point_it = traj_point_it;
    }

    //sample the gesture segment at the result trajectory way point time
    typename Segment::State way_point_state = typename Segment::State(axis_names.size());
    current_pose_segment.sample(time_it->toSec(), way_point_state);

    int result_pt_index = std::distance(result_way_times.begin(), time_it);

    /* now we have to transform the gesture state back to the neck frame from the head frame */
    geometry_msgs::TransformStamped posture_transform_stamped;
    geometry_msgs::TransformStamped gesture_transform_stamped;
    geometry_msgs::TransformStamped merged_transform_stamped;

    posture_transform_stamped.transform.translation.x = resultTrajectory.points[result_pt_index].positions[0];
    posture_transform_stamped.transform.translation.y = resultTrajectory.points[result_pt_index].positions[1];
    posture_transform_stamped.transform.translation.z = resultTrajectory.points[result_pt_index].positions[2];
    tf2::Quaternion posture_quat;
    posture_quat.setRPY(resultTrajectory.points[result_pt_index].positions[3],
        resultTrajectory.points[result_pt_index].positions[4], resultTrajectory.points[result_pt_index].positions[5]);
    posture_transform_stamped.transform.rotation.x = posture_quat.x();
    posture_transform_stamped.transform.rotation.y = posture_quat.y();
    posture_transform_stamped.transform.rotation.z = posture_quat.z();
    posture_transform_stamped.transform.rotation.w = posture_quat.w();
    tf2::Matrix3x3 posture_rot(posture_quat);

    /* if the gesture contains positions, transform it back to the neck frame */
    if(!gesture.points.begin()->positions.empty())
    {
      /* resize the position array of the result trajectory point if necessary, no-op if already the right size */
      resultTrajectory.points[result_pt_index].positions.resize(axis_names.size());
      /* construct the gesture transform for the position */
      gesture_transform_stamped.transform.translation.x = way_point_state.position[0];
      gesture_transform_stamped.transform.translation.y = way_point_state.position[1];
      gesture_transform_stamped.transform.translation.z = way_point_state.position[2];
      tf2::Quaternion gesture_quat;
      gesture_quat.setRPY(way_point_state.position[3], way_point_state.position[4], way_point_state.position[5]);
      gesture_transform_stamped.transform.rotation.x = gesture_quat.x();
      gesture_transform_stamped.transform.rotation.y = gesture_quat.y();
      gesture_transform_stamped.transform.rotation.z = gesture_quat.z();
      gesture_transform_stamped.transform.rotation.w = gesture_quat.w();

      /* transform the gesture position to the neck frame */
      tf2::doTransform(gesture_transform_stamped, merged_transform_stamped, posture_transform_stamped);
      /* extract roll, pitch, and yaw from the transformed quaternion */
      tf2::Quaternion merged_quat(merged_transform_stamped.transform.rotation.x, merged_transform_stamped.transform.rotation.y,
                         merged_transform_stamped.transform.rotation.z, merged_transform_stamped.transform.rotation.w);
      tf2::Matrix3x3 merged_rot(merged_quat);
      double roll, pitch, yaw;
      internal::getFixedYPR(merged_rot, yaw, pitch, roll);

      /* move the merged position data to the result trajectory */
      resultTrajectory.points[result_pt_index].positions[0] = merged_transform_stamped.transform.translation.x;
      resultTrajectory.points[result_pt_index].positions[1] = merged_transform_stamped.transform.translation.y;
      resultTrajectory.points[result_pt_index].positions[2] = merged_transform_stamped.transform.translation.z;
      resultTrajectory.points[result_pt_index].positions[3] = roll;
      resultTrajectory.points[result_pt_index].positions[4] = pitch;
      resultTrajectory.points[result_pt_index].positions[5] = yaw;
    }

    /* if the gesture point contains velocities, transform it back to the neck frame */
    if(!gesture.points.begin()->velocities.empty())
    {
      /* resize the velocity array of the result trajectory point if necessary, no-op if already the right size */
      resultTrajectory.points[result_pt_index].velocities.resize(axis_names.size());

      /* rotate the linear and angular velocity of the gesture back to the base frame */
      tf2::Vector3 gest_linear_vel, rot_linear_vel, gest_angular_vel, rot_angular_vel;
      gest_linear_vel.setX(way_point_state.velocity[0]);
      gest_linear_vel.setY(way_point_state.velocity[1]);
      gest_linear_vel.setZ(way_point_state.velocity[2]);
      gest_angular_vel.setX(way_point_state.velocity[3]);
      gest_angular_vel.setY(way_point_state.velocity[4]);
      gest_angular_vel.setZ(way_point_state.velocity[5]);
      internal::rotateV3(posture_rot, gest_linear_vel, rot_linear_vel);
      internal::rotateV3(posture_rot, gest_angular_vel, rot_angular_vel);

      /* combine the rotated velocity data with the result trajectory */
      resultTrajectory.points[result_pt_index].velocities[0] += rot_linear_vel.x();
      resultTrajectory.points[result_pt_index].velocities[1] += rot_linear_vel.y();
      resultTrajectory.points[result_pt_index].velocities[2] += rot_linear_vel.z();
      resultTrajectory.points[result_pt_index].velocities[3] += rot_angular_vel.x();
      resultTrajectory.points[result_pt_index].velocities[4] += rot_angular_vel.y();
      resultTrajectory.points[result_pt_index].velocities[5] += rot_angular_vel.z();
    }

    /* if the gesture point contains accelerations, transform it back to the neck frame */
    if(!gesture.points.begin()->accelerations.empty())
    {
      /* resize the acceleration array of the result trajectory point if necessary, no-op if already the right size */
      resultTrajectory.points[result_pt_index].accelerations.resize(axis_names.size());

      /* rotate the linear and angular acceleration of the gesture back to the base frame */
      tf2::Vector3 gest_linear_acc, rot_linear_acc, gest_angular_acc, rot_angular_acc;
      gest_linear_acc.setX(way_point_state.acceleration[0]);
      gest_linear_acc.setY(way_point_state.acceleration[1]);
      gest_linear_acc.setZ(way_point_state.acceleration[2]);
      gest_angular_acc.setX(way_point_state.acceleration[3]);
      gest_angular_acc.setY(way_point_state.acceleration[4]);
      gest_angular_acc.setZ(way_point_state.acceleration[5]);
      internal::rotateV3(posture_rot, gest_linear_acc, rot_linear_acc);
      internal::rotateV3(posture_rot, gest_angular_acc, rot_angular_acc);

      /* combine the rotated acceleration data with the result trajectory */
      resultTrajectory.points[result_pt_index].accelerations[0] += rot_linear_acc.x();
      resultTrajectory.points[result_pt_index].accelerations[1] += rot_linear_acc.y();
      resultTrajectory.points[result_pt_index].accelerations[2] += rot_linear_acc.z();
      resultTrajectory.points[result_pt_index].accelerations[3] += rot_angular_acc.x();
      resultTrajectory.points[result_pt_index].accelerations[4] += rot_angular_acc.y();
      resultTrajectory.points[result_pt_index].accelerations[5] += rot_angular_acc.z();
    }

  }

  merged = resultTrajectory;

  return 0;
}

} //namespace ryan_trajectory_merger
#endif // RYAN_TRAJECTORY_MERGER_RYAN_TRAJECTORY_MERGER_H

