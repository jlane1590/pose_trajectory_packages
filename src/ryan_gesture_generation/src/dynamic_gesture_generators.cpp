#include <ryan_gesture_generation/dynamic_gesture_generators.h>
//#include <ros/ros.h>
//#include <pose_trajectory_controller/PoseTrajectory.h>
//#include <pose_trajectory_controller/init_pose_trajectory.h>
#include <ryan_gesture_generation/pos_vel_acc_state.h>

namespace ryan_gesture_generation {

  double roundToBase(double input, double base=1.0, double precision=0.00000000001)
  {
    double rem = fmod(input, base);

    if(rem < (base/2.0 - precision))
        return (input - rem);
    else
        return (input + base - rem);
  }

  double floorToBase(double input, double base=1.0)
  {
    return (input - fmod(input, base));
  }

  template <typename T>
  T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
  }

  pose_trajectory_controller::PoseTrajectory createNodTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;
    ros::NodeHandle nh("~");
    std::vector<std::string> ps_axis_names;

    /* get axis names from the parameter server and check that they match the intended axes */
    nh.getParam("/neck_controller/axes", ps_axis_names);

    /* expected axes for Ryan head gesture generation */
    std::vector<std::string> axis_names;
    axis_names.push_back("posX");
    axis_names.push_back("posY");
    axis_names.push_back("posZ");
    axis_names.push_back("roll");
    axis_names.push_back("pitch");
    axis_names.push_back("yaw");
/*
    if (ps_axis_names != axis_names)
    {
      ROS_ERROR("Axes on parameter server don't match the expected axes for Ryan head gesture generation.");
      return trajectory;
    }
*/
    ryan_gesture_generation::PosVelAccState<double> nod_state(6);
    nod_state.position[0] = 0.0;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = 0.0;
    nod_state.position[3] = 0.0;
    nod_state.position[4] = 0.349;
    nod_state.position[5] = 0.0;

    /* the amount of time needed to complete one loop/cycle at max speed, found experimentally */
    double base_period = 0.5;

    /* check input parameters to make sure they are valid */
    //change to ros assert for duration?
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested nod gesture duration is < 0, setting duration to 0.");
    }
    /* time to delay the start of the trajectory from ros::Time::now() */
    start_delay = std::max(start_delay, 0.0);
    /* scale the displacement of the gesture 0-100% */
    intensity = clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = clamp(rate, 0.0, 1.0);

    /* calculate desired period of this gesture */
    double period = (rate==0) ? 0.0 : (intensity*base_period)/rate;

    /* limit period to <= 2*duration so that we can complete at least one half nod */
    if(period > (2*duration))
    {
      period = 2*duration;
      intensity = (period*rate)/base_period;
    }

    /* the number of trajectory points in the gesture corresponds to the number of half nods.
     * if period is 0, intensity is also 0, so there is no actual movement and we just set gesture points to 0 */
    int gesture_points = (period==0.0) ? 0 : round((2*duration)/period);

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
      trajectory.points[i].velocities.resize(axis_names.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period to evenly space out the gesture */
      period = (2*duration)/gesture_points;

      double step_time = 0.0;
      /* set all intermediate points of the nod gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        if(i==1)
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/4));
        else
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/2));

        trajectory.points[i].positions[4] = pow(-1, (i+invert))*intensity*nod_state.position[4];
      }
    }
    trajectory.axis_names = axis_names;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory createTiltTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;
    ros::NodeHandle nh("~");
    std::vector<std::string> ps_axis_names;

    /* get axis names from the parameter server and check that they match the intended axes */
    nh.getParam("/neck_controller/axes", ps_axis_names);

    /* expected axes for Ryan head gesture generation */
    std::vector<std::string> axis_names;
    axis_names.push_back("posX");
    axis_names.push_back("posY");
    axis_names.push_back("posZ");
    axis_names.push_back("roll");
    axis_names.push_back("pitch");
    axis_names.push_back("yaw");
/*
    if (ps_axis_names != axis_names)
    {
      ROS_ERROR("Axes on parameter server don't match the expected axes for Ryan head gesture generation.");
      return trajectory;
    }
*/
    ryan_gesture_generation::PosVelAccState<double> tilt_state(6);
    tilt_state.position[0] = 0.0;
    tilt_state.position[1] = -0.0127;
    tilt_state.position[2] = -0.0127;
    tilt_state.position[3] = 0.349;
    tilt_state.position[4] = 0.0;
    tilt_state.position[5] = 0.0;

    /* the amount of time needed to get to the tilt_state at max speed, found experimentally */
    double base_time = 0.2;

    /* check input parameters to make sure they are valid */
    //change to ros assert for duration?
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested tilt gesture duration is < 0, setting duration to 0.");
    }
    /* time to delay the start of the trajectory from ros::Time::now() */
    start_delay = std::max(start_delay, 0.0);
    /* scale the displacement of the gesture 0-100% */
    intensity = clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = clamp(rate, 0.0, 1.0);

    /* time required to get to the scaled state at the scaled speed */
    double time_to_state = (rate==0) ? 0.0 : (intensity/rate)*base_time;
    /* if the time required to get to the gesture state is more than half the duration,
     * we need to scale back the intensity, so the head can return smoothly to the start point */
    if(time_to_state > (duration/2))
    {
      time_to_state = duration/2;
      intensity = (time_to_state*rate)/base_time;
    }
    /* the tilt gesture typically has 2 gesture points at the tilt state to result in a hold at the state,
     * but if there isn't enough time for a hold there is only one point */
    int gesture_points = 2;

    if(time_to_state == 0.0)
    {
      gesture_points = 0;
    }
    else if(((duration/2)-time_to_state) < 0.001 && ((duration/2)-time_to_state) > -0.001)
    {
      gesture_points = 1;
    }

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
      trajectory.points[i].velocities.resize(axis_names.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    for(std::size_t i=1; i<=gesture_points; ++i)
    {
      if(i==1)
        trajectory.points[i].time_from_start = ros::Duration(time_to_state);
      else
        trajectory.points[i].time_from_start = ros::Duration(duration-time_to_state);

      for(std::size_t j=0; j<6; ++j)
      {
        trajectory.points[i].positions[j] = pow(-1, (j*invert))*intensity*tilt_state.position[j];
      }
    }

    trajectory.axis_names = axis_names;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory createSurpriseTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;
    //ros::NodeHandle nh("~");

    /* expected axes for Ryan head gesture generation */
    std::vector<std::string> axis_names;
    axis_names.push_back("posX");
    axis_names.push_back("posY");
    axis_names.push_back("posZ");
    axis_names.push_back("roll");
    axis_names.push_back("pitch");
    axis_names.push_back("yaw");

    ryan_gesture_generation::PosVelAccState<double> surprise_state(6);
    surprise_state.position[0] = -0.0127;
    surprise_state.position[1] = 0.0;
    surprise_state.position[2] = 0.0127;
    surprise_state.position[3] = 0.0;
    surprise_state.position[4] = -0.175;
    surprise_state.position[5] = 0.0;

    /* the amount of time needed to get to the surprise_state at max speed, found experimentally */
    double base_time = 0.2;

    /* check input parameters to make sure they are valid */
    //change to ros assert for duration?
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested nod gesture duration is < 0, setting duration to 0.");
    }
    /* time to delay the start of the trajectory from ros::Time::now() */
    start_delay = std::max(start_delay, 0.0);
    /* scale the displacement of the gesture 0-100% */
    intensity = clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the surprise state 0-100% */
    rate = clamp(rate, 0.0, 1.0);

    /* time required to get to the scaled state at the scaled speed */
    double time_to_state = (rate==0) ? 0.0 : (intensity/rate)*base_time;
    /* if the time required to get to the gesture state is more than half the duration,
     * we need to scale back the intensity, so the head can return smoothly to the start point */
    if(time_to_state > (duration/2))
    {
      time_to_state = duration/2;
      intensity = (time_to_state*rate)/base_time;
    }

    /* the surprise trajectory typically consists of 3 waypoints: the start and end points at neutral and
     * the point at the surprise_state. Unless rate = 0, then there is no movement and we just have the endpoints*/
    int gesture_points = (time_to_state == 0) ? 0 : 1;

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
      trajectory.points[i].velocities.resize(axis_names.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if gesture_points = 1, set the midpoint of the trajectory at the scaled surprise state */
    if(gesture_points > 0)
    {
      for(std::size_t i=0; i<6; ++i)
      {
        trajectory.points[1].positions[i] = intensity*surprise_state.position[i];
      }
      trajectory.points[1].time_from_start = ros::Duration(time_to_state);
    }

    trajectory.axis_names = axis_names;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory createHoldSurpriseTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;
    //ros::NodeHandle nh("~");

    /* expected axes for Ryan head gesture generation */
    std::vector<std::string> axis_names;
    axis_names.push_back("posX");
    axis_names.push_back("posY");
    axis_names.push_back("posZ");
    axis_names.push_back("roll");
    axis_names.push_back("pitch");
    axis_names.push_back("yaw");

    ryan_gesture_generation::PosVelAccState<double> surprise_state(6);
    surprise_state.position[0] = -0.0127;
    surprise_state.position[1] = 0.0;
    surprise_state.position[2] = 0.0127;
    surprise_state.position[3] = 0.0;
    surprise_state.position[4] = -0.175;
    surprise_state.position[5] = 0.0;

    /* the amount of time needed to get to the surprise_state at max speed, found experimentally */
    double base_time = 0.2;

    /* check input parameters to make sure they are valid */
    //change to ros assert for duration?
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested nod gesture duration is < 0, setting duration to 0.");
    }
    /* time to delay the start of the trajectory from ros::Time::now() */
    start_delay = std::max(start_delay, 0.0);
    /* scale the displacement of the gesture 0-100% */
    intensity = clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the surprise state 0-100% */
    rate = clamp(rate, 0.0, 1.0);

    /* time required to get to the scaled state at the scaled speed */
    double time_to_state = (rate==0) ? 0.0 : (intensity/rate)*base_time;
    /* if the time required to get to the gesture state is more than half the duration,
     * we need to scale back the intensity, so the head can return smoothly to the start point */
    if(time_to_state > (duration/2))
    {
      time_to_state = duration/2;
      intensity = (time_to_state*rate)/base_time;
    }

    /* the hold surprise gesture typically has 2 gesture points at the surprise state to result in a hold at the state,
     * but if there isn't enough time for a hold there is only one point */
    int gesture_points = 2;

    if(time_to_state == 0.0)
    {
      gesture_points = 0;
    }
    else if(((duration/2)-time_to_state) < 0.001 && ((duration/2)-time_to_state) > -0.001)
    {
      gesture_points = 1;
    }

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
      trajectory.points[i].velocities.resize(axis_names.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* Set the midpoints of the trajectory at the scaled surprise state */
    for(std::size_t i=1; i<=gesture_points; ++i)
    {
      if(i==1)
        trajectory.points[i].time_from_start = ros::Duration(time_to_state);
      else
        trajectory.points[i].time_from_start = ros::Duration(duration-time_to_state);

      for(std::size_t j=0; j<6; ++j)
      {
        trajectory.points[i].positions[j] = intensity*surprise_state.position[j];
      }
    }

    trajectory.axis_names = axis_names;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory createCircleTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {

  }

  pose_trajectory_controller::PoseTrajectory createShakeTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {

  }

  pose_trajectory_controller::PoseTrajectory createExaggeratedNodTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {

  }

}
