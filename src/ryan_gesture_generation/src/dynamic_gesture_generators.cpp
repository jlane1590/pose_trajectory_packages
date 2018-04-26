#include <ryan_gesture_generation/dynamic_gesture_generators.h>
//#include <ros/ros.h>
//#include <pose_trajectory_controller/PoseTrajectory.h>
//#include <pose_trajectory_controller/init_pose_trajectory.h>

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

    if (ps_axis_names != axis_names)
    {
      ROS_ERROR("Axes on parameter server don't match the expected axes for Ryan head gesture generation.");
      return trajectory;
    }

    /* get max pitch from the parameter server, as well as max frequency */
    /* default max pitch for a nod is +- 20 degree or 0.349 radians */
    double max_pitch = 0.349;
    nh.getParam("max_pitch", max_pitch);

    /* default max nod frequency is 3 nods/sec, the neck should be able to keep up at the max_pitch */
    double max_nod_frequency = 3.0;
    nh.getParam("max_nod_frequency", max_nod_frequency);

    /* The base frequency of the nod template is 1 nod/sec, this is scaled by the rate parameter */
    //double base_nod_frequency = 1.0;

    /* check input parameters to make sure they are valid */
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested nod gesture duration is < 0, setting duration to 0.");
    }

    start_delay = std::max(start_delay, 0.0);
    intensity = clamp(intensity, 0.0, 1.0);
    rate = clamp(rate, 0.0, 1.0);

    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);
    trajectory.axis_names = axis_names;

    double pitch = intensity*max_pitch;

    double frequency = max_nod_frequency * rate;
    /* regardless of the desired frequency, round the number of nods in the gesture
     * to the nearest half nod to return to the starting point smoothly */
    double total_nods = roundToBase(frequency*duration, 0.5);

    double actual_frequency = total_nods/duration;
    if(actual_frequency > max_nod_frequency)
    {
      total_nods = floorToBase(max_nod_frequency*duration, 0.5);
    }

    int time_divisions = total_nods * 4;
    int gesture_points = total_nods * 2;

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names.size());
      trajectory.points[i].velocities.resize(axis_names.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    if(total_nods >= 0.5)
    {
      double time_slice = duration/time_divisions;
      double step_time = 0;

      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        if(i==1)
          trajectory.points[i].time_from_start = ros::Duration(step_time += time_slice);
        else
          trajectory.points[i].time_from_start = ros::Duration(step_time += 2*time_slice);

        trajectory.points[i].positions[4] = pow(-1, (i+invert))*pitch;
      }
    }

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

    if (ps_axis_names != axis_names)
    {
      ROS_ERROR("Axes on parameter server don't match the expected axes for Ryan head gesture generation.");
      return trajectory;
    }

    /* get max roll from the parameter server, as well as max speed */
    /* default max roll for a nod is +- 20 degree or 0.349 radians */
    double max_roll = 0.349;
    nh.getParam("max_roll", max_roll);

    /* default max angular speed is 0.349 rads/sec */
    double max_angular_speed = 0.349;
    nh.getParam("max_angular_speed", max_angular_speed);

    /* check input parameters to make sure they are valid */
    if(duration < 0)
    {
      duration = 0;
      ROS_INFO("Requested nod gesture duration is < 0, setting duration to 0.");
    }

    start_delay = std::max(start_delay, 0.0);
    intensity = clamp(intensity, 0.0, 1.0);
    rate = clamp(rate, 0.0, 1.0);

    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);
    trajectory.axis_names = axis_names;

    double roll = intensity*max_roll;
    double tilt_speed = rate*max_angular_speed;

    /* if the min speed needed to finish the gesture exceeds the max speed, increase to max speed */
    double min_speed = (2*roll)/duration;
    if(min_speed > max_angular_speed)
    {
      tilt_speed = max_angular_speed;
    }

    if((roll/tilt_speed) > (duration/2))
    {
      roll = (tilt_speed*duration)/2;
    }

    double tilt_duration = roll/tilt_speed;
    int gesture_points = 2;

    if(((duration/2)-tilt_duration) < 0.001)
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
        trajectory.points[i].time_from_start = ros::Duration(tilt_duration);
      else
        trajectory.points[i].time_from_start = ros::Duration(duration-tilt_duration);

      trajectory.points[i].positions[3] = pow(-1, (invert))*roll;
    }

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory createSurpriseTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

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

  pose_trajectory_controller::PoseTrajectory createExagNodTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {

  }

}
