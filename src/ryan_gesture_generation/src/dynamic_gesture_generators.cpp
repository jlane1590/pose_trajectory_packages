#include <ryan_gesture_generation/dynamic_gesture_generators.h>
//#include <ros/ros.h>
//#include <pose_trajectory_controller/PoseTrajectory.h>
#include <pose_trajectory_controller/init_pose_trajectory.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ryan_gesture_generation
{

namespace internal
{

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

  void getFixedYPR(tf2::Matrix3x3& rot, double& yaw, double& pitch, double& roll)
  {
    pitch = tf2Asin(rot.getRow(0).getZ());
    roll = -tf2Atan2(rot.getRow(1).getZ(),rot.getRow(2).getZ());
    yaw = -tf2Atan2(rot.getRow(0).getY(),rot.getRow(0).getX());
  }

} //namespace internal

  RyanGestureGenerator::RyanGestureGenerator(ros::NodeHandle& nodeHandle)
    : nh_(nodeHandle)
  {
    /* expected axes for Ryan head gesture generation */
    axis_names_.push_back("posX");
    axis_names_.push_back("posY");
    axis_names_.push_back("posZ");
    axis_names_.push_back("roll");
    axis_names_.push_back("pitch");
    axis_names_.push_back("yaw");

    std::vector<std::string> ps_axis_names;

    /* get axis names from the parameter server and check that they match the intended axes */
    nh_.getParam("/neck_controller/axes", ps_axis_names);

    ROS_ASSERT_MSG(ps_axis_names == axis_names_,
                   "Axes on parameter server don't match the expected axes for Ryan head gesture generation. ABORTING...");
  }

  RyanGestureGenerator::~RyanGestureGenerator()
  {
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createHeadbangTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> nod_state(6);
    nod_state.position[0] = 0.0;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = 0.0;
    nod_state.position[3] = 0.0;
    nod_state.position[4] = 0.1749;
    nod_state.position[5] = 0.0;

    ryan_gesture_generation::PosVelAccState<double> posture_state(6);
    posture_state.position[0] = 0.0;
    posture_state.position[1] = 0.0;
    posture_state.position[2] = 0.0;
    posture_state.position[3] = 0.0;
    posture_state.position[4] = 0.1749;
    posture_state.position[5] = 0.5236;

    geometry_msgs::TransformStamped curr_posture_stamped;
    curr_posture_stamped.transform.translation.x = posture_state.position[0];
    curr_posture_stamped.transform.translation.y = posture_state.position[1];
    curr_posture_stamped.transform.translation.z = posture_state.position[2];
    tf2::Quaternion q2;
    q2.setRPY(posture_state.position[3],posture_state.position[4],posture_state.position[5]);
    curr_posture_stamped.transform.rotation.x = q2.x();
    curr_posture_stamped.transform.rotation.y = q2.y();
    curr_posture_stamped.transform.rotation.z = q2.z();
    curr_posture_stamped.transform.rotation.w = q2.w();

    tf2::Quaternion test_q;
    test_q.setRPY(0.0,0.5236,0.5236);
    geometry_msgs::Quaternion test_g;
    test_g.x = test_q.x();
    test_g.y = test_q.y();
    test_g.z = test_q.z();
    test_g.w = test_q.w();
    std::cout << "Test Quaternion:\n" << test_g << std::endl;
    tf2::Matrix3x3 test_m(test_q);
    std::cout << "matrix = \n" << test_m.getRow(0).getX() << "\t" << test_m.getRow(0).getY() << "\t" << test_m.getRow(0).getZ() << "\n"
              << test_m.getRow(1).getX() << "\t" << test_m.getRow(1).getY() << "\t" << test_m.getRow(1).getZ() << "\n"
              << test_m.getRow(2).getX() << "\t" << test_m.getRow(2).getY() << "\t" << test_m.getRow(2).getZ() << std::endl;
    double testR, testP, testY;
    //test_m.getRPY(testR, testP, testY);
    internal::getFixedYPR(test_m, testY, testP, testR);
    std::cout << "RPY = " << testR <<", " << testP << ", " << testY << std::endl;

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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

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
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period and intensity to evenly space out the gesture */
      period = (2*duration)/gesture_points;
      intensity = (period*rate)/base_period;

      geometry_msgs::TransformStamped state_out_stamped;
      geometry_msgs::TransformStamped gesture_transform_stamped;
      gesture_transform_stamped.header.stamp = ros::Time::now();
      gesture_transform_stamped.header.frame_id = "base";
      gesture_transform_stamped.child_frame_id = "head";

      double step_time = 0.0;
      /* set all intermediate points of the nod gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        if(i==1)
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/4));
        else
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/2));

        double sign = pow(-1, (i+invert));
        //transform nod position back to base frame
        gesture_transform_stamped.transform.translation.x = sign*intensity*nod_state.position[0];
        gesture_transform_stamped.transform.translation.y = sign*intensity*nod_state.position[1];
        gesture_transform_stamped.transform.translation.z = sign*intensity*nod_state.position[2];
        tf2::Quaternion q;
        q.setRPY(sign*intensity*nod_state.position[3],sign*intensity*nod_state.position[4],sign*intensity*nod_state.position[5]);
        gesture_transform_stamped.transform.rotation.x = q.x();
        gesture_transform_stamped.transform.rotation.y = q.y();
        gesture_transform_stamped.transform.rotation.z = q.z();
        gesture_transform_stamped.transform.rotation.w = q.w();
/*        std::cout << "Curr Quaternion:\n" << curr_posture_stamped.transform.rotation << std::endl;
        std::cout << "Gesture Quaternion:\n" << gesture_transform_stamped.transform.rotation << std::endl;
        tf2::Quaternion mq = q*q2;
        geometry_msgs::Quaternion gq;
        gq.x = mq.x();
        gq.y = mq.y();
        gq.z = mq.z();
        gq.w = mq.w();
        std::cout << "Result Quaternion:\n" << gq << std::endl;
        tf2::Matrix3x3 m2(mq);

        double R,P,Y;
        m2.getRPY(R,P,Y);
        std::cout << "RPY = " << m2.getRow(0).x() <<", " << P << ", " << Y << std::endl;
        */
        tf2::doTransform(gesture_transform_stamped, state_out_stamped, curr_posture_stamped);
        std::cout << "Orientation:\n" << state_out_stamped.transform.rotation << std::endl;
        tf2::Quaternion q3(state_out_stamped.transform.rotation.x, state_out_stamped.transform.rotation.y,
                           state_out_stamped.transform.rotation.z, state_out_stamped.transform.rotation.w);
        q3.normalize();
        tf2::Matrix3x3 m(q3);
        double roll, pitch, yaw;
        //m.getRPY(roll, pitch, yaw);
        internal::getFixedYPR(m, yaw, pitch, roll);

        trajectory.points[i].positions[0] = state_out_stamped.transform.translation.x;
        trajectory.points[i].positions[1] = state_out_stamped.transform.translation.y;
        trajectory.points[i].positions[2] = state_out_stamped.transform.translation.z;
        trajectory.points[i].positions[3] = roll;
        trajectory.points[i].positions[4] = pitch;
        trajectory.points[i].positions[5] = yaw;

        //trajectory.points[i].positions[4] = pow(-1, (i+invert))*intensity*nod_state.position[4];
      }
    }
    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createNodTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> nod_state(6);
    nod_state.position[0] = 0.0;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = 0.0;
    nod_state.position[3] = 0.0;
    nod_state.position[4] = 0.349;
    nod_state.position[5] = 0.0;
/*
    nod_state.position[0] = -0.0174;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = 0.0015;
    nod_state.position[3] = 0.0;
    nod_state.position[4] = 0.175;
    nod_state.position[5] = 0.0;

    nod_state.position[0] = 0.0;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = 0.0;
    nod_state.position[3] = -0.281;
    nod_state.position[4] = 0.448;
    nod_state.position[5] = 0.5236;
    */
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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

    /* calculate desired period of this gesture */
    double period = (rate==0) ? 0.0 : (intensity*base_period)/rate;

    /* limit period to < duration so that we can complete at least one nod */
    if(period > (duration))
    {
      period = duration;
      intensity = (period*rate)/base_period;
    }

    /* the number of trajectory points in the gesture relates to the number of nods.
     * if period is 0, intensity is also 0, so there is no actual movement and 0 nods */
    int total_nods = (period==0.0) ? 0 : round(duration/period);
    int gesture_points = (total_nods == 0) ? 0 : (2*total_nods - 1);
    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period to evenly space out the gesture */
      period = (duration)/total_nods;
      intensity = (period*rate)/base_period;

      double step_time = 0.0;
      /* set all intermediate points of the nod gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        /* increment time_from_start */
        trajectory.points[i].time_from_start = ros::Duration(step_time += (period/2));
        /* for odd numbered gesture points, go to the nod state. For others stay at neutral */
        if(i%2 == 1)
        {
          for(std::size_t j=0; j<6; ++j)
          {
            trajectory.points[i].positions[j] = pow(-1, invert)*intensity*nod_state.position[j];
          }
        }
        //else
        //{
        //  trajectory.points[i].positions[5] = intensity*0.5236;
        //}
        //trajectory.points[i].positions[4] = pow(-1, (i+invert))*intensity*nod_state.position[4];
      }
    }
    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createExaggeratedNodTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> nod_state(6);
    nod_state.position[0] = 0.00635;
    nod_state.position[1] = 0.0;
    nod_state.position[2] = -0.00635;
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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

    /* calculate desired period of this gesture */
    double period = (rate==0) ? 0.0 : (intensity*base_period)/rate;

    /* limit period to < duration so that we can complete at least one nod */
    if(period > (duration))
    {
      period = duration;
      intensity = (period*rate)/base_period;
    }

    /* the number of trajectory points in the gesture relates to the number of nods.
     * if period is 0, intensity is also 0, so there is no actual movement and 0 nods */
    int total_nods = (period==0.0) ? 0 : round(duration/period);
    int gesture_points = (total_nods == 0) ? 0 : (2*total_nods - 1);
    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period to evenly space out the gesture */
      period = (duration)/total_nods;
      intensity = (period*rate)/base_period;

      double step_time = 0.0;
      /* set all intermediate points of the nod gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        /* increment time_from_start */
        trajectory.points[i].time_from_start = ros::Duration(step_time += (period/2));
        /* for odd numbered gesture points, go to the nod state. For others stay at neutral */
        if(i%2 == 1)
        {
          for(std::size_t j=0; j<6; ++j)
          {
            trajectory.points[i].positions[j] = pow(-1, invert)*intensity*nod_state.position[j];
          }
        }
        //trajectory.points[i].positions[4] = pow(-1, (i+invert))*intensity*nod_state.position[4];
      }
    }
    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createTiltTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> tilt_state(6);
    tilt_state.position[0] = 0.0;
    tilt_state.position[1] = -0.0127;
    tilt_state.position[2] = -0.00635;
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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

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
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
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

    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createMoveTrajectory(
      pose_trajectory_controller::PoseTrajectoryPoint& start_pt, pose_trajectory_controller::PoseTrajectoryPoint& end_pt)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    //Check if the sizes of the points are consistent
    if(!isValid(start_pt, axis_names_.size()) || !isValid(end_pt, axis_names_.size()))
    {
      return trajectory;
    }

    trajectory.points.resize(2);

    if(start_pt.time_from_start <= end_pt.time_from_start)
    {
      trajectory.points[0] = start_pt;
      trajectory.points[1] = end_pt;
    }
    else
    {
      trajectory.points[0] = end_pt;
      trajectory.points[1] = start_pt;
    }

    //make sure the velocities are not empty and fill them with zeros
    //make sure the accelerations are empty
    //this will result in a cubic spline trajectory that stops at the end pt
    trajectory.points[0].velocities.resize(axis_names_.size());
    trajectory.points[1].velocities.resize(axis_names_.size());
    trajectory.points[0].accelerations.resize(0);
    trajectory.points[1].accelerations.resize(0);

    for(std::size_t i=0; i<axis_names_.size(); ++i)
    {
      trajectory.points[0].velocities[i] = 0.0;
      trajectory.points[1].velocities[i] = 0.0;
    }

    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now();

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createSurpriseTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> surprise_state(6);
    surprise_state.position[0] = -0.0159;
    surprise_state.position[1] = 0.0;
    surprise_state.position[2] = 0.00635;
    surprise_state.position[3] = 0.0;
    surprise_state.position[4] = -0.0875;
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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the surprise state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

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
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
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

    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createHoldSurpriseTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> surprise_state(6);
    surprise_state.position[0] = -0.0127;
    surprise_state.position[1] = 0.0;
    surprise_state.position[2] = 0.00635;
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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the surprise state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

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
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
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

    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createShakeTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    ryan_gesture_generation::PosVelAccState<double> shake_state(6);
    shake_state.position[0] = 0.0;
    shake_state.position[1] = 0.0;
    shake_state.position[2] = 0.0;
    shake_state.position[3] = 0.0;
    shake_state.position[4] = 0.0;
    shake_state.position[5] = 0.785;

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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

    /* calculate desired period of this gesture */
    double period = (rate==0) ? 0.0 : (intensity*base_period)/rate;

    /* limit period to <= duration so that we can complete at least one head shake */
    if(period > (duration))
    {
      period = duration;
      intensity = (period*rate)/base_period;
    }

    /* the number of trajectory points in the gesture relates to the number of head shakes.
     * if period is 0, intensity is also 0, so there is no actual movement and we just set gesture points to 0 */
    int total_nods = (period==0.0) ? 0 : round(duration/period);
    int gesture_points = 2*total_nods;

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period to evenly space out the gesture */
      period = duration/total_nods;
      intensity = (period*rate)/base_period;

      double step_time = 0.0;
      /* set all intermediate points of the shake gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        /* increment time_from_start */
        if(i==1)
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/4));
        else
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/2));

        trajectory.points[i].positions[5] = pow(-1, (i+invert))*intensity*shake_state.position[5];
      }
    }
    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

  pose_trajectory_controller::PoseTrajectory RyanGestureGenerator::createCircleTrajectory(
      double duration, double intensity, double rate, double start_delay, bool invert)
  {
    pose_trajectory_controller::PoseTrajectory trajectory;

    std::vector< ryan_gesture_generation::PosVelAccState<double> > circle_states;
    ryan_gesture_generation::PosVelAccState<double> circle_state(6);
    /* Only need to specify the axes that are in use for the gesture, the rest default to zero */
    circle_state.position[0] = 0.0127;
    circle_state.position[1] = 0.0;
    circle_state.velocity[0] = 0.0;
    circle_state.velocity[1] = -0.04;
    circle_states.push_back(circle_state);
    circle_state.position[0] = 0.0;
    circle_state.position[1] = -0.0127;
    circle_state.velocity[0] = -0.04;
    circle_state.velocity[1] = 0.0;
    circle_states.push_back(circle_state);
    circle_state.position[0] = -0.0127;
    circle_state.position[1] = 0.0;
    circle_state.velocity[0] = 0.0;
    circle_state.velocity[1] = 0.04;
    circle_states.push_back(circle_state);
    circle_state.position[0] = 0.0;
    circle_state.position[1] = 0.0127;
    circle_state.velocity[0] = 0.04;
    circle_state.velocity[1] = 0.0;
    circle_states.push_back(circle_state);

    /* the amount of time needed to complete one loop/cycle at max speed, found experimentally */
    double base_period = 1.0;

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
    intensity = internal::clamp(intensity, 0.0, 1.0);
    /* scale the movement speed to the tilt state 0-100% */
    rate = internal::clamp(rate, 0.0, 1.0);

    /* calculate desired period of this gesture */
    double period = (rate==0) ? 0.0 : (intensity*base_period)/rate;

    /* limit period to <= duration so that we can complete at least one revolution */
    if(period > (duration))
    {
      period = duration;
      intensity = (period*rate)/base_period;
    }

    /* the number of trajectory points in the gesture corresponds to the number of revolutions.
     * if period is 0, intensity is also 0, so there is no actual movement and we just set revolutions to 0 */
    int revolutions = (period==0.0) ? 0 : round((duration)/period);
    int gesture_points = 4*revolutions;

    /* resize the trajectory elements and set start and end values */
    trajectory.points.resize(gesture_points+2);
    for (std::size_t i=0; i<(gesture_points+2); ++i)
    {
      trajectory.points[i].positions.resize(axis_names_.size());
      trajectory.points[i].velocities.resize(axis_names_.size());
    }
    trajectory.points.back().time_from_start = ros::Duration(duration);

    /* if intensity > 0, there are gesture points so populate them */
    if(gesture_points > 0)
    {
      /* correct period and intensity to evenly space out the gesture */
      period = (duration)/revolutions;
      intensity = (period*rate)/base_period;

      double step_time = 0.0;
      /* set all intermediate points of the circle gesture */
      for(std::size_t i=1; i<=gesture_points; ++i)
      {
        if(i==1)
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/8));
        else
          trajectory.points[i].time_from_start = ros::Duration(step_time += (period/4));
        if(!invert)
        {
          for(std::size_t j=0; j<6; ++j)
          {
            trajectory.points[i].positions[j] = intensity*circle_states[(i-1)%4].position[j];
            trajectory.points[i].velocities[j] = intensity*rate*circle_states[(i-1)%4].velocity[j];
          }
        }
        else
        {
          for(std::size_t j=0; j<6; ++j)
          {
            trajectory.points[i].positions[j] = intensity*circle_states[((i%4)+pow(-1,i))].position[j];
            trajectory.points[i].velocities[j] = -intensity*rate*circle_states[((i%4)+pow(-1,i))].velocity[j];
          }
        }
      }
    }
    trajectory.axis_names = axis_names_;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay);

    return trajectory;
  }

} //namespace ryan_gesture_generation
