#ifndef RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H
#define RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H

#include <ros/ros.h>
#include <pose_trajectory_controller/PoseTrajectory.h>
#include <ryan_gesture_generation/pos_vel_acc_state.h>

namespace ryan_gesture_generation {

class RyanGestureGenerator
{
public:

  RyanGestureGenerator(ros::NodeHandle& nodehandle);

  virtual ~RyanGestureGenerator();

  pose_trajectory_controller::PoseTrajectory createNodTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createExaggeratedNodTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createTiltTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createSurpriseTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createHoldSurpriseTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createCircleTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createShakeTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

private:

  ros::NodeHandle nh_;
  std::vector<std::string> axis_names_;

};

}

#endif // RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H
