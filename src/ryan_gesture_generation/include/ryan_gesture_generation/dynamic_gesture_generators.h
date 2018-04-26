#ifndef RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H
#define RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H

#include <ros/ros.h>
#include <pose_trajectory_controller/PoseTrajectory.h>

namespace ryan_gesture_generation {

  /**
    * @brief createNodTrajectory constructs a PoseTrajectory message for a nod on the Ryan 2.0 RSP neck
    * @param duration specifies how long the nod should last
    * @param intensity is a scaling factor as a percentage of maximum deflection
    * @param rate is a scaling factor as a percentage of maximum nod frequency
    * @param start_delay delays the start of the trajectory from the current wall time by start_delay seconds
    * @param invert flips the starting direction of the nod
    * @return the constructed PoseTrajectory message
    */
  pose_trajectory_controller::PoseTrajectory createNodTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

  pose_trajectory_controller::PoseTrajectory createTiltTrajectory(
      double duration, double intensity=1.0, double rate=1.0, double start_delay=0.0, bool invert=false);

}

#endif // RYAN_GESTURE_GENERATION_DYNAMIC_GESTURE_GENERATORS_H
