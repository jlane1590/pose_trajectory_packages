#include <ros/ros.h>
#include <ryan_gesture_generation/dynamic_gesture_generators.h>
#include <pose_trajectory_controller/PoseTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_gesture_tester");
  ros::NodeHandle nh;

  ryan_gesture_generation::RyanGestureGenerator ryanGestureGenerator(nh);
  pose_trajectory_controller::PoseTrajectory trajectory;
  //trajectory = ryanGestureGenerator.createNodTrajectory(2.0,1.0,0.1,0.0,true);
  //trajectory = ryanGestureGenerator.createExaggeratedNodTrajectory(1.0,1.0,0.1,0.0,true);
  //trajectory = ryanGestureGenerator.createTiltTrajectory(3.0,1.0,0.0,0.0,false);
  trajectory = ryanGestureGenerator.createSurpriseTrajectory(2.0,1.0,1.0,0.0,false);
  //trajectory = ryanGestureGenerator.createHoldSurpriseTrajectory(2.0,1.0,0.0,0.0,false);
  //trajectory = ryanGestureGenerator.createShakeTrajectory(1.0,1.0,0.1,0.0,false);
  //trajectory = ryanGestureGenerator.createCircleTrajectory(1.0,1.0,1.0,0.0,true);
  std::cout << "Trajectory:\n" << trajectory << std::endl;

}
