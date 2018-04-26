#include <ros/ros.h>
#include <ryan_gesture_generation/dynamic_gesture_generators.h>
#include <pose_trajectory_controller/PoseTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_gesture_tester");
  ros::NodeHandle nh;

  //ROS_INFO("Hello world!");
  pose_trajectory_controller::PoseTrajectory trajectory;
  //trajectory = ryan_gesture_generation::createNodTrajectory(5.0,0.0,2.5,0.0,true);
  trajectory = ryan_gesture_generation::createTiltTrajectory(1.0,1.0,0.3,0.0,false);
  //trajectory = ryan_gesture_generation::createSurpriseTrajectory(1.5,1.0,0.0,0.0,false);
  std::cout << "Trajectory:\n" << trajectory << std::endl;

}
