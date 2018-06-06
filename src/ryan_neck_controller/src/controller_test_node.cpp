#include <ros/ros.h>
#include "ryan_neck_controller/UpdateMotion.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_test_node");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<ryan_neck_controller::UpdateMotion>("update_motion");
  ROS_INFO("Connecting to Update Motion Service...");
  client.waitForExistence();
  ROS_INFO("Connected to Update Motion Service");

  //posture point to turn head left in one second
  pose_trajectory_controller::PoseTrajectoryPoint posture_left;
  posture_left.positions.resize(6);
  posture_left.velocities.resize(6);
  posture_left.positions[5] = 1.05;
  posture_left.time_from_start = ros::Duration(1.0);

  //posture point to turn head right in four seconds
  pose_trajectory_controller::PoseTrajectoryPoint posture_right;
  posture_right.positions.resize(6);
  posture_right.velocities.resize(6);
  posture_right.positions[5] = -1.05;
  posture_right.time_from_start = ros::Duration(4.0);

  //gesture command to nod for 2 seconds
  ryan_neck_controller::GestureCommand nod_gesture;
  nod_gesture.type = ryan_neck_controller::GestureCommand::NOD;
  nod_gesture.duration = 2.0;
  nod_gesture.intensity = 0.5;
  nod_gesture.rate = 0.5;
  nod_gesture.delay = 0.0;
  nod_gesture.invert = false;

  ryan_neck_controller::UpdateMotion srv;

  //send a service request to turn left
  srv.request.desired_motion.type = ryan_neck_controller::MotionCommand::POSTURE;
  srv.request.desired_motion.merge = false;
  srv.request.desired_motion.posture = posture_left;
  client.call(srv);
  ROS_INFO("Left posture request sent");

  //wait for 2 seconds
  ros::Duration(2.0).sleep();

  //send a service request to turn right
  srv.request.desired_motion.type = ryan_neck_controller::MotionCommand::POSTURE;
  srv.request.desired_motion.merge = false;
  srv.request.desired_motion.posture = posture_right;
  client.call(srv);
  ROS_INFO("Right posture request sent");

  //wait for 1 second
  ros::Duration(1.0).sleep();

  //send a service request to merge a nod with the current posture move
  srv.request.desired_motion.type = ryan_neck_controller::MotionCommand::GESTURE;
  srv.request.desired_motion.merge = true;
  srv.request.desired_motion.gesture = nod_gesture;
  client.call(srv);
  ROS_INFO("Nod gesture request sent");

  return 0;
}
