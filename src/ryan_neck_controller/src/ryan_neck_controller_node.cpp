#include <ros/ros.h>
#include "ryan_neck_controller/ryan_neck_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ryan_neck_controller");
  ros::NodeHandle nh;

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ryan_neck_controller::RyanNeckController ryanNeckController(nh);

  ros::waitForShutdown();
  //ros::spin();
}
