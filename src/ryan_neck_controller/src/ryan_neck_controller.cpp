#include "ryan_neck_controller/ryan_neck_controller.h"

namespace ryan_neck_controller {

RyanNeckController::RyanNeckController(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle),
    gesture_generator_(nodeHandle),
    current_trajectory_ptr_(new pose_trajectory_controller::PoseTrajectory),
    current_gesture_ptr_(new pose_trajectory_controller::PoseTrajectory),
    current_posture_ptr_(new pose_trajectory_controller::PoseTrajectory)
{
  if(!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  status_msg_.status = MotionStatus::MOVE_COMPLETE;

  motion_status_pub_ = nodeHandle_.advertise<ryan_neck_controller::MotionStatus>("motion_status", 1000);

  update_motion_srv_ = nodeHandle_.advertiseService("update_motion",
                                                &RyanNeckController::updateMotionServiceCB, this);

  ac_ptr_.reset(new ActionClient(action_topic_, true));

  ROS_INFO_NAMED("ryan_neck_controller", "Waiting for action server to start.");
  // wait for the action server to start
  ac_ptr_->waitForServer();  // will wait for infinite time

  ROS_INFO_NAMED("ryan_neck_controller", "Action server started.");

  ROS_INFO_NAMED("ryan_neck_controller", "Successfully launched node.");
}

RyanNeckController::~RyanNeckController()
{
}

bool RyanNeckController::readParameters()
{
  //get action_topic
  //check axis_names to ensure they are what we expect -> this is already checked in gesture generator constructor,
  //no need to do it again

  if(!nodeHandle_.getParam("neck_controller/action_topic", action_topic_))
    return false;

  return true;
}

bool RyanNeckController::updateMotionServiceCB(ryan_neck_controller::UpdateMotion::Request& request,
                     ryan_neck_controller::UpdateMotion::Response& response)
{

}

void RyanNeckController::publishMotionStatus()
{
  //other stuff may be needed, otherwise this function is somewhat useless
  motion_status_pub_.publish(status_msg_);
}

void RyanNeckController::trajControllerDoneCB(const actionlib::SimpleClientGoalState& state,
            const pose_trajectory_controller::FollowPoseTrajectoryResultConstPtr& result)
{
  current_trajectory_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
  current_gesture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
  current_posture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());

  status_msg_.status = MotionStatus::MOVE_COMPLETE;
}

bool RyanNeckController::updateTrajectory()
{

}

} //namespace ryan_neck_controller
