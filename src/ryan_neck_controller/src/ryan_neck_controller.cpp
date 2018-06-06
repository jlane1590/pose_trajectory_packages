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

  //motion_cmd_sub_ = nodeHandle_.subscribe(motion_cmd_topic_, 1, &RyanNeckController::motionCommandCB, this);

  neck_state_sub_ = nodeHandle_.subscribe(neck_state_topic_, 1, &RyanNeckController::neckStateCB, this);

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
  //delete[] current_positions_;
}

bool RyanNeckController::readParameters()
{
  //get action_topic
  //check axis_names to ensure they are what we expect -> this is already checked in gesture generator constructor,
  //no need to do it again

  //add neck state subscriber topic
  if(!nodeHandle_.getParam("neck_controller/action_topic", action_topic_) ||
     //!nodeHandle_.getParam("neck_controller/motion_cmd_topic", motion_cmd_topic_) ||
     !nodeHandle_.getParam("neck_controller/neck_state_topic", neck_state_topic_))
    return false;

  return true;
}

bool RyanNeckController::updateMotionServiceCB(ryan_neck_controller::UpdateMotion::Request& request,
                     ryan_neck_controller::UpdateMotion::Response& response)
{
  ROS_INFO("Service request received");
  switch(request.desired_motion.type)
  {
    case MotionCommand::POSTURE:
    {
    pose_trajectory_controller::PoseTrajectoryPoint goal_pt;
    goal_pt = request.desired_motion.posture;
      //generate the posture trajectory using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectory posture = gesture_generator_.createMoveTrajectory(
            current_state_, goal_pt);
      //check if generated posture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //if we want to run the posture with the current running gesture
      if(request.desired_motion.merge)
      {
        pose_trajectory_controller::PoseTrajectory merged;
        //merge the new posture with the current running gesture
        if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, *current_gesture_ptr_) < 0)
        {
          //publish somewhere an error message
          response.error_code = UpdateMotion::Response::ERROR;
          return true;
        }
        goal.trajectory = merged;

      }
      else //we want to run the posture alone
      {
        goal.trajectory = posture;
      }
      ROS_INFO("Posture request ready to send to action server");
      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      ROS_INFO("Posture goal sent");
      //update the current running posture and trajectory
      pose_trajectory_controller::PoseTrajectory* posturePtr = &posture;
      current_posture_ptr_.reset(posturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      //if we didn't merge, then the old gesture was wiped out
      if(!request.desired_motion.merge)
        current_gesture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
      response.error_code = UpdateMotion::Response::SUCCESSFUL;
      ROS_INFO("Current pointers updated in posture case");
      break;
    }
    case MotionCommand::GESTURE:
    {
      //generate the gesture trajectory using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectory gesture = createGestureFromMsg(request.desired_motion.gesture);
      //check if generated gesture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //if we want to run the gesture with the current running posture
      if(request.desired_motion.merge)
      {
        pose_trajectory_controller::PoseTrajectory merged;
        //merge the new gesture with the current running posture
        if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, *current_posture_ptr_, gesture) < 0)
        {
          //publish somewhere an error message
          response.error_code = UpdateMotion::Response::ERROR;
          return true;
        }
        goal.trajectory = merged;

      }
      else //we want to run the gesture alone
      {
        goal.trajectory = gesture;
      }
      ROS_INFO("Gesture request ready to send to action server");
      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      ROS_INFO("Gesture goal sent");
      //update the current running gesture and trajectory
      pose_trajectory_controller::PoseTrajectory* gesturePtr = &gesture;
      current_gesture_ptr_.reset(gesturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      //if we didn't merge, then the old posture was wiped out
      if(!request.desired_motion.merge)
        current_posture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
      response.error_code = UpdateMotion::Response::SUCCESSFUL;
      ROS_INFO("Current pointers updated in gesture case");
      break;
    }
    case MotionCommand::BOTH:
    {
      //generate the posture and gesture trajectories using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectoryPoint goal_pt;
      goal_pt = request.desired_motion.posture;

      pose_trajectory_controller::PoseTrajectory posture = gesture_generator_.createMoveTrajectory(
            current_state_, goal_pt);

      pose_trajectory_controller::PoseTrajectory gesture = createGestureFromMsg(request.desired_motion.gesture);
      //check if generated posture/gesture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //merge the two input trajectories
      pose_trajectory_controller::PoseTrajectory merged;
      if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, gesture) < 0)
      {
        //publish somewhere an error message
        response.error_code = UpdateMotion::Response::ERROR;
        return true;
      }
      goal.trajectory = merged;
      ROS_INFO("Both request ready to send to action server");

      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      ROS_INFO("Combined goal sent");
      //update the current running posture, gesture, and trajectory
      pose_trajectory_controller::PoseTrajectory* posturePtr = &posture;
      current_posture_ptr_.reset(posturePtr);
      pose_trajectory_controller::PoseTrajectory* gesturePtr = &gesture;
      current_gesture_ptr_.reset(gesturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      response.error_code = UpdateMotion::Response::SUCCESSFUL;
      ROS_INFO("Current pointers updated in both case");
      break;
    }
    default:
      //publish somewhere an error message
      ROS_INFO("Default found");
      response.error_code = UpdateMotion::Response::ERROR;
      break;
  }
  return true;
}

/**
void RyanNeckController::motionCommandCB(const MotionCommand& msg)
{
  switch(msg.type)
  {
    case MotionCommand::POSTURE:
    {
    pose_trajectory_controller::PoseTrajectoryPoint goal_pt;
    goal_pt = msg.posture;
      //generate the posture trajectory using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectory posture = gesture_generator_.createMoveTrajectory(
            current_state_, goal_pt);
      //check if generated posture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //if we want to run the posture with the current running gesture
      if(msg.merge)
      {
        pose_trajectory_controller::PoseTrajectory merged;
        //merge the new posture with the current running gesture
        if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, *current_gesture_ptr_) < 0)
        {
          //publish somewhere an error message
          return;
        }
        goal.trajectory = merged;

      }
      else //we want to run the posture alone
      {
        goal.trajectory = posture;
      }

      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      //update the current running posture and trajectory
      pose_trajectory_controller::PoseTrajectory* posturePtr = &posture;
      current_posture_ptr_.reset(posturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      //if we didn't merge, then the old gesture was wiped out
      if(!msg.merge)
        current_gesture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
      break;
    }
    case MotionCommand::GESTURE:
    {
      //generate the gesture trajectory using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectory gesture = createGestureFromMsg(msg.gesture);
      //check if generated gesture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //if we want to run the gesture with the current running posture
      if(msg.merge)
      {
        pose_trajectory_controller::PoseTrajectory merged;
        //merge the new gesture with the current running posture
        if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, *current_posture_ptr_, gesture) < 0)
        {
          //publish somewhere an error message
          return;
        }
        goal.trajectory = merged;

      }
      else //we want to run the gesture alone
      {
        goal.trajectory = gesture;
      }

      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      //update the current running gesture and trajectory
      pose_trajectory_controller::PoseTrajectory* gesturePtr = &gesture;
      current_gesture_ptr_.reset(gesturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      //if we didn't merge, then the old posture was wiped out
      if(!msg.merge)
        current_posture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
      break;
    }
    case MotionCommand::BOTH:
    {
      //generate the posture and gesture trajectories using dynamic gesture generation
      pose_trajectory_controller::PoseTrajectoryPoint goal_pt;
      goal_pt = msg.posture;

      pose_trajectory_controller::PoseTrajectory posture = gesture_generator_.createMoveTrajectory(
            current_state_, goal_pt);

      pose_trajectory_controller::PoseTrajectory gesture = createGestureFromMsg(msg.gesture);
      //check if generated posture/gesture is empty

      pose_trajectory_controller::FollowPoseTrajectoryGoal goal;

      //merge the two input trajectories
      pose_trajectory_controller::PoseTrajectory merged;
      if(ryan_trajectory_merger::createMergedTrajectory<trajectory_interface::QuinticSplineSegment<double> >(merged, posture, gesture) < 0)
      {
        //publish somewhere an error message
        return;
      }
      goal.trajectory = merged;


      //Send the goal trajectory to the pose trajectory controller
      ac_ptr_->sendGoal(goal,
                        boost::bind(&RyanNeckController::trajControllerDoneCB, this, _1, _2),
                        ActionClient::SimpleActiveCallback(),
                        ActionClient::SimpleFeedbackCallback());
      //update the current running posture, gesture, and trajectory
      pose_trajectory_controller::PoseTrajectory* posturePtr = &posture;
      current_posture_ptr_.reset(posturePtr);
      pose_trajectory_controller::PoseTrajectory* gesturePtr = &gesture;
      current_gesture_ptr_.reset(gesturePtr);
      pose_trajectory_controller::PoseTrajectory* trajectoryPtr = &goal.trajectory;
      current_trajectory_ptr_.reset(trajectoryPtr);
      break;
    }
    default:
      //publish somewhere an error message
      return;
  }
}
**/
void RyanNeckController::neckStateCB(const pose_trajectory_controller::PoseTrajectoryControllerState& msg)
{
  current_state_ = msg.actual;
}

void RyanNeckController::publishMotionStatus()
{
  //other stuff may be needed, otherwise this function is somewhat useless
  motion_status_pub_.publish(status_msg_);
}

void RyanNeckController::trajControllerDoneCB(const actionlib::SimpleClientGoalState& state,
            const pose_trajectory_controller::FollowPoseTrajectoryResultConstPtr& result)
{
  current_gesture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
  current_posture_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());
  current_trajectory_ptr_.reset(new pose_trajectory_controller::PoseTrajectory());

  status_msg_.status = MotionStatus::MOVE_COMPLETE;
}

void RyanNeckController::trajControllerActiveCB()
{
  //update motion status to be ready (for a new motion command) and in motion
}

bool RyanNeckController::updateTrajectory()
{

}

pose_trajectory_controller::PoseTrajectory RyanNeckController::createGestureFromMsg(const GestureCommand& msg)
{
  switch(msg.type)
  {
    case GestureCommand::NOD:
      return gesture_generator_.createNodTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::HEADBANG:
      return gesture_generator_.createHeadbangTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::EX_NOD:
      return gesture_generator_.createExaggeratedNodTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::TILT:
      return gesture_generator_.createTiltTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::SURPRISE:
      return gesture_generator_.createSurpriseTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::HOLD_SURPRISE:
      return gesture_generator_.createHoldSurpriseTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::CIRCLE:
      return gesture_generator_.createCircleTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    case GestureCommand::SHAKE:
      return gesture_generator_.createShakeTrajectory(
            msg.duration, msg.intensity, msg.rate, msg.delay, msg.invert);
    default:
      pose_trajectory_controller::PoseTrajectory empty_traj;
      return empty_traj;
  }
}

} //namespace ryan_neck_controller
