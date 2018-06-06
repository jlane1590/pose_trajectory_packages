#ifndef RYAN_NECK_CONTROLLER_RYAN_NECK_CONTROLLER_H
#define RYAN_NECK_CONTROLLER_RYAN_NECK_CONTROLLER_H

#include "ryan_gesture_generation/dynamic_gesture_generators.h"
#include "ryan_trajectory_merger/ryan_trajectory_merger.h"
//include validation header

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <pose_trajectory_controller/FollowPoseTrajectoryAction.h>
#include "pose_trajectory_controller/PoseTrajectory.h"
#include "pose_trajectory_controller/PoseTrajectoryControllerState.h"

// Boost
#include <boost/shared_ptr.hpp>

#include "ryan_neck_controller/UpdateMotion.h"
#include "ryan_neck_controller/MotionStatus.h"

namespace ryan_neck_controller {

/*!
 *
 */
class RyanNeckController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RyanNeckController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RyanNeckController();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool updateMotionServiceCB(ryan_neck_controller::UpdateMotion::Request& request,
                       ryan_neck_controller::UpdateMotion::Response& response);

  //void motionCommandCB(const MotionCommand& msg);

  void neckStateCB(const pose_trajectory_controller::PoseTrajectoryControllerState& msg);

  /*!
   * \brief Publish current motion status. Called whenever motion_status_ is changed
   */
  void publishMotionStatus();

  /**
   * ROS pose_trajectory_controller action done callback. Called when the goal trajectory is complete.
   * Resets current_trajectory_, current_posture_, and current_gesture_ and updates motion_status_
   */
  void trajControllerDoneCB(const actionlib::SimpleClientGoalState& state,
              const pose_trajectory_controller::FollowPoseTrajectoryResultConstPtr& result);

  void trajControllerActiveCB();

  pose_trajectory_controller::PoseTrajectory createGestureFromMsg(const GestureCommand& msg);

  /**
   * takes in a motionCmd message and updates the active goal trajectory
   */
  bool updateTrajectory();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic publisher.
  ros::Publisher motion_status_pub_;

  //! ROS action name to connect to.
  std::string action_topic_;

  //ros::Subscriber motion_cmd_sub_;
  //std::string motion_cmd_topic_;

  ros::Subscriber neck_state_sub_;
  std::string neck_state_topic_;

  //! ROS service server.
  ros::ServiceServer update_motion_srv_;

  //! Gesture generator object.
  ryan_gesture_generation::RyanGestureGenerator gesture_generator_;

  typedef actionlib::SimpleActionClient<pose_trajectory_controller::FollowPoseTrajectoryAction> ActionClient;
  typedef boost::shared_ptr<ActionClient> ActionClientPtr;
  ActionClientPtr ac_ptr_;

  ryan_neck_controller::MotionStatus status_msg_;

  pose_trajectory_controller::PoseTrajectoryPoint current_state_;

  pose_trajectory_controller::PoseTrajectoryPtr current_trajectory_ptr_;
  pose_trajectory_controller::PoseTrajectoryPtr current_gesture_ptr_;
  pose_trajectory_controller::PoseTrajectoryPtr current_posture_ptr_;

};

} /* namespace */


#endif // RYAN_NECK_CONTROLLER_RYAN_NECK_CONTROLLER_H
