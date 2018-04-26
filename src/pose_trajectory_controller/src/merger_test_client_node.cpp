#include "ros/ros.h"
#include <pose_trajectory_controller/MergeTwoPoseTraj.h>
#include <pose_trajectory_controller/PoseTrajectory.h>

namespace pose_trajectory_merger
{
static const double SEC_PER_TRAJ_POINT_1 = 8;  // time between points
static const std::size_t TRAJ_POINTS_1 = 2;     // number of points to generate
static const double SEC_PER_TRAJ_POINT_2 = 0.5;  // time between points
static const std::size_t TRAJ_POINTS_2 = 6;     // number of points to generate

class TestTrajectory
{
public:
  /**
   * \brief Constructor
   */
  TestTrajectory(ros::NodeHandle& nodeHandle)
    : nh_private_("~")
  {
    ros::ServiceClient client = nh_private_.serviceClient<pose_trajectory_controller::MergeTwoPoseTraj>("/pose_trajectory_merger/merge_trajectories");
    pose_trajectory_controller::MergeTwoPoseTraj srv;

    srv.request.traj1 = createTrajectory(1);
    srv.request.traj2 = createTrajectory(2);

    if (client.call(srv))
    {
      ROS_INFO("Call Succeeded");
      std::cout << "Trajectory:\n" << srv.response.mergedTraj << std::endl;
    }
    else
      ROS_INFO("Failed to call service");

  }
  /**
   * \brief Create random trajectory
   */
  pose_trajectory_controller::PoseTrajectory createTrajectory(int num)
  {
    std::vector<std::string> axis_names;
    double min_joint_value = -1.570796327;
    double max_joint_value = 1.570796327;

    // Get axis names
    nh_private_.getParam("/neck_controller/axes", axis_names);
    if (axis_names.size() == 0)
    {
      ROS_FATAL_STREAM_NAMED(
          "init",
          "No axes found on parameter server for controller, did you load the proper yaml file?"
              << " Namespace: " << nh_private_.getNamespace() << "/neck_controller/axes");
      exit(-1);
    }

    nh_private_.getParam("min_joint_value", min_joint_value);
    nh_private_.getParam("max_joint_value", max_joint_value);
    ROS_DEBUG_STREAM_NAMED("test_trajectory", "Creating trajectory with joint values from "
                                                  << min_joint_value << " to " << max_joint_value);

    // Create header
    pose_trajectory_controller::PoseTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    if(num == 2)
      trajectory.header.stamp += ros::Duration(2.0);
    trajectory.axis_names = axis_names;

    // Create trajectory with x points
    if(num == 1)
    {
      trajectory.points.resize(TRAJ_POINTS_1);
      for (std::size_t i = 0; i < TRAJ_POINTS_1; ++i)
      {
        trajectory.points[i].positions.resize(axis_names.size());
        trajectory.points[i].velocities.resize(axis_names.size());
        // for each joint
        for (std::size_t j = 0; j < axis_names.size(); ++j)
        {
          //trajectory.points[i].positions[j] = dRand(min_joint_value, max_joint_value);
          //if(j == 2)
            //trajectory.points[i].positions[j] = i%2==0 ? -0.5 : 0.5;
          //else
          trajectory.points[i].positions[j] = 0;
          trajectory.points[i].velocities[j] = 0;
          trajectory.points[i].time_from_start = ros::Duration(i * SEC_PER_TRAJ_POINT_1 + 1);
        }
      }
      trajectory.points[1].positions[0] = 1.0;
    }
    else if(num == 2)
    {
      trajectory.points.resize(TRAJ_POINTS_2);
      for (std::size_t i = 0; i < TRAJ_POINTS_2; ++i)
      {
        trajectory.points[i].positions.resize(axis_names.size());
        trajectory.points[i].velocities.resize(axis_names.size());
        // for each joint
        for (std::size_t j = 0; j < axis_names.size(); ++j)
        {
          //trajectory.points[i].positions[j] = dRand(min_joint_value, max_joint_value);
          //if(j == 2)
            //trajectory.points[i].positions[j] = i%2==0 ? -0.5 : 0.5;
          //else
          trajectory.points[i].positions[j] = 0;
          trajectory.points[i].velocities[j] = 0;
          trajectory.points[i].time_from_start = ros::Duration(i * SEC_PER_TRAJ_POINT_2 + 1);
        }
      }
      trajectory.points[1].positions[0] = 0.25;
      trajectory.points[2].positions[0] = -0.25;
      trajectory.points[3].positions[0] = 0.25;
      trajectory.points[4].positions[0] = -0.25;
    }
/*
    trajectory.points[0].positions[0] = 0.0;
    trajectory.points[0].positions[1] = 0.5;
    trajectory.points[0].velocities[0] = 0.0;
    trajectory.points[0].velocities[1] = 0.0;

    trajectory.points[1].positions[0] = 0.5;
    trajectory.points[1].positions[1] = 0.0;
    trajectory.points[1].velocities[0] = 0.0;
    trajectory.points[1].velocities[1] = -0.7854;

    trajectory.points[2].positions[0] = 0.0;
    trajectory.points[2].positions[1] = -0.5;
    trajectory.points[2].velocities[0] = -0.7854;
    trajectory.points[2].velocities[1] = 0.0;

    trajectory.points[3].positions[0] = -0.5;
    trajectory.points[3].positions[1] = 0.0;
    trajectory.points[3].velocities[0] = 0.0;
    trajectory.points[3].velocities[1] = 0.7854;

    trajectory.points[4].positions[0] = 0.0;
    trajectory.points[4].positions[1] = 0.5;
    trajectory.points[4].velocities[0] = 0.7854;
    trajectory.points[4].velocities[1] = 0.0;

    trajectory.points[5].positions[0] = 0.0;
    trajectory.points[5].positions[1] = 0.0;
    trajectory.points[5].velocities[0] = 0.0;
    trajectory.points[5].velocities[1] = 0.0;
*/
    return trajectory;
  }

  /** \brief Get random number */
  double dRand(double dMin, double dMax)
  {
    double d = (double)rand() / RAND_MAX;
    return dMin + d * (dMax - dMin);
  }

private:
  // A shared node handle
  ros::NodeHandle nh_private_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestTrajectory> TestTrajectoryPtr;
typedef boost::shared_ptr<const TestTrajectory> TestTrajectoryConstPtr;

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ROS_INFO_STREAM_NAMED("test_trajectory", "Starting TestTrajectory...");

  ros::NodeHandle nodeHandle("~");

  pose_trajectory_merger::TestTrajectory testClient(nodeHandle);

  ROS_INFO_STREAM_NAMED("test_trajectory", "Shutting down.");
  ros::shutdown();

  return 0;
}
