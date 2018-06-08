#ifndef INCLUDE_RSPTILT_KINEMATICS_PLUGIN_KINEMATICS_BASE_H_
#define INCLUDE_RSPTILT_KINEMATICS_PLUGIN_KINEMATICS_BASE_H_

#include <kdl/frames.hpp>

namespace kinematics_base
{
  class KinematicsPlugin
  {
    public:
      virtual bool initialize(const std::vector<double> &min_angles,
      												const std::vector<double> &max_angles) = 0;
      virtual int getIK(std::vector<double> &pose, std::vector<double> &joints) = 0;
      virtual ~KinematicsPlugin(){}

    protected:
      KinematicsPlugin(){}
  };
};
#endif