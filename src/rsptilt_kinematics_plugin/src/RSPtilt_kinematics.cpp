/*
 * RSPtilt_kinematics.cpp
 *
 *  Created on: 31-Aug-2016
 *      Author: jlane
 */

#include <pluginlib/class_list_macros.h>
#include <rsptilt_kinematics_plugin/RSPtilt_kinematics.h>
#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(rsptilt_kinematics::Kinematics,
        kinematics_base::KinematicsPlugin);

using namespace rsptilt_kinematics;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double Kinematics::ALMOST_PLUS_ONE = 0.9999999;
const double Kinematics::ALMOST_MINUS_ONE = -0.9999999;

/* radius of base joints; 0 index for even joints, 1 index for odd joints */
const double Kinematics::baseRadius[2] = {
          0.0402,
          0.0622
          };

/* radius of platform joints; 0 index for even joints, 1 index for odd joints */
const double Kinematics::platformRadius[2] = {
          0.0408,
          0.0408
          };

/* angle from x axis of base joints */
const double Kinematics::baseAngles[6] = {
          38.43,
          120.48,
          158.43,
          240.48,
          278.43,
          0.48
          };

/* angle from x axis of platform joints */
const double Kinematics::platformAngles[6] = {
          74.32,
          105.68,
          194.32,
          225.68,
          314.32,
          345.68
          };

/* angle from x axis of servo horn planes */
const double Kinematics::beta[6] = {
          90.0,
          270.0,
          210.0,
          30.0,
          330.0,
          150.0
          };

const double Kinematics::initialHeight =   0.0980;
const double Kinematics::hornLength =    0.0318;
const double Kinematics::legLength =     0.1016;
const double Kinematics::lambda = 12.0; //angle of tilt of servos

std::stringstream sstr;

//InverseKinematics::InverseKinematics()
//{
//}

Kinematics::~Kinematics()
{
}

bool Kinematics::initialize(
        const std::vector<double> &min_angles,
        const std::vector<double> &max_angles)
{
    min_angles_ = min_angles;
    max_angles_ = max_angles;

  /* calculate base and platform joint positions wrt their own frames */
  for(unsigned int i = 0; i < 6; i++) {
    b[i].x(baseRadius[i%2]*cos(DEG_TO_RAD(baseAngles[i])));
    b[i].y(baseRadius[i%2]*sin(DEG_TO_RAD(baseAngles[i])));
    b[i].z(0);
    //ROS_DEBUG("Base Positions: (%f, %f, %f)", b[i].x(), b[i].y(), b[i].z());
    p[i].x(platformRadius[i%2]*cos(DEG_TO_RAD(platformAngles[i])));
    p[i].y(platformRadius[i%2]*sin(DEG_TO_RAD(platformAngles[i])));
    p[i].z(0);
    //ROS_DEBUG("Platform Positions: (%f, %f, %f)", p[i].x(), p[i].y(), p[i].z());
	}
  return true;
}

/*
int InverseKinematics::CartToJnt(const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        std::vector<KDL::JntArray> &q_out)
{
    KDL::JntArray solution;
    bool bools[] = { true, false };

    // there are no solutions available yet
    q_out.clear();

    // iterate over all redundant solutions
    solution = ik(p_in);
    if (isSolutionValid(solution)) q_out.push_back(solution);

    if (q_out.size() > 0) {
        logger_.write("Inverse kinematics found a solution",
                __FILE__, __LINE__);

        return 1;
    } else {
        logger_.write("Inverse kinematics found no solution",
                __FILE__, __LINE__);

        return -1;
    }
}KDL::Frame& g0
*/
int Kinematics::getIK(std::vector<double> &pose, std::vector<double> &joints)
{
	if(joints.size() != 7)
		return -1;
  if(pose.size() != 6)
    return -1;

	KDL::Vector q, l;

	//std::vector<double> joints;
	//joints.resize(7);

	double R, P, Y, lMag, L, M, N;

	KDL::Vector pos(pose[0],pose[1],pose[2]);
  R = pose[3];
  P = pose[4];
  Y = pose[5];

  sstr << "Goal Position is: "
  		<< pos.x() << ","
  		<< pos.y() << ","
		<< pos.z()
		<< std::endl;
  //ROS_INFO("Goal Position: (%f, %f, %f)", pos.x(), pos.y(), pos.z());


    sstr << "RPY is: "
        		<< R << ","
        		<< P << ","
    			<< Y
    			<< std::endl;
  //ROS_INFO("Goal Rotation: (%f, %f, %f)", R, P, Y);


	for(unsigned int i = 0; i < 6; i++)
	{
		//calculate q vector, platform joints wrt base coord system
		q.x(p[i].x()*cos(P) + p[i].y()*sin(P)*sin(R) + pos.x());
		q.y(p[i].y()*cos(R) + pos.y());
		q.z(-p[i].x()*sin(P) + p[i].y()*cos(P)*sin(R) + pos.z() + initialHeight);
    //ROS_DEBUG("q[%d] = (%f, %f, %f)", i, q.x(), q.y(), q.z());
		//calculate virtual leg length
		l = q - b[i];
    //ROS_DEBUG("l[%d] = (%f, %f, %f)", i, l.x(), l.y(), l.z());
		lMag = l.Norm();
    //ROS_DEBUG("lMag[%d] = %f", i, lMag);

		L = (lMag*lMag) - (legLength*legLength) + (hornLength*hornLength);
    //ROS_DEBUG("L[%d] = %f", i, L);
		M = 2*hornLength*(-sin(DEG_TO_RAD(lambda))*sin(DEG_TO_RAD(beta[i]))*(q.x()-b[i].x()) + sin(DEG_TO_RAD(lambda))*cos(DEG_TO_RAD(beta[i]))*(q.y()-b[i].y()) + cos(DEG_TO_RAD(lambda))*(q.z()-b[i].z()));
    //ROS_DEBUG("M[%d] = %f", i, M);
		N = 2*hornLength*(cos(DEG_TO_RAD(beta[i]))*(q.x()-b[i].x()) + sin(DEG_TO_RAD(beta[i]))*(q.y()-b[i].y()));
    //ROS_DEBUG("N[%d] = %f", i, N);

    //ROS_DEBUG("sqrt(M^2 + N^2) = %f", sqrt(M*M+N*N));
    //ROS_DEBUG("ASIN = %f", asin(L/sqrt(M*M+N*N)));
    //ROS_DEBUG("ATAN = %f", atan2(N,M));
		joints[i] = asin(L/sqrt(M*M+N*N)) - atan2(N,M);
	}
	//write yaw to joint6
	joints[6] = Y;
  //ROS_DEBUG("IK Joint Solution");
  //for(unsigned int i = 0; i < 7; i++){
  //  ROS_DEBUG("Joint %d: %f", i, joints[i]);
  //}

	for(unsigned int i = 0; i < 7; i++)
	{
		if(joints[i] < -M_PI_2 || joints[i] > M_PI_2){
      ROS_DEBUG("Joint %d out of bounds",i);
			return -1;
    }
	}

	//log solution
	sstr << "j0: " << joints[0] << std::endl ;
	sstr << "j1: " << joints[1] << std::endl ;
	sstr << "j2: " << joints[2] << std::endl ;
	sstr << "j3: " << joints[3] << std::endl ;
	sstr << "j4: " << joints[4] << std::endl ;
	sstr << "j5: " << joints[5] << std::endl ;
	sstr << "j6: " << joints[6] << std::endl ;

/*	KDL::JntArray solution(5);
	solution(0) = j1;
	solution(1) = j2;
	solution(2) = j3;
	solution(3) = j4;
	solution(4) = j5;
*/
//	logger_.write(sstr.str(), __FILE__, __LINE__);

	return 1;


}
/*
bool InverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
    bool valid = true;

    if (solution.rows() != 5) return false;

    for (unsigned int i = 0; i < solution.rows(); i++) {
        if ((solution(i) < min_angles_[i]) || (solution(i) > max_angles_[i])) {
            valid = false;
        }
    }

    return valid;
}
*/


