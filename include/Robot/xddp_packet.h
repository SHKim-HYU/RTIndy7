/*! 
 *  @file xddp_packet.h
 *  @brief header for xddp packet definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Jun. 13. 2024
 *  @Comm
 */

#pragma once

#include "PropertyDefinition.h"

#define XDDP_PORT_CMD_VEL 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
#define XDDP_PORT_ODOM 1
#define XDDP_PORT_SIM 2
#define XDDP_PORT_ACT 3

namespace packet{

struct Vector3{
	double x;
	double y;
	double z;
};

struct Quaternion{
	double x;
	double y;
	double z;
	double w;
};

struct Pose{
	Vector3 position;
	Quaternion orientation;
};

struct Twist{
	Vector3 linear;
	Vector3 angular;
};

struct Odometry{
	Pose pose;
	Twist twist;	
};

struct JointState{
	double position[ROBOT_DOF];
	double velocity[ROBOT_DOF];
	double effort[ROBOT_DOF];
};
}