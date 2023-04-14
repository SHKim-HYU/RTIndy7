/*
 * robot.h
 *
 *  Created on: Oct 23, 2017
 *      Author: root
 */

#ifndef SERIALROBOT_H_
#define SERIALROBOT_H_

#include <cmath>

#include <Eigen/Dense>
#include "LieDynamics.h"
#include "PoEKinematics.h"
#include "PropertyDefinition.h"

#define LEFT_ARM 0
#define RIGHT_ARM 1
#define MAX_CURRENT_ELMO 1200

using namespace Eigen;
typedef Matrix<float, ROBOT_DOF, 1> Jointf;
typedef Matrix<float, 6, 1> Taskf;
typedef Matrix<float, 3, 1> Cartecianf;
typedef Matrix<float, 3, 1> Orientationf;
typedef Matrix<float, 3, 1> Axisf;

typedef struct MOTOR_INFO{
    float toq_const[ROBOT_DOF];
    float gear_ratio[ROBOT_DOF];
    float rate_current[ROBOT_DOF];
}MotorInfo;

#define RADtoDEG 180/M_PI
#define DEGtoRAD M_PI/180

#define sign(a) (((a)<0) ? -1 : ((a)>0))

typedef struct STATE{
    float q[ROBOT_DOF];
    float q_dot[ROBOT_DOF];
    float q_ddot[ROBOT_DOF];
    float torque[ROBOT_DOF];
    float dq[ROBOT_DOF];
    float dq_dot[ROBOT_DOF];
    float dq_ddot[ROBOT_DOF];

	Jointf j_q;
	Jointf j_q_d;
	Jointf j_q_dd;
	Jointf j_torque;

	Taskf x;                           //Task space
	Taskf x_dot;
	Taskf x_ddot;
    float s_time;
}state;

class robot //: public HYUMotionDynamics::Liedynamics
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	robot();
	virtual ~robot();

protected:
    Vector3f w[6], p[6], L[6], CoM[6];
    Matrix3f inertia[6];
    float mass[6];

public:

    void robot_update_R(void);
	void motor_update_R(MOTOR_INFO *motor);
	void ENCtoRAD_R(int *enc, Jointf& q);
	void ENCtoRAD_R(int *enc, Jointf& q, Jointf& q_dot, float s_time);
	void ELMO_OUTPUT_R(MOTOR_INFO *motor, Jointf& torque, short *output);

	HYUMotionDynamics::Liedynamics *pDyn;
	HYUMotionKinematics::PoEKinematics *pCoM;
	HYUMotionKinematics::PoEKinematics *pKin;


private:
	Jointf q_p;
};

#endif /* SERIALROBOT_H_ */
