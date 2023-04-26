/*
 * Trajectory.h
 *
 *  Created on: Nov 20, 2018
 *      Author: spec
 */

#ifndef CONTROL_TRAJECTORY_H_
#define CONTROL_TRAJECTORY_H_

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "../KDL/SerialRobot.h"

using namespace Eigen;
namespace hyuCtrl {



class Trajectory {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Trajectory();
	virtual ~Trajectory();
	bool isReady(){return m_isReady;};

	int isReady(int NumJoint){return m_isReady[NumJoint];};
	void SetPolynomial5th(int NumJoint, float startPos, float FinalPos, float InitTime, float Duration);
	void SetPolynomial5th_j(int NumJoint, state *act, float FinalPos, float InitTime, float Duration, float *q_, int traj_changed);
	void SetPolynomial5th_t(int NumJoint, state *act, float FinalPos, float InitTime, float Duration, float *q_);

	void SetPolynomial5th(int NumJoint, float startPos, float FinalPos, float InitTime, float Duration, float *q_);

    float Polynomial5th(int NumJoint, float CurrentTime, int *Flag);
    float Polynomial5th(int NumJoint, float CurrentTime, int *Flag, float *q_);


private:
	int m_isReady[6];

	Eigen::Matrix<float, 6, 6> m_cof;

    float TrajDuration[10];
    float TrajTime[10];
    float TrajInitTime[10];
	Eigen::Matrix<float, 6, 1> StateVec[10], Coefficient[10];
    float dq, dq_dot, dq_ddot;
};

} /* namespace HYUDA */

#endif /* CONTROL_TRAJECTORY_H_ */
