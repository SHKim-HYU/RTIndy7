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
	void SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration);
	void SetPolynomial5th_j(int NumJoint, state *act, double FinalPos, double InitTime, double Duration, double *q_);
	void SetPolynomial5th_t(int NumJoint, state *act, double FinalPos, double InitTime, double Duration, double *q_);

	void SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration, double *q_);

    double Polynomial5th(int NumJoint, double CurrentTime, int *Flag);
    double Polynomial5th(int NumJoint, double CurrentTime, int *Flag, double *q_);


private:
	int m_isReady[6];

	Eigen::Matrix<double, 6, 6> m_cof;

    double TrajDuration[10];
    double TrajTime[10];
    double TrajInitTime[10];
	Eigen::Matrix<double, 6, 1> StateVec[10], Coefficient[10];
    double dq, dq_dot, dq_ddot;
};

} /* namespace HYUDA */

#endif /* CONTROL_TRAJECTORY_H_ */
