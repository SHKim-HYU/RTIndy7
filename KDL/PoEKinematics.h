/*
 * Kinematics.h
 *
 *  Created on: Aug 7, 2018
 *      Author: spec
 */

#ifndef POEKINEMATICS_H_
#define POEKINEMATICS_H_

#include <cmath>
#include <Eigen/Dense>

#include "LieOperator.h"
#include "PropertyDefinition.h"
#include "iostream"

using namespace Eigen;

typedef Matrix<float, ROBOT_DOF, 1> Jointd;
typedef Matrix<float, 6, ROBOT_DOF> Jaco;
typedef Matrix<float,ROBOT_DOF,6> InvJaco;
typedef Matrix<float,3,ROBOT_DOF> LinJaco;
typedef Matrix<float,ROBOT_DOF,3>  PinvLJaco;


namespace HYUMotionKinematics {

class PoEKinematics : public HYUMotionBase::LieOperator {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PoEKinematics();
	PoEKinematics(int DoF);
	virtual ~PoEKinematics();

public:
	void UpdateKinematicInfo(Vector3f _w, Vector3f _p, Vector3f _l, int _link_num);
	void UpdateKinematicInfo_R(Vector3f _w, Vector3f _p, Vector3f _l, int _link_num);
	Vector3f GetV(Vector3f _w, Vector3f _p);
	SE3 GetM(Vector3f _link);
	se3 GetTwist(Vector3f _w, Vector3f _v);
	void HTransMatrix(float q[]);
	se3 Twistout(int i);

	Jaco SpaceJacobian(void);
	Jaco BodyJacobian(void);
	Jaco AnalyticJacobian();
	//Matrix<float, 3, ROBOT_DOF> AnalyticJacobian(void);
    LinJaco LinearJacobian(void);

	InvJaco Pinv(Jaco _j);
//	Matrix<float,6,3> Pinv(Matrix<float,3,6> _j);
	PinvLJaco Pinv(LinJaco _j);
	PinvLJaco DPI(LinJaco _j);
	InvJaco DPI(Jaco _j);

	Vector3f ForwardKinematics(void);
	Vector3f GetEulerAngle(void);
	Vector4f GetQuaternion(void);
	Matrix3f Rot(void);
	SE3 GetTMat(int _i, int _j);
	float Manipulability(LinJaco _J);
	float Manipulability(Jaco _J);
	float Condition_Number(LinJaco _J);
	LinJaco Jacobian_l_dot();
//	Vector4f InverseKinematics(Matrix4f _td, )
	void Unflag_isInfoupdate();


    se3 A[ROBOT_DOF+1];
private:
	int RobotDoF;
	Jaco Jacobian;
	Jaco _SpaceJacobian;
	Jaco _BodyJacobian;
    Jaco _AnalyticJacobian;
	LinJaco linjacobian;
	LinJaco linjacobian_old;
	PinvLJaco PinvLinJaco;
	LinJaco _LinJaco_dot;

	Matrix<float,4,4> _BaseT;
	Matrix<float,4,4> _EndT;
    se3 twist;
    float w, k;
    Vector4f quat;

    VectorXf q_;




protected:
	SE3 T[ROBOT_DOF+1][ROBOT_DOF+1];
	SE3 M[ROBOT_DOF+1];
	SE3 _M[ROBOT_DOF+1][ROBOT_DOF+1];
	SE3 Exp_S[ROBOT_DOF+1];
	Matrix3f RotMat;


	se3 v_se3[ROBOT_DOF+1];

	int isInfoUpdated;
};

} /* namespace HYUSpesA */

#endif /* POEKINEMATICS_H_ */
