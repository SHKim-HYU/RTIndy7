#pragma once

#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;
#include "PropertyDefinition.h"
#include "PoEKinematics.h"



#include <iostream>
using namespace std;
using namespace Eigen;
#define _RotX 1
#define _RotY 2
#define _RotZ 3
#define gravity 9.8

typedef Matrix<float, 6, 1> se3;

typedef Matrix<float, 4, 4> SE3;
typedef Matrix<float, 6, 6> Adjoint;
typedef Matrix<float, 6, 6> adjoint;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<float, ROBOT_DOF, ROBOT_DOF> Matrixf;
typedef Matrix<float, 6*ROBOT_DOF, 6*ROBOT_DOF> Matrix6n6nf;
typedef Matrix<float, 6*ROBOT_DOF, ROBOT_DOF> Matrix6nnf;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6 * ROBOT_DOF, 1> Vector6nf;

typedef Matrix<float, ROBOT_DOF, 1> Jointf;
typedef Matrix<float, 6, 1> Taskf;
typedef Matrix<float, 3, 1> Cartecianf;
typedef Matrix<float, 3, 1> Orientationf;
typedef Matrix<float, 3, 1> Axisf;
typedef Matrix<Vector3f, 6, 1> Axisv;
typedef Matrix<Matrix3f, 6, 1> Axism;

typedef Matrix<std::complex<float>,Dynamic,Dynamic> MatrixXcd;

namespace HYUMotionDynamics{


class Liedynamics : public HYUMotionBase::LieOperator //: public HYUMotionKinematics::PoEKinematics
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Liedynamics();
	~Liedynamics();
    Liedynamics(HYUMotionKinematics::PoEKinematics &_PoEKin, HYUMotionKinematics::PoEKinematics &_PoECoM);

protected:

	Matrix3f Iner[ROBOT_DOF+1];
    float mass[ROBOT_DOF+1];
	Matrix6f GIner[ROBOT_DOF+1];

	Matrix3f i;
	Matrix3f zero3;
	Matrix6f i6, zero6;

private:

	Matrix6n6nf Gamma_mat;
	Matrix6nnf A_mat, LA_mat;
	Matrix6n6nf L_mat;
	Matrix6n6nf Iner_mat;
	Matrix6n6nf ad_Aqd;
	Matrix6n6nf ad_V;
	Vector6nf V;

	HYUMotionKinematics::PoEKinematics *pCoM;
	HYUMotionKinematics::PoEKinematics *pLink; //for Jacobian of Task Space Control

	int isFirstRun;

public:
	void UpdateDynamicInfo(Matrix3f inertia, float mass, int link_num );
	Matrix6f GeneralizedInertia(Matrix3f inertia, float mass);

	Matrix6n6nf Inertia_Link(void);
	Matrix6n6nf ad_Aqdot_Link(float qdot[]);
	Matrix6n6nf Gamma_Link(void);
	Matrix6n6nf L_link(void);
	Matrix6nnf A_Link(void);
	Matrix6n6nf ad_V_Link(float qdot[]);
	Vector6nf Vdot_base(int axis);

	void Prepare_Dynamics(float q[], float qdot[]);

	Matrixf M_Matrix(void);
	Matrixf M_verify;


	Matrixf C_Matrix(void);
	Matrixf C_out(void);
	Matrixf C_verify;

	Jointf G_Matrix(void);
	Jointf G_verify;
	Jointf q_d;

    void Mdot_Matrix( MatrixXf &_Mdot );

	Matrix6nnf noz;
	Matrix6n6nf nos;
};
}


