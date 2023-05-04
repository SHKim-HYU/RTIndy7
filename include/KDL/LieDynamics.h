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

typedef Matrix<double, 6, 1> se3;

typedef Matrix<double, 4, 4> SE3;
typedef Matrix<double, 6, 6> Adjoint;
typedef Matrix<double, 6, 6> adjoint;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, ROBOT_DOF, ROBOT_DOF> Matrixd;
typedef Matrix<double, 6*ROBOT_DOF, 6*ROBOT_DOF> Matrix6n6nd;
typedef Matrix<double, 6*ROBOT_DOF, ROBOT_DOF> Matrix6nnd;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6 * ROBOT_DOF, 1> Vector6nd;

typedef Matrix<double, ROBOT_DOF, 1> Jointd;
typedef Matrix<double, 6, 1> Taskd;
typedef Matrix<double, 3, 1> Carteciand;
typedef Matrix<double, 3, 1> Orientationd;
typedef Matrix<double, 3, 1> Axisd;
typedef Matrix<Vector3d, 6, 1> Axisv;
typedef Matrix<Matrix3d, 6, 1> Axism;

typedef Matrix<std::complex<double>,Dynamic,Dynamic> MatrixXcd;

namespace HYUMotionDynamics{


class Liedynamics : public HYUMotionBase::LieOperator //: public HYUMotionKinematics::PoEKinematics
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Liedynamics();
	~Liedynamics();
    Liedynamics(HYUMotionKinematics::PoEKinematics &_PoEKin, HYUMotionKinematics::PoEKinematics &_PoECoM);

protected:

	Matrix3d Iner[ROBOT_DOF+1];
    double mass[ROBOT_DOF+1];
	Matrix6d GIner[ROBOT_DOF+1];

	Matrix3d i;
	Matrix3d zero3;
	Matrix6d i6, zero6;

private:

	Matrix6n6nd Gamma_mat;
	Matrix6nnd A_mat, LA_mat;
	Matrix6n6nd L_mat;
	Matrix6n6nd Iner_mat;
	Matrix6n6nd ad_Aqd;
	Matrix6n6nd ad_V;
	Vector6nd V;

	HYUMotionKinematics::PoEKinematics *pCoM;
	HYUMotionKinematics::PoEKinematics *pLink; //for Jacobian of Task Space Control

	int isFirstRun;

public:
	void UpdateDynamicInfo(Matrix3d inertia, double mass, int link_num );
	Matrix6d GeneralizedInertia(Matrix3d inertia, double mass);

	Matrix6n6nd Inertia_Link(void);
	Matrix6n6nd ad_Aqdot_Link(double qdot[]);
	Matrix6n6nd Gamma_Link(void);
	Matrix6n6nd L_link(void);
	Matrix6nnd A_Link(void);
	Matrix6n6nd ad_V_Link(double qdot[]);
	Vector6nd Vdot_base(int axis);

	void Prepare_Dynamics(double q[], double qdot[]);

	Matrixd M_Matrix(void);
	Matrixd M_verify;


	Matrixd C_Matrix(void);
	Matrixd C_out(void);
	Matrixd C_verify;

	Jointd G_Matrix(void);
	Jointd G_verify;
	Jointd q_d;

    void Mdot_Matrix( MatrixXd &_Mdot );

	Matrix6nnd noz;
	Matrix6n6nd nos;
};
}


