/*
 * LieOperator.cpp
 *
 *  Created on: Aug 13, 2018
 *      Author: spec
 */

#include "LieOperator.h"

LieOperator::LieOperator() {

}

LieOperator::~LieOperator() {

}

bool LieOperator::NearZero(double near){
		if (near < 0.000001)
			return true;
		else
			return false;
}

SE3 LieOperator::inverse_SE3( SE3 _SE3 )
{
	SE3 res;
	res << _SE3.block<3, 3>(0, 0).transpose(), -_SE3.block<3, 3>(0, 0).transpose()*_SE3.block<3, 1>(0, 3),
		0, 0, 0, 1;
	return res;
}

Matrix3d LieOperator::SkewMatrix(Vector3d _Vec3)
{
	Matrix3d res;
	res << 	0, 			-_Vec3(2), 	_Vec3(1),
			_Vec3(2), 	0, 			-_Vec3(0),
			-_Vec3(1), 	_Vec3(0), 	0;
	return res;
}

Matrix3d LieOperator::SkewMatrixSquare(Vector3d _Vec3)
{
	Matrix3d res;
	res << -_Vec3(2)*_Vec3(2)-_Vec3(1)*_Vec3(1), 	_Vec3(0)*_Vec3(1), 						_Vec3(0)*_Vec3(2),
			_Vec3(0)*_Vec3(1), 						-_Vec3(0)*_Vec3(0)-_Vec3(2)*_Vec3(2), 	_Vec3(1)*_Vec3(2),
			_Vec3(0)*_Vec3(2), 						_Vec3(1)*_Vec3(2), 						-_Vec3(0)*_Vec3(0)-_Vec3(1)*_Vec3(1);
	return res;
}

Matrix6d LieOperator::AdjointMatrix( SE3 _SE3 )
{
	Matrix6d res;
	res.setZero();

	res.block<3,3>(0,0) = _SE3.block(0, 0, 3, 3);
	res.block<3,3>(3,3) = _SE3.block(0, 0, 3, 3);
	res.block<3,3>(3,0) = SkewMatrix(_SE3.block(0, 3, 3, 1))*_SE3.block(0, 0, 3, 3);

	return res;
}

Matrix6d LieOperator::AdjointDualMatrix(SE3 _SE3)
{
	Matrix6d res;
	res.setZero();

	res.block(0, 0, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
	res.block(0, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose()*SkewMatrix(_SE3.block(0, 3, 3, 1)).transpose();
	res.block(3, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
	return res;
}

Matrix6d LieOperator::adjointMatrix(Twist _Twist)
{
	Matrix6d res;
	res.setZero();

	res.block(0, 0, 3, 3) = SkewMatrix(_Twist.head(3));
	res.block(3, 0, 3, 3) = SkewMatrix(_Twist.tail(3));
	res.block(3, 3, 3, 3) = SkewMatrix(_Twist.head(3));
	return res;
}

Matrix6d LieOperator::adjointDualMatrix(Twist _Twist)
{
	Matrix6d res;
	res.setZero();

	res.block(0, 0, 3, 3) = -SkewMatrix(_Twist.head(3));
	res.block(0, 3, 3, 3) = -SkewMatrix(_Twist.tail(3));
	res.block(3, 3, 3, 3) = -SkewMatrix(_Twist.head(3));

	return res;
}

SE3 LieOperator::SE3Matrix(Twist _Twist, double _q)
{
	Matrix4d res = Matrix4d::Identity();
	Matrix3d i = Matrix3d::Identity();

	//Rodrigues' formula
	res.block(0, 0, 3, 3) = i + sin(_q)*SkewMatrix(_Twist.head(3)) + (1 - cos(_q))*SkewMatrixSquare(_Twist.head(3));
	res.block(0, 3, 3, 1) = (i*_q + (1 - cos(_q))*SkewMatrix(_Twist.head(3))
			+ (_q - sin(_q))*SkewMatrixSquare(_Twist.head(3)))*_Twist.tail(3);

	return res;
}
Matrix3d LieOperator::MatrixLog3(Matrix3d & R)
{
    double acosinput = (R.trace() - 1) / 2.0;
    MatrixXd m_ret;
    m_ret.resize(3,3);
    m_ret.setZero();
    if (acosinput >= 1)
        return m_ret;
    else if (acosinput <= -1) {
        Vector3d omg;
        if (!NearZero(1 + R(2, 2)))
            omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
        else if (!NearZero(1 + R(1, 1)))
            omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
        else
            omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
        m_ret = SkewMatrix(M_PI*omg);
        return m_ret;
    }
    else {
        double theta = std::acos(acosinput);
        m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
        return m_ret;
    }
}
Vector3d LieOperator::so3ToVec(Matrix3d& so3mat) {
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return v_ret;
}
/*
Matrix<double, 7, 1> LieOperator::AxisAng6(Twist expc6) {
	Matrix<double, 7, 1> S_theta;
	Vector3d omg_theta,vec_theta;
	double theta;

	omg_theta << expc6(0,0), expc6(1,0), expc6(2,0);
	vec_theta << expc6(3, 0), expc6(4, 0), expc6(5, 0);

	theta = omg_theta.norm();

	if (NearZero(theta)) {
		theta = vec_theta.norm();
		S_theta << expc6, theta;
		return S_theta;
	}
	else {
		S_theta << expc6 / theta, theta;
		return S_theta;
	}
}
Matrix4d LieOperator::MatrixExp6(Twist _s, double _q) {
	Matrix3d so3, SO3;
	Matrix4d SE3;
	Vector3d v;
	double theta;


	so3 << SkewMatrix(_s.head(3));
	v << _s.tail(3);
	theta = _q;
	SO3 << SO3.setIdentity() + sin(theta)*so3 + (1 - cos(theta))*so3*so3;

	if (NearZero(_s.head(3).norm()) && _s.tail(3).norm() == 1) {
		SE3 << Matrix3d::Identity(), v*theta, 0, 0, 0, 1;
		return SE3;
	}
	else {
		SE3 << SO3, (Matrix3d::Identity()*theta + (1 - cos(theta))*so3 + (theta - sin(theta))*so3*so3)*v,
			0, 0, 0, 1;
		return SE3;
	}
}
*/