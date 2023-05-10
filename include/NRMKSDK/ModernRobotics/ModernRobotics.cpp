#include "ModernRobotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include "../eigen/Eigen/Dense"
#include <cmath>
#include <vector>
#include <iostream>
# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace mr {
	so3 Identity3d=Eigen::Matrix3d::Identity();
	SE3 Identity4d=Eigen::Matrix4d::Identity();
	so3 Zero3d=Eigen::Matrix3d::Zero();
	
	SE3 MatrixExp6(const se3& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		so3 se3mat_cut = se3mat.block<3, 3>(0, 0);
		Vector3d omgtheta = so3ToVec(se3mat_cut);

		SE3 m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut<<1, 0 , 0 , 0 ,1 ,0 , 0 , 0 ,1;
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Identity3d* theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}
	}

	se3 VecTose3(const Vector6d& V) {
		// Separate angular (exponential representation) and linear velocities
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));
		// Fill in values to the appropriate parts of the transformation matrix
		SE3 m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
			0, 0, 0, 0;
		return m_ret;
	}
	so3 VecToso3(const Vector3d& omg) {
		so3 m_ret;
		m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
		return m_ret;
	}

	Vector3d so3ToVec(const so3& so3mat) {
		Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}
	Vector6d se3ToVec(const se3& T) {
		Vector6d m_ret(6);
		m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

		return m_ret;
	}

	Vector4d AxisAng3(const Vector3d& expc3) {
		Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

	bool NearZero(const double val) {
		return (std::abs(val) < .000001);
	}
	SO3 MatrixExp3(const so3& so3mat) {
		Vector3d omgtheta = so3ToVec(so3mat);

		SO3 m_ret = Identity3d;
		if (NearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}	
	SE3 TransInv(const SE3& transform) {
		SO3 R = TransToR(transform);
		Vector3d p = TransToP(transform);
		SO3 Rt = R.transpose();
		Vector3d t = -(Rt * p);
		SE3 inv;
		inv <<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}
	Vector3d Normalize(Vector3d V) {
		V.normalize();
		return V;
	}
	SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = Blist;
		SE3 T = Identity4d;
		Vector6d bListTemp;
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}

	pinvJacobian pinvJacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = JacobianBody(Blist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Jb ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * std::max(Jb.cols(), Jb.rows()) *svd.singularValues().array().abs()(0);
 		pinvJacobian pinv_relJb =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJb;
	}
	pinvJacobian pinvJacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = JacobianSpace(Slist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Js ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * std::max(Js.cols(), Js.rows()) *svd.singularValues().array().abs()(0);
 		pinvJacobian pinv_relJs =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJs;
	}	
	Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = Slist;	
		SE3 T = Identity4d;
		Vector6d sListTemp;
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}
		return Js;
	}	
	SO3 TransToR(const SE3& T) {
		Matrix3d R_ret;
		R_ret<< T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2);
		return R_ret;
	}	
	Vector3d TransToP(const SE3& T) {
		Vector3d p_ret;
		p_ret<<T(0, 3), T(1, 3), T(2, 3);
		return p_ret;
	}	
	Matrix6d Adjoint(const SE3& T) {
		SO3 R = TransToR(T);
		Vector3d p = TransToP(T);
		Matrix6d ad_ret(6, 6);
		Matrix6d ZeroMat6d;
		ZeroMat6d<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
		ad_ret = ZeroMat6d;
		ad_ret << R, Zero3d,
			VecToso3(p) * R, R;
		return ad_ret;
	}
	Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X){
		Vector6d vec=se3ToVec(MatrixLog6(TransInv(X)*Xd));
		return vec;
	}

	so3 MatrixLog3(const SO3& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		so3 m_ret =Zero3d;	
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}	
	se3 MatrixLog6(const SE3& T) {
		se3 m_ret(4, 4);
		SO3 R = TransToR(T);
		Vector3d p = TransToP(T);
		so3 omgmat = MatrixLog3(R);

		if (NearZero(omgmat.norm())) {
			m_ret << Zero3d, p,
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((R.trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Identity3d - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*p,
				0, 0, 0, 0;
		}
		return m_ret;
	}	

}
