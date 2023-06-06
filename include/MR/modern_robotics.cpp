#include "modern_robotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>

# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace mr {
	so3 Identity3d=Matrix3d::Identity();
	SE3 Identity4d=SE3::Identity();
	so3 Zero3d=Matrix3d::Zero();
	Matrix6d ad(const Vector6d& V) {
		Matrix6d result=Matrix6d::Zero();
		result<<    0,-V(2),  V(1), 0 ,0 ,0,
			     V(2),    0, -V(0),0 ,0 ,0,
				 -V(1),V(0),     0, 0,0 ,0,
				 0,    -V(5),   V(4), 0 ,-V(2) ,V(1),
				 V(5),    0,   -V(3), V(2) ,0 ,-V(0),
				 -V(4) ,V(3) ,0,-V(1),V(0) ,0;
		return result;
	}

	// SE3 MatrixExp6(const se3& se3mat) {
	// 	// Extract the angular velocity vector from the transformation matrix
	// 	so3 se3mat_cut = se3mat.block<3, 3>(0, 0);
	// 	Vector3d omgtheta = so3ToVec(se3mat_cut);

	// 	SE3 m_ret(4, 4);

	// 	// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
	// 	//									[	0	 ,		1		   ]]
	// 	if (NearZero(omgtheta.norm())) {
	// 		// Reuse previous variables that have our required size
	// 		se3mat_cut<<1, 0 , 0 , 0 ,1 ,0 , 0 , 0 ,1;
	// 		omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
	// 		m_ret << se3mat_cut, omgtheta,
	// 			0, 0, 0, 1;
	// 		return m_ret;
	// 	}
	// 	// If not negligible, MR page 105
	// 	else {
	// 		double theta = (AxisAng3(omgtheta))(3);
	// 		Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
	// 		Eigen::Matrix3d expExpand = Identity3d* theta + (1 - cos(theta)) * omgmat + ((theta - sin(theta)) * (omgmat * omgmat));
	// 		Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
	// 		Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
	// 		m_ret << MatrixExp3(se3mat_cut), GThetaV,
	// 			0, 0, 0, 1;
	// 		return m_ret;
	// 	}
	// }
	SE3 MatrixExp6(const se3& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		so3 se3mat_cut = se3mat.block<3, 3>(0, 0);
		Vector3d omgtheta = so3ToVec(se3mat_cut);
		SE3 m_ret= SE3::Identity();
		double theta = omgtheta.norm();
		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			m_ret.block<3,3>(0,0) = Matrix3d::Identity();
			m_ret.block<3,1>(0,3) = Vector3d(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			Matrix3d omgmat = se3mat_cut / theta;
			Matrix3d omgmatomgmat;
			double w3 = omgmat(1,0);
			double w2 = omgmat(0,2);
			double w1 = omgmat(2,1);
			omgmatomgmat(0,0) = -w2*w2-w3*w3;
			omgmatomgmat(1,1) = -w1*w1-w3*w3;
			omgmatomgmat(2,2) = -w1*w1-w2*w2;			
			omgmatomgmat(0,1) = omgmatomgmat(1,0) = w1*w2;
			omgmatomgmat(0,2) = omgmatomgmat(2,0) =w1*w3;
			omgmatomgmat(1,2) = omgmatomgmat(2,1) =w2*w3;
			Matrix3d expExpand = Identity3d* theta + (1 - cos(theta)) * omgmat + ((theta - sin(theta)) * (omgmatomgmat));
			Vector3d linear(se3mat(0, 3)/theta, se3mat(1, 3)/theta, se3mat(2, 3)/theta);
			Vector3d GThetaV(expExpand(0,0)*linear(0)+expExpand(0,1)*linear(1)+expExpand(0,2)*linear(2),
			expExpand(1,0)*linear(0)+expExpand(1,1)*linear(1)+expExpand(1,2)*linear(2),
			expExpand(2,0)*linear(0)+expExpand(2,1)*linear(1)+expExpand(2,2)*linear(2));
			m_ret.block<3,3>(0,0) = Identity3d + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
			m_ret.block<3,1>(0,3) = GThetaV;
			return m_ret;
		}
	}
	se3 VecTose3(const Vector6d& V) {
		// Separate angular (exponential representation) and linear velocities
		// Fill in values to the appropriate parts of the transformation matrix
		SE3 m_ret;
		m_ret << 0,-V(2),V(1),V(3),
		         V(2),0,-V(0),V(4),
				-V(1),V(0),0,V(5),
				 0 ,0 ,0,1;
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
		Vector3d v_ret(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
		return v_ret;
	}
	Vector6d se3ToVec(const se3& T) {
		Vector6d m_ret;
		m_ret<<T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);
		return m_ret;
	}

	Vector4d AxisAng3(const Vector3d& expc3) {
		Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

	bool NearZero(const double val) {
		return (abs(val) < .000001);
	}
	// SO3 MatrixExp3(const so3& so3mat) {
	// 	Vector3d omgtheta = so3ToVec(so3mat);
	// 	double theta = omgtheta.norm();
	// 	if (NearZero(so3mat.norm())) {
	// 		return Identity3d;
	// 	}
	// 	else {			
	// 		Matrix3d omgmat = so3mat * (1.0 / theta);
	// 		return Identity3d + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmat * omgmat));
	// 	}
	// }	
	SO3 MatrixExp3(const so3& so3mat) {
		Vector3d omgtheta = so3ToVec(so3mat);
		double theta = omgtheta.norm();
		if (NearZero(so3mat.norm())) {
			return Identity3d;
		}
		else {			
			Matrix3d omgmat = so3mat / theta;
			Matrix3d omgmatomgmat;
			double w3 = omgmat(1,0);
			double w2 = omgmat(0,2);
			double w1 = omgmat(2,1);
			omgmatomgmat(0,0) = -w2*w2-w3*w3;
			omgmatomgmat(1,1) = -w1*w1-w3*w3;
			omgmatomgmat(2,2) = -w1*w1-w2*w2;		
			omgmatomgmat(0,1) = omgmatomgmat(1,0) = w1*w2;
			omgmatomgmat(0,2) = omgmatomgmat(2,0) =w1*w3;			
			omgmatomgmat(1,2) = omgmatomgmat(2,1) =w2*w3;			
			return Identity3d + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
		}
	}		
	SE3 TransInv(const SE3& transform) {
		SO3 R = TransToR(transform);
		Vector3d p = TransToP(transform);
		SO3 Rt = R.transpose();
		Vector3d t = -(Rt * p);
		SE3 inv = SE3::Identity();
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		return inv;
	}
	Vector3d Normalize(Vector3d V) {
		V.normalize();
		return V;
	}
	SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = (JOINTNUM - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < JOINTNUM; i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}	
	
	// Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
	// 	Jacobian Jb = Blist;
	// 	SE3 T = Identity4d;
	// 	Vector6d bListTemp;
	// 	for (int i = JOINTNUM -2; i >= 0; i--) {
	// 		bListTemp << Blist.col(i + 1) * thetaList(i + 1);
	// 		T = T * MatrixExp6(VecTose3(-1 * bListTemp));
	// 		// cout << "array: " << sListTemp << endl;
	// 		Jb.col(i) = Adjoint(T) * Blist.col(i);
	// 	}
	// 	return Jb;
	// }

	Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = Blist;
		SE3 T = Identity4d;
		for (int i = JOINTNUM -2; i >= 0; i--) {
			T *= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * thetaList(i + 1)));
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}	
	Jacobian AnalyticJacobianBody(SE3 M, const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb =  JacobianBody(Blist,thetaList);
		SE3 Tsb = FKinBody(M,Blist, thetaList);
		SO3 Rsb = TransToR(Tsb);
		Matrix6d AdR = Matrix6d::Identity();
		AdR.block<3,3>(0,0) = Matrix3d::Identity();;
		AdR.block<3,3>(3,3) = Rsb;
		Jacobian Ja = AdR * Jb;  
		return Ja;
	}
	pinvJacobian pinvAnalyticJacobianBody(SE3 M, const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb =  AnalyticJacobianBody(M,Blist,thetaList);
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Jb ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * max(Jb.cols(), Jb.rows()) *svd.singularValues().array().abs()(0);
 		pinvJacobian pinv_relJb =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJb;
	}

	pinvJacobian pinvJacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = JacobianBody(Blist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Jb ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * max(Jb.cols(), Jb.rows()) *svd.singularValues().array().abs()(0);
 		pinvJacobian pinv_relJb =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJb;
	}
	pinvJacobian pinvJacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = JacobianSpace(Slist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Js ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * max(Js.cols(), Js.rows()) *svd.singularValues().array().abs()(0);
 		pinvJacobian pinv_relJs =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJs;
	}	
	Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = Slist;	
		SE3 T = Identity4d;
		for (int i = 1; i < JOINTNUM; i++) {
			T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetaList(i - 1)));
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}
		return Js;
	}	
	SO3 TransToR(const SE3& T) {
		return T.block<3,3>(0,0);
	}	
	Vector3d TransToP(const SE3& T) {
		return Vector3d(T(0,3), T(1,3),T(2,3));
	}	
	// Matrix6d Adjoint(const SE3& T) {
	// 	SO3 R = TransToR(T);
	// 	Vector3d p = TransToP(T);
	// 	Matrix6d ad_ret(6, 6);
	// 	Matrix6d ZeroMat6d;
	// 	ZeroMat6d<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	// 	ad_ret = ZeroMat6d;
	// 	ad_ret << R, Zero3d,
	// 		VecToso3(p) * R, R;
	// 	return ad_ret;
	// }
	Matrix6d Adjoint(const SE3& T) {
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret<<T(0,0),T(0,1),T(0,2),0,0,0,
		T(1,0),T(1,1),T(1,2),0,0,0,
		T(2,0),T(2,1),T(2,2),0,0,0,
		T(1,3)*T(2,0) - T(2,3)*T(1,0), T(1,3)*T(2,1) - T(2,3)*T(1,1), T(1,3)*T(2,2) - T(2,3)*T(1,2),T(0,0),T(0,1),T(0,2),
		T(2,3)*T(0,0) - T(0,3)*T(2,0), T(2,3)*T(0,1) - T(0,3)*T(2,1), T(2,3)*T(0,2) - T(0,3)*T(2,2),T(1,0),T(1,1),T(1,2),
		T(0,3)*T(1,0) - T(1,3)*T(0,0), T(0,3)*T(1,1) - T(1,3)*T(0,1), T(0,3)*T(1,2) - T(1,3)*T(0,2),T(2,0),T(2,1),T(2,2);
		return ad_ret;
	}	
	SO3 RotInv(const SO3& rotMatrix) {
		return rotMatrix.transpose();
	}
	Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X,double dt){
		Vector6d vec=se3ToVec(MatrixLog6(TransInv(X)*Xd))/dt;
		return vec;
	}
	Vector6d VelSE3toSE3(const SE3 &Xd,const SE3 &X,double dt){
		SO3 Rd = TransToR(Xd);
		Vector3d pd = TransToP(Xd);
		SO3 R = TransToR(X);
		Vector3d p = TransToP(X);

		Vector3d w = so3ToVec(MatrixLog3(R.transpose()*Rd))/dt;
		Vector3d v = (pd-p)/dt;

		Vector6d vec;
		vec<<w[0],w[1],w[2],v[0],v[1],v[2];
		return vec;
	}	

	Vector6d AccVeltoVel(const Vector6d &Vd,const Vector6d &V,double dt){
		Vector6d vec = (Vd-V)/dt;
		return vec;
	}
	SE3 RpToTrans(const Matrix3d& R, const Vector3d& p) {
		SE3 m_ret=SE3::Identity();
		m_ret.block<3,3>(0,0) = R;
		m_ret.block<3,1>(0,3) = p;
		return m_ret;
	}
	so3 MatrixLog3(const SO3& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		so3 m_ret =Zero3d;	
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / sqrt(2 * (1 + R(2, 2))))*Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / sqrt(2 * (1 + R(1, 1))))*Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / sqrt(2 * (1 + R(0, 0))))*Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = acos(acosinput);
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
			double theta = acos((R.trace() - 1) / 2.0);
			Matrix3d logExpand1 = Identity3d - omgmat / 2.0;
			Matrix3d logExpand2 = (1.0 / theta - 1.0 / tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*p,
				0, 0, 0, 0;
		}
		return m_ret;
	}	
	bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinBody(M, Blist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vb = se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vb(0), Vb(1), Vb(2));
		Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Jb;
		while (err && i < maxiterations) {
			Jb = JacobianBody(Blist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}
	bool IKinSpace(const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinSpace(M, Slist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vs(0), Vs(1), Vs(2));
		Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Js;
		while (err && i < maxiterations) {
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
									const vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    // the size of the lists
		int n = JOINTNUM;

		SE3 Mi = SE3::Identity();
		Matrix6xn Ai = Matrix6xn::Zero();
		vector<ScrewList> AdTi;
		for (int i = 0; i < n+1; i++) {
			AdTi.push_back(Matrix6d::Zero());
		}
		Matrix6xn_1 Vi = Matrix6xn_1::Zero();    // velocity
		Matrix6xn_1 Vdi = Matrix6xn_1::Zero();   // acceleration

		Vdi.block(3, 0, 3, 1) = - g;
		AdTi[n] = Adjoint(TransInv(Mlist[n]));
		Vector6d Fi = Ftip;

		JVec taulist = JVec::Zero();

		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = Adjoint(TransInv(Mi))*Slist.col(i);

			AdTi[i] = Adjoint(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
			          * TransInv(Mlist[i]));

			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}

		// backward pass
		for (int i = n-1; i >= 0; i--) {
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}

	JVec GravityForces(const JVec& thetalist, const Vector3d& g,
									const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    int n = JOINTNUM;
		JVec dummylist = JVec::Zero();
		Vector6d dummyForce = Vector6d::Zero();
		JVec grav = InverseDynamics(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist);
		return grav;
	}	

	MassMat MassMatrix(const JVec& thetalist,
                                const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = JOINTNUM;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero();
		MassMat M = MassMat::Zero();
		for (int i = 0; i < n; i++) {
			JVec ddthetalist = JVec::Zero();
			ddthetalist(i) = 1;
			M.col(i) = InverseDynamics(thetalist, dummylist, ddthetalist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		}
		return M;
	}

	JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = JOINTNUM;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero(6);
		JVec c = InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
	}	
	void JointTrajectory(const JVec q0, const JVec qT, double Tf, double t , int method , JVec& q_des, JVec& dq_des, JVec& ddq_des) {
		if(t>Tf)t = Tf;
		if(t<0) t= 0;
		double st;
		double dst;
		double ddst;
		if (method == 3){
			st = CubicTimeScaling(Tf, t);
			dst = CubicTimeScalingDot(Tf, t);
			ddst = CubicTimeScalingDdot(Tf, t);
		}
			
		else{
			st = QuinticTimeScaling(Tf, t);
			dst = QuinticTimeScalingDot(Tf, t);
			ddst = QuinticTimeScalingDdot(Tf, t);
		}
			
		q_des =st * qT + (1 - st)*q0;
		dq_des =dst * qT  - dst*q0;
		ddq_des =ddst * qT  - ddst*q0;
		
	}
	JVec EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = JOINTNUM;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		JVec JTFtip = InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}
	JVec ForwardDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
									const vector<Matrix6d>& Glist, const ScrewList& Slist) {

		JVec totalForce = taulist - VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
                 							 - GravityForces(thetalist, g, Mlist, Glist, Slist)
                                             - EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);
		MassMat M = MassMatrix(thetalist, Mlist, Glist, Slist);
		// Use LDLT since M is positive definite
	    Eigen::LDLT<MassMat> ldlt(M);
		Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(totalForce.data(), totalForce.size());
		Eigen::VectorXd x = ldlt.solve(v);
        // JVec ddthetalist = M.ldlt().solve(totalForce);
		JVec ddthetalist =  Eigen::Map<JVec>(x.data(), x.size());
		return ddthetalist;
	}	

	void EulerStep(JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt) {
		thetalist += dthetalist * dt;
		dthetalist += ddthetalist * dt;
		return;
	}
	JVec ComputedTorque(const JVec& thetalist, const JVec& dthetalist, const JVec& eint,
		const Vector3d& g, const vector<SE3>& Mlist, const vector<Matrix6d>& Glist,
		const ScrewList& Slist, const JVec& thetalistd, const JVec& dthetalistd, const JVec& ddthetalistd,
		double Kp, double Ki, double Kd) {

		JVec e = thetalistd - thetalist;  // position err
		JVec tau_feedforward = MassMatrix(thetalist, Mlist, Glist, Slist)*(Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

		Vector6d Ftip = Vector6d::Zero(6);
		JVec tau_inversedyn = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);
		JVec tau_computed = tau_feedforward + tau_inversedyn;
		return tau_computed;
	}	
	double CubicTimeScaling(double Tf, double t) {
		// a0 = a1 =0
		double a2 = 3.0/pow(Tf,2);
		double a3 = -2.0/pow(Tf,3);
		double s = a2*pow(t,2) + a3*pow(t,3);
		return s;
	}
	double CubicTimeScalingDot(double Tf, double t) {
		// a0 = a1 =0
		double a2 = 3.0/pow(Tf,2);
		double a3 = -2.0/pow(Tf,3);
		double ds = 2*a2*t + 3*a3*pow(t,2);
		return ds;
	}
	double CubicTimeScalingDdot(double Tf, double t) {
		// a0 = a1 =0
		double a2 = 3.0/pow(Tf,2);
		double a3 = -2.0/pow(Tf,3);
		double dds = 2*a2 + 6*a3*t;
		return dds;
	}	
	double QuinticTimeScaling(double Tf, double t) {          
		// a0 = a1 =a2=0
		double a3 = 10.0/pow(Tf,3);
		double a4 = -15.0/pow(Tf,4);
		double a5 = 6.0/pow(Tf,5);
		double s = a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
		return s;
	}
	double QuinticTimeScalingDot(double Tf, double t) {          
		// a0 = a1 =a2=0
		double a3 = 10.0/pow(Tf,3);
		double a4 = -15.0/pow(Tf,4);
		double a5 = 6.0/pow(Tf,5);
		double ds = 3*a3*pow(t,2)+4*a4*pow(t,3)+5*a5*pow(t,4);
		return ds;
	}		
	double QuinticTimeScalingDdot(double Tf, double t) {          
		// a0 = a1 =a2=0
		double a3 = 10.0/pow(Tf,3);
		double a4 = -15.0/pow(Tf,4);
		double a5 = 6.0/pow(Tf,5);
		double dds =3*2*a3*t+4*3*a4*pow(t,2)+5*4*a5*pow(t,3);
		return dds;
	}		
	Vector3d QuinticTimeScalingKinematics(double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) {          
		Vector3d s_ds_dds;
		Vector6d x;
		x[0] = s0;
		x[1] = ds0;
		x[2] = dds0/2.0;
		x[3] = -(10*s0 - 10*sT + 2*Tf*(3*ds0 + 2*dsT) + (pow(Tf,2)*(3*dds0 - ddsT))/2)/pow(Tf,3);
		x[4] =    (((3*dds0)/2 - ddsT)*pow(Tf,2) + (8*ds0 + 7*dsT)*Tf + 15*s0 - 15*sT)/pow(Tf,4);
		x[5] =         -(6*s0 - 6*sT + (pow(Tf,2)*(dds0 - ddsT))/2 + 3*Tf*(ds0 + dsT))/pow(Tf,5);

		s_ds_dds[0] = x[0]+x[1]*t+x[2]*pow(t,2)+x[3]*pow(t,3)+x[4]*pow(t,4)+x[5]*pow(t,5);
		s_ds_dds[1] = x[1]+2*x[2]*t+3*x[3]*pow(t,2)+4*x[4]*pow(t,3)+5*x[5]*pow(t,4);
		s_ds_dds[2] = 2*x[2]+2*3*x[3]*t+3*4*x[4]*pow(t,2)+4*5*x[5]*pow(t,3);

		return s_ds_dds;
	}	
	JVec Desired_q(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method) {
		JVec ret_thetalist = thetastart;
		double st;
		if (method == 3)
			st = CubicTimeScaling(Tf, t);
		else
			st = QuinticTimeScaling(Tf, t);
		ret_thetalist = st * thetaend + (1 - st)*thetastart;
		if(t>Tf)ret_thetalist = thetaend;
		return ret_thetalist;
	}	
	JVec Desired_dq(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method) {
		JVec ret_dthetalist = JVec::Zero();
		double dst;
		if (method == 3)
			dst = CubicTimeScalingDot(Tf, t);
		else
			dst = QuinticTimeScalingDot(Tf, t);
		ret_dthetalist = dst * thetaend +  - dst*thetastart;
		if(t>Tf)ret_dthetalist = JVec::Zero();
		return ret_dthetalist;
	}		
	JVec Desired_ddq(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method) {
		JVec ret_ddthetalist = JVec::Zero();
		double ddst;
		if (method == 3)
			ddst = CubicTimeScalingDdot(Tf, t);
		else
			ddst = QuinticTimeScalingDdot(Tf, t);
		ret_ddthetalist = ddst * thetaend +  - ddst*thetastart;
		if(t>Tf)ret_ddthetalist = JVec::Zero();
		return ret_ddthetalist;
	}		

	SE3 ScrewT(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method) {
		double st;
		
		if (method == 3)
			st = CubicTimeScaling(Tf, t);
		else
			st = QuinticTimeScaling(Tf, t);
		se3 Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
		SE3 retT = Xstart * MatrixExp6(Ttemp*st);
		if(t>Tf)retT = Xend;
		return retT;
	}		
	Vector6d ScrewTwist(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method) {
		double dst;
		if (method == 3)
			dst = CubicTimeScalingDot(Tf, t);
		else
			dst = QuinticTimeScalingDot(Tf, t);
		se3 Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
		Vector6d retTwist = se3ToVec(Ttemp*dst);
		if(t>Tf)retTwist = Vector6d::Zero();
		return retTwist;
	}
	Vector6d ScrewTwistDot(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method) {
		double ddst;
		if (method == 3)
			ddst = CubicTimeScalingDdot(Tf, t);
		else
			ddst = QuinticTimeScalingDdot(Tf, t);
		se3 Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
		Vector6d retTwistdot = se3ToVec(Ttemp*ddst);
		if(t>Tf)retTwistdot = Vector6d::Zero();

		return retTwistdot;
	}			

	SE3 CartesianT(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method) {
		SO3 Rstart = TransToR(Xstart);
		Vector3d pstart = TransToP(Xstart);
		SO3 Rend = TransToR(Xend);
		Vector3d pend = TransToP(Xend);
		double st;
		if (method == 3)
			st = CubicTimeScaling(Tf, t);
		else
			st = QuinticTimeScaling(Tf, t);
		SO3 Rs = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend)*st);
		Vector3d ps = st*pend + (1 - st)*pstart;
		SE3 retT = RpToTrans(Rs,ps);
		if(t>Tf)retT =Xend;
		return retT;
	}			
	Vector6d CartesianVel(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method) {
		SO3 Rstart = TransToR(Xstart);
		Vector3d pstart = TransToP(Xstart);
		SO3 Rend = TransToR(Xend);
		Vector3d pend = TransToP(Xend);
		double dst;
		if (method == 3)
			dst = CubicTimeScalingDot(Tf, t);
		else
			dst = QuinticTimeScalingDot(Tf, t);
		Vector3d omega_s = so3ToVec(MatrixLog3(Rstart.transpose() * Rend)*dst);
		Vector3d v_s = dst*pend - dst*pstart;
		Vector6d retVel;
		retVel<< omega_s[0], omega_s[1], omega_s[2],v_s[0],v_s[1],v_s[2];
		if(t>Tf)retVel = Vector6d::Zero();
		return retVel;
	}
	Vector6d CartesianAcc(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method) {
		SO3 Rstart = TransToR(Xstart);
		Vector3d pstart = TransToP(Xstart);
		SO3 Rend = TransToR(Xend);
		Vector3d pend = TransToP(Xend);
		double ddst;
		if (method == 3)
			ddst = CubicTimeScalingDdot(Tf, t);
		else
			ddst = QuinticTimeScalingDdot(Tf, t);
		Vector3d omegadot_s = so3ToVec(MatrixLog3(Rstart.transpose() * Rend)*ddst);
		Vector3d vdot_s = ddst*pend - ddst*pstart;
		Vector6d retAcc;
		retAcc<< omegadot_s[0], omegadot_s[1], omegadot_s[2],vdot_s[0],vdot_s[1],vdot_s[2];
		if(t>Tf)retAcc = Vector6d::Zero();
		return retAcc;
	}	
	void rotm2eulm(SO3 R,SO3& Rz,SO3& Ry,SO3& Rx){
		 Vector3d e_a = R.eulerAngles(2, 1, 0); // euler angles
		 if(e_a[0]<0.00000001)
		 	e_a<<e_a[0]-M_PI,e_a[1]-M_PI,e_a[2]-M_PI;
		 Rz<< cos(e_a[0]), -sin(e_a[0]),0,
		      sin(e_a[0]), cos(e_a[0]),0,
			  0,0,1;
		 Ry << cos(e_a[1]), 0 ,sin(e_a[1]),
		 	  0,1,0,
			  -sin(e_a[1]), 0 ,cos(e_a[1]);
		 Rx <<1 ,0,0,
		      0, cos(e_a[2]), -sin(e_a[2]),
			  0 , sin(e_a[2]) , cos(e_a[2]);
		 return;
	}
	so3 LieBracket(const so3& A,const so3& B){
		so3 ret = A*B - B*A;
		return ret;
	}
	double wrapToPI(double angle) {
		constexpr double TwoPi = 2 * M_PI;
		angle = fmod(angle + M_PI, TwoPi);
		if (angle < 0) {
			angle += TwoPi;
		}
		return angle - M_PI;
	}

	double wrapTo2PI(double angle) {
		constexpr double TwoPi = 2 * M_PI;
		angle = fmod(angle, TwoPi);
		if (angle < 0) {
			angle += TwoPi;
		}
		return angle;
	}
	JVec wrapTo2PI(const JVec& angles) {
		JVec retJ = JVec::Zero();
		for(int i = 0;i<JOINTNUM;i++){
			retJ(i) = wrapTo2PI(angles(i));
		}
		return retJ;
	}
	JVec wrapToPI(const JVec& angles) {
		JVec retJ = JVec::Zero();
		for(int i = 0;i<JOINTNUM;i++){
			retJ(i) = wrapToPI(angles(i));
		}
		return retJ;
	}	
	

	Vector6d CartesianError(const SE3& X,const SE3& Xd ){
		SO3 Rsb = TransToR(X);
		SO3 Rd = TransToR(Xd);
		Vector3d psb = TransToP(X);
		Vector3d pd = TransToP(Xd);
		Vector3d rot_err  = so3ToVec(MatrixLog3(Rsb.transpose()*Rd));
		Vector3d pos_err = pd-psb;
		Vector6d Xe;
		Xe<< rot_err[0],rot_err[1],rot_err[2],pos_err[0],pos_err[1],pos_err[2];
		return Xe;
	}


	DerivativeJacobianVec DerivativeVectorizeJacobianBody(const ScrewList& Blist, const JVec& thetaList){
		DerivativeJacobianVec dvecJ = DerivativeJacobianVec::Zero();
		Jacobian Jb = JacobianBody(Blist,thetaList);
		for(int i = 0;i<JOINTNUM-1;i++){
			Vector6d Jb_i = Jb.col(i) ;
			for(int j= i+1;j<JOINTNUM;j++){
				Vector6d Jb_j = Jb.col(j) ;
				Vector6d adJb_iJb_j = ad(Jb_i)*Jb_j;
				dvecJ.block(i*6,j,6,1) = adJb_iJb_j;
			}
		}
		return dvecJ;
	}
	Jacobian dJacobianBody(const Jacobian& Jb ,const JVec& dthetaList){
			Vector6d Ji,Jj,dJidt,aJiaqj;
			Jacobian dJb= Jacobian::Zero();
			for(int i = 0;i<JOINTNUM;i++){
				Ji = Jb.col(i);
				dJidt = Vector6d::Zero();
				for(int j =0;j<JOINTNUM;j++){
					Jj = Jb.col(j);
					aJiaqj = Vector6d::Zero();
					if(i<j){
						aJiaqj = ad(Ji)*Jj;
					}
					dJidt += aJiaqj*dthetaList[j];
				}
				dJb.col(i) = dJidt;
			}
			return dJb;
	}
	Jacobian dAnalyticJacobianBody(const SE3&M, const ScrewList& Blist, const JVec& thetaList ,const JVec& dthetaList){
			Jacobian Jb = JacobianBody(Blist,thetaList);
			Jacobian dJb = dJacobianBody(Jb,dthetaList);
			Jacobian dJa =Jacobian::Zero();
			SE3 Tsb = FKinBody(M,Blist,thetaList);
			SO3 Rsb = TransToR(Tsb);
			Vector6d Vb = Jb*dthetaList;
			Vector3d wb;
			wb<<Vb[0],Vb[1],Vb[2];

			Matrix6d Ad = Matrix6d::Identity();
			Ad.block<3,3>(3,3) = Rsb;

			Matrix6d dAd = Matrix6d::Zero();
			dAd.block<3,3>(3,3) = Rsb*VecToso3(wb);
			dJa = (dAd*Jb+Ad*dJb);
			return dJa;
	}

	DerivativeJacobianVec DerivativeVectorizeAnalyticJacobianBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList){
		DerivativeJacobianVec dvecJ = DerivativeJacobianVec::Zero();
		SE3 Tsb = FKinBody(M,Blist,thetaList);
		SO3 Rsb = TransToR(Tsb);

		Matrix6d AdRsb = Matrix6d::Identity();
		AdRsb.block<3,3>(0,0) = Matrix3d::Identity();		
		AdRsb.block<3,3>(3,3) = Rsb;		
		Jacobian Jb = JacobianBody(Blist,thetaList);
		Jacobian Ja = AdRsb*Jb;
		for(int i = 0;i<JOINTNUM-1;i++){
			Vector6d Ja_i = Ja.col(i) ;
			for(int j= i+1;j<JOINTNUM;j++){
				Vector6d Ja_j = Ja.col(j) ;
				Vector6d adJa_iJa_j = ad(Ja_i)*Ja_j;
				dvecJ.block(i*6,j,6,1) = adJa_iJa_j;
			}
		}
		return dvecJ;
	}
	vecJVec vec(const JVec& thetaList){
		vecJVec vecq = vecJVec::Zero();
		for(int i = 0;i<JOINTNUM;i++){
			for(int j = 0;j<6;j++){
				vecq(i*6+j) = thetaList(i);
			}
		}
		return vecq;
	}

	void FkinBody(SE3 M,ScrewList Blist, const JVec& q ,const JVec& dq, SE3 &T, Jacobian &Jb,Jacobian& dJb){
		Jb = Blist;
		dJb = Jacobian::Zero();
		T = SE3::Identity();
		SE3 T_ = SE3::Identity();
		Vector6d prev_dJidt=Blist.col(JOINTNUM-1)*dq(JOINTNUM-1);
		for(int i = JOINTNUM-2 ;i >= 0;i--){	
				T_*= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * q(i + 1)));
				Vector6d Jbi = Adjoint(T_)*Blist.col(i);
				Jb.col(i)=Jbi;
				dJb.col(i) = ad(Jbi)*prev_dJidt;
				prev_dJidt += Jbi*dq(i);			   
		}
		T = M*TransInv(T_);
	}
	Matrix3d skew3(const Vector3d& xi) {
		Matrix3d skew;
		skew <<  0, -xi(2),  xi(1),
				xi(2),     0, -xi(0),
			-xi(1),  xi(0),     0;
		return skew;
	}
	Matrix3d dexp3(const Vector3d& xi) {
		const double eps = std::numeric_limits<double>::epsilon();
		if (xi.norm() < eps) {
			return Matrix3d::Identity();
		}
		
		Matrix3d ceil_xi = skew3(xi);
		double norm_xi = xi.norm();
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;
		
		Matrix3d dexp =Matrix3d::Identity() + beta / 2 * ceil_xi + (1 - alpha) / (norm_xi * norm_xi) * ceil_xi * ceil_xi;
		return dexp;
	}
	Matrix3d dlog3(const Vector3d& xi) {
		const double eps = std::numeric_limits<double>::epsilon();
		
		if (xi.norm() < eps) {
			return Matrix3d::Identity();
		}
		
		double norm_xi = xi.norm();
		Matrix3d ceil_xi = skew3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;
		double gamma = alpha / beta;
		
		Matrix3d dlog = Matrix3d::Identity() - 0.5 * skew3(xi) + (1 - gamma) / (norm_xi * norm_xi) * ceil_xi * ceil_xi;
		
		return dlog;
	}
	Matrix6d dexp6(const Vector6d& lambda) {
		const double eps = std::numeric_limits<double>::epsilon();

		Vector3d eta = lambda.segment(0, 3);
		Matrix3d ceil_eta = skew3(eta);

		Vector3d xi = lambda.segment(3, 3);
		double norm_xi = xi.norm();
		Matrix3d ceil_xi = skew3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;

		Matrix3d Cxi;
		if (xi.norm() < eps) {
			Cxi = 0.5 * skew3(eta);
		} else {
			Cxi = beta/2 * skew3(eta) +
				(1-alpha)/(norm_xi*norm_xi) * (skew3(eta)*skew3(xi) + skew3(xi)*skew3(eta)) +
				(alpha-beta)/(norm_xi*norm_xi) * xi.transpose()*eta*skew3(xi) -
				1/(norm_xi*norm_xi) * (3*(1-alpha)/(norm_xi*norm_xi) - beta/2) * xi.transpose()*eta*skew3(xi)*skew3(xi);
		}

		Matrix3d dexp_xi = dexp3(xi);
		Matrix6d dexp=Matrix6d::Zero();
		dexp << dexp_xi, Cxi,
				Matrix3d::Zero(), dexp_xi;

		return dexp;
	}
	Matrix3d ddexp3(const Vector3d& xi, const Vector3d& dxi) {
		const double eps = std::numeric_limits<double>::epsilon();

		if (xi.norm() < eps) {
			return 0.5 * skew3(dxi);
		}

		Matrix3d ceil_xi = skew3(xi);
		double norm_xi = xi.norm();
		double s = sin(norm_xi / 2.0) / (norm_xi / 2.0);
		double c = cos(norm_xi / 2.0);
		double alpha = s * c;
		double beta = s * s;
		Vector3d eta = dxi;
		double norm_xi2 = norm_xi*norm_xi;
		Matrix3d Cxi = beta/2.0* skew3(eta) +
					   (1.0-alpha)/norm_xi2 * skew_sum(eta,xi) +
						(alpha-beta)/norm_xi2 * xi.dot(eta)  * skew3(xi) -
						1.0/norm_xi2 * (3.0*(1.0-alpha)/norm_xi2 - beta/2.0) * xi.dot(eta)* ceil_xi * ceil_xi;
		return Cxi;
	}	
	Matrix3d dddexp3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
	const double eps = std::numeric_limits<double>::epsilon();

	if (xi.norm() < eps) {
		return 0.5 * skew3(dy) + (1.0/6.0) * (skew3(dxi)*skew3(y) + skew3(y)*skew3(dxi));
	}
	Matrix3d dCxi;
	Matrix3d ceil_xi = skew3(xi);
	double norm_xi = xi.norm();
	double norm_xi2 = norm_xi * norm_xi;
	Matrix3d ceil_dxi = skew3(dxi);
	double s = sin(norm_xi / 2) / (norm_xi / 2);
	double c = cos(norm_xi / 2);
	double alpha = s * c;
	double beta = s * s;
	Matrix3d ceil_dy = skew3(dy);
	Matrix3d ceil_y = skew3(y);
	Matrix3d ceil_dy_xi = (ceil_dy*ceil_xi + ceil_xi*ceil_dy);
	Matrix3d ceil_y_dxi = (ceil_y*ceil_dxi + ceil_dxi*ceil_y);
	Matrix3d ceil_xi_y = (ceil_xi*ceil_y + ceil_y*ceil_xi);
	Matrix3d ceil_xi_dxi = (ceil_xi*ceil_dxi + ceil_dxi*ceil_xi);
	Matrix3d ceil_xi2 = ceil_xi*ceil_xi;

	double zeta = xi.dot( y) * xi.dot(dxi)  / norm_xi2;
	double Gamma1 = (1 - alpha) / norm_xi2;
	double Gamma2 = (alpha - beta) / norm_xi2;
	double Gamma3 = (beta / 2.0 - 3.0 * Gamma1) / norm_xi2;
	double Gamma4 = -Gamma2 / beta;
	double Gamma5 = (Gamma1 + 2.0 * Gamma2) / (norm_xi2 * beta);
	double delta0 = dxi.dot(y) + xi.dot(dy) ;
	 Matrix3d delta1 = xi.dot( y)  * ceil_dxi + xi.dot(dxi)  * ceil_y + (delta0 - 4 * zeta) * ceil_xi;
	 Matrix3d delta2 = xi.dot( y)  * ceil_xi_dxi + xi.dot(dxi) * ceil_xi_y + (delta0 - 5 * zeta) * ceil_xi2;
	 Matrix3d delta3 = xi.dot( y)  * ceil_xi_dxi + xi.dot(dxi) * ceil_xi_y + (delta0 - 3 * zeta) * ceil_xi2;
	 dCxi = beta/2.0 * (ceil_dy - zeta * ceil_xi) + Gamma1 * (ceil_dy_xi + ceil_y_dxi + zeta * ceil_xi) +
	 						Gamma2 * (delta1 + zeta * ceil_xi2) + Gamma3 * delta2;

	return dCxi;
	}
	Matrix6d ddexp6(const Vector6d& lambda, const Vector6d& lambda_dot) {
		const double eps = std::numeric_limits<double>::epsilon();

		Vector3d eta = lambda.segment(0, 3);
		Vector3d xi = lambda.segment(3, 3);
		Vector3d eta_dot = lambda_dot.segment(0, 3);
		Vector3d xi_dot = lambda_dot.segment(3, 3);

		Matrix3d C = ddexp3(xi, xi_dot);
		Matrix3d C_dot = dddexp3(xi, xi_dot, eta, eta_dot);

		Matrix6d ddexp;
		ddexp << C, C_dot,
				Matrix3d::Zero(), C;

		return ddexp;
	}	
	Matrix3d skew_sum(const Vector3d& a, const Vector3d& b) {
		return skew3(a) * skew3(b) + skew3(b) * skew3(a);
	}

Matrix3d ddlog3(const Vector3d& xi, const Vector3d& dxi) {
		const double eps = std::numeric_limits<double>::epsilon();

		if (xi.norm() < eps) {
			return -0.5 * skew3(dxi);
		}

		double norm_xi = xi.norm();
		double norm_xi2 = norm_xi * norm_xi;
		Matrix3d ceil_xi = skew3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;
		double gamma = alpha / beta;

		Matrix3d D = -0.5 * skew3(dxi) + (1 - gamma) / norm_xi2 * skew_sum(dxi, xi) +
							1 / norm_xi2 * (1 / beta + gamma - 2) / norm_xi2 * xi.transpose() * dxi * skew3(xi) * skew3(xi);

		return D;
	}	
Matrix3d dddlog3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
    const double eps = std::numeric_limits<double>::epsilon();

    if (xi.norm() < eps) {
        return -0.5 * skew3(dy) + (1.0/12.0) * (skew3(dxi)*skew3(y) + skew3(y)*skew3(dxi));
    }

    Matrix3d ceil_xi = skew3(xi);
    double norm_xi = xi.norm();
    double norm_xi2 = norm_xi * norm_xi;
    Matrix3d ceil_dxi = skew3(dxi);
    double s = sin(norm_xi / 2) / (norm_xi / 2);
    double c = cos(norm_xi / 2);
    double alpha = s * c;
    double beta = s * s;
    Matrix3d ceil_dy = skew3(dy);
    Matrix3d ceil_y = skew3(y);
    Matrix3d ceil_dy_xi = (ceil_dy*ceil_xi + ceil_xi*ceil_dy);
    Matrix3d ceil_y_dxi = (ceil_y*ceil_dxi + ceil_dxi*ceil_y);
    Matrix3d ceil_xi_y = (ceil_xi*ceil_y + ceil_y*ceil_xi);
    Matrix3d ceil_xi_dxi = (ceil_xi*ceil_dxi + ceil_dxi*ceil_xi);
    Matrix3d ceil_xi2 = ceil_xi*ceil_xi;

    double zeta = xi.dot(y) * xi.dot(dxi) / norm_xi2;
    double gamma = alpha / beta;
    double Gamma1 = (1 - alpha) / norm_xi2;
    double Gamma2 = (alpha - beta) / norm_xi2;
    double Gamma3 = (beta / 2 - 3 * Gamma1) / norm_xi2;
    double Gamma4 = -Gamma2 / beta;
    double Gamma5 = (Gamma1 + 2 * Gamma2) / (norm_xi2 * beta);
    double delta0 = dxi.dot(y) + xi.dot(dy);
    Matrix3d delta1 = xi.dot(y) * ceil_dxi + xi.dot(dxi) * ceil_y + (delta0 - 4 * zeta) * ceil_xi;
    Matrix3d delta2 = xi.dot(y) * ceil_xi_dxi + xi.dot(dxi) * ceil_xi_y + (delta0 - 5 * zeta) * ceil_xi2;
    Matrix3d delta3 = xi.dot(y) * ceil_xi_dxi + xi.dot(dxi) * ceil_xi_y + (delta0 - 3 * zeta) * ceil_xi2;
    Matrix3d dDxi = -0.5 * skew3(dy) + 2 / norm_xi2 * (1 - gamma / beta) / norm_xi2 * zeta * ceil_xi2 +
                           Gamma4 * (ceil_dy_xi + ceil_y_dxi) + Gamma5 * delta3;

    return dDxi;
}
Matrix6d dlog6(const Vector6d& lambda) {
    const double eps = std::numeric_limits<double>::epsilon();

    Vector3d eta = lambda.segment(0, 3);
    Vector3d xi = lambda.segment(3, 3);

    double norm_xi = xi.norm();
    Matrix3d ceil_xi = skew3(xi);
    double s = sin(norm_xi / 2) / (norm_xi / 2);
    double c = cos(norm_xi / 2);
    double alpha = s * c;
    double beta = s * s;
    double gamma = alpha / beta;

    Matrix3d dlog_3 = dlog3(xi);
    Matrix3d O = Matrix3d::Zero();
    double norm_xixi = norm_xi * norm_xi;
    Matrix3d D = -0.5 * skew3(eta) + (1 - gamma) / norm_xixi * (skew3(eta) * skew3(xi) + skew3(xi) * skew3(eta)) +
                        1 / norm_xixi * (1 / beta + gamma - 2) / norm_xixi * xi.transpose() * eta * skew3(xi) * skew3(xi);

    if (norm_xi < eps) {
        D = -0.5 * skew3(eta);
    }

    Matrix6d dlog(6, 6);
    dlog << dlog_3, D,
            O, dlog_3;

    return dlog;
}
Matrix6d ddlog6(const Vector6d& lambda, const Vector6d& lambda_dot) {
    Vector3d xi = lambda.segment(3, 3);
    Vector3d xi_dot = lambda_dot.segment(3, 3);
    Vector3d eta = lambda.segment(0, 3);
    Vector3d eta_dot = lambda_dot.segment(0, 3);

    Matrix3d D = ddlog3(xi, xi_dot);
    Matrix3d D_dot = dddlog3(xi, xi_dot, eta, eta_dot);

    Matrix6d ddlog(6, 6);
    ddlog << D, D_dot,
             Matrix3d::Zero(), D;

    return ddlog;
}
Vector6d flip(Vector6d V){
	Vector6d V_flip;
	V_flip<<V(3),V(4),V(5),V(0),V(1),V(2);
	return V_flip;
}
void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,vector<SE3>&Xd_list,vector<Vector6d>&Vd_list,vector<Vector6d>&dVd_list){
	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
	lambda_0 = Vector6d::Zero();
	lambda_T = flip(se3ToVec(MatrixLog6(TransInv(X0)*XT)));
	dlambda_0 = V0;
	dlambda_T = dlog6(-lambda_T)*VT;
	ddlambda_0 = dV0;
	ddlambda_T = dlog6(-lambda_T)*dVT +ddlog6(-lambda_T,-dlambda_T)*VT;
	double timegap = Tf /(N/1.0 - 1.0);
	for (int i = 0;i<N;i++){
		lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();
		double t= timegap*(i-1);
		for(int j = 0;j<6;j++){
			Vector3d ret = QuinticTimeScalingKinematics(lambda_0(j),lambda_T(j),dlambda_0(j),dlambda_T(j),ddlambda_0(j),ddlambda_T(j),Tf,t) ;
			lambda_t(j) = ret(0);
			dlambda_t(j) = ret(1);
			ddlambda_t(j) = ret(2);
		}
		Vector6d V = dexp6(-lambda_t)*dlambda_t;
		Vector6d dV = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
		SE3 T = X0*MatrixExp6(VecTose3(flip(lambda_t)));
		Xd_list.at(i) = T;
		Vd_list.at(i) = flip(V);
		dVd_list.at(i) = flip(dV);
	}
	
	
}

}

