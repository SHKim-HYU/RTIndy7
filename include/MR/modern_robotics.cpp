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
		Matrix3d omgmat = VecToso3(Vector3d(V(0), V(1), V(2)));
		Matrix6d result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Zero3d;
		result.bottomLeftCorner<3, 3>() = VecToso3(Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
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
		SO3 R = T.block<3,3>(0,0);
		Vector3d p =T.block<3,1>(0,3);
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret.block<3,3>(0,0) = R;
		ad_ret.block<3,3>(3,3) = R;
		ad_ret.block<3,3>(3,0) << p(1)*R(2,0) - p(2)*R(1,0), p(1)*R(2,1) - p(2)*R(1,1), p(1)*R(2,2) - p(2)*R(1,2),
								  p(2)*R(0,0) - p(0)*R(2,0), p(2)*R(0,1) - p(0)*R(2,1), p(2)*R(0,2) - p(0)*R(2,2),
								  p(0)*R(1,0) - p(1)*R(0,0), p(0)*R(1,1) - p(1)*R(0,1), p(0)*R(1,2) - p(1)*R(0,2);
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
	void AngularJacobian(const SO3& R0,const SO3& RT,Matrix3d& Jw0, Matrix3d& JwT){
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm(R0,Rz0,Ry0,Rx0); 
		rotm2eulm(RT,RzT,RyT,RxT); 
		
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d Jw0x=  so3ToVec(wx0T);
		Vector3d Jw0y=  Rx0.transpose()*so3ToVec(wy0T);
		Vector3d Jw0z=  Rx0.transpose()*Ry0.transpose()*so3ToVec(wz0T);

		Vector3d JwTx=  so3ToVec(wx0T);
		Vector3d JwTy=  RxT.transpose()*so3ToVec(wy0T);
		Vector3d JwTz=  RxT.transpose()*RyT.transpose()*so3ToVec(wz0T);

		Jw0<<Jw0x[0],Jw0y[0],Jw0z[0], 
			 Jw0x[1],Jw0y[1],Jw0z[1],
			 Jw0x[2],Jw0y[2],Jw0z[2];
		JwT<<JwTx[0],JwTy[0],JwTz[0], 
			 JwTx[1],JwTy[1],JwTz[1],
			 JwTx[2],JwTy[2],JwTz[2];
	}

	void DerivativeAngularJacobian(const Matrix3d& Jw0, const Matrix3d& JwT, Matrix3d& Cw0,Matrix3d& CwT){
		Vector3d Jw0x,Jw0y,Jw0z,JwTx,JwTy,JwTz;
		Jw0x<< Jw0(0,0),Jw0(1,0),Jw0(2,0);
		Jw0y<< Jw0(0,1),Jw0(1,1),Jw0(2,1);
		Jw0z<< Jw0(0,2),Jw0(1,2),Jw0(2,2);
		JwTx<< JwT(0,0),JwT(1,0),JwT(2,0);
		JwTy<< JwT(0,1),JwT(1,1),JwT(2,1);
		JwTz<< JwT(0,2),JwT(1,2),JwT(2,2);
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		
		// Vector3d Cw0xx = Vector3d::Zero()*ds0[0]*ds0[0];
		// Vector3d Cw0xy = so3ToVec(-LieBracket(VecToso3(Jw0x),VecToso3(Jw0y)))*ds0[0]*ds0[1];
		// Vector3d Cw0xz = so3ToVec(-LieBracket(VecToso3(Jw0x),VecToso3(Jw0z)))*ds0[0]*ds0[2];
		// Vector3d Cw0yz = so3ToVec(-LieBracket(VecToso3(Jw0y),VecToso3(Jw0z)))*ds0[1]*ds0[2];		

		// Vector3d CwTxx = Vector3d::Zero()*dsT[0]*dsT[0];
		// Vector3d CwTxy = so3ToVec(-LieBracket(VecToso3(JwTx),VecToso3(JwTy)))*dsT[0]*dsT[1];
		// Vector3d CwTxz = so3ToVec(-LieBracket(VecToso3(JwTx),VecToso3(JwTz)))*dsT[0]*dsT[2];
		// Vector3d CwTyz = so3ToVec(-LieBracket(VecToso3(JwTy),VecToso3(JwTz)))*dsT[1]*dsT[2];		


		// Cw0<< Cw0xx[0]+ Cw0xy[0]+ Cw0xz[0]+Cw0yz[0],
		// 	  Cw0xx[1]+ Cw0xy[1]+ Cw0xz[1]+Cw0yz[1],
		// 	  Cw0xx[2]+ Cw0xy[2]+ Cw0xz[2]+Cw0yz[2];
		// CwT << CwTxx[0]+ CwTxy[0]+ CwTxz[0]+CwTyz[0],
		// 	  CwTxx[1]+ CwTxy[1]+ CwTxz[1]+CwTyz[1],
		// 	  CwTxx[2]+ CwTxy[2]+ CwTxz[2]+CwTyz[2];
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
	

	SE3 EulerT(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf) {
		
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;		
		rotm2eulm(R0,Rz0,Ry0,Rx0); 
		rotm2eulm(RT,RzT,RyT,RxT); 
		//37us
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);


		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);
		
		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		v0 = v0;
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		vT = vT;
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];




		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*Ry0T*vec_wz0T;

		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*vec_wy0T;
		Vector3d JwTz=  RxTT*RyT*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();
		
		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();
		Vector3d ds0_w = pinvJw0*w0;
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		
		Vector3d dsT_w = pinvJwT*wT;
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
	
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rys*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];


		SE3 retT =RpToTrans(Rs,ps);
		if(t>Tf)retT = Xend;
		return retT;
	}	

	Vector6d EulerVel(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf) {
		
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;		
		rotm2eulm(R0,Rz0,Ry0,Rx0); 
		rotm2eulm(RT,RzT,RyT,RxT); 
		//37us
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);


		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);
		
		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		v0 = v0;
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		vT = vT;
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];




		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*Ry0T*vec_wz0T;

		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*vec_wy0T;
		Vector3d JwTz=  RxTT*RyT*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();
		
		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();
		Vector3d ds0_w = pinvJw0*w0;
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		
		Vector3d dsT_w = pinvJwT*wT;
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rys*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  vec_wx0T;
		Vector3d Jwy=  RxsT*vec_wy0T;
		Vector3d Jwz=  RxsT*RysT*vec_wz0T;
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		ds_w << ds[0],ds[1],ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];
		vs = vs;
		Vector6d retVel;
		retVel<<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		if(t>Tf)retVel = Vend;
		return retVel;
	}			
	Vector6d EulerAcc(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf) {
				SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;		
		rotm2eulm(R0,Rz0,Ry0,Rx0); 
		rotm2eulm(RT,RzT,RyT,RxT); 
		//37us
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);


		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);
		
		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		v0 = v0;
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		vT = vT;
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];




		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*Ry0T*vec_wz0T;

		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*vec_wy0T;
		Vector3d JwTz=  RxTT*RyT*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();
		
		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();
		Vector3d ds0_w = pinvJw0*w0;
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		
		Vector3d dsT_w = pinvJwT*wT;
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rys*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  vec_wx0T;
		Vector3d Jwy=  RxsT*vec_wy0T;
		Vector3d Jwz=  RxsT*RysT*vec_wz0T;
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf)retAcc = Vdotend;
		return retAcc;
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
	/*
		void EulerKinematics(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd) {
				SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;		
		rotm2eulm(R0,Rz0,Ry0,Rx0); 
		rotm2eulm(RT,RzT,RyT,RxT); 
		//37us
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);


		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);
		
		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		v0 = v0;
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		vT = vT;
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];




		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*Ry0T*vec_wz0T;

		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*vec_wy0T;
		Vector3d JwTz=  RxTT*RyT*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();
		
		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();
		Vector3d ds0_w = pinvJw0*w0;
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		
		Vector3d dsT_w = pinvJwT*wT;
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rys*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  vec_wx0T;
		Vector3d Jwy=  RxsT*vec_wy0T;
		Vector3d Jwz=  RxsT*RysT*vec_wz0T;
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}
	}
	
	*/

	
	void EulerKinematics(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd ,int* order) {
		if(order[0]==0){
			if(order[1]==1){
				//cout<<"EULER XYZ"<<endl;
				EulerKinematicsXYZ(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);	
			}else{
				//cout<<"EULER XZY"<<endl;
			    EulerKinematicsXZY(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);	
			}
		}
		else if(order[0]==1){
			if(order[1]==0){
				//cout<<"EULER YXZ"<<endl;
				EulerKinematicsYXZ(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);	
			}else{
				//cout<<"EULER YZX"<<endl;
				EulerKinematicsYZX(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);	
			}
		}
		else if(order[0]==2){
			if(order[1]==0){
				//cout<<"EULER ZXY"<<endl;
				EulerKinematicsZXY(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);
			}else{
				//cout<<"EULER ZYX"<<endl;
				EulerKinematicsZYX(Xstart,Xend,Vstart,Vdotend,Vdotstart,Vend,t, Tf, Xd,Vd, dVd);
			}
		}		

	}
void rotm2eulm_(SO3& R,SO3& Rx,SO3& Ry,SO3& Rz,int order[3]){
	Vector3d euler_angle =  R.eulerAngles(order[0],order[1],order[2]); 
    double roll = 0, pitch = 0, yaw = 0;
	for(int i =0;i<3;i++){
		if(order[i] == 0){
			roll = wrapTo2PI(euler_angle(i)+2*M_PI);
		}
		else if(order[i] == 1){
			pitch = wrapTo2PI(euler_angle(i)+2*M_PI);
		}
		else if(order[i] == 2){
			yaw = wrapTo2PI(euler_angle(i)+2*M_PI);
		}
	}
	
	Rx<< 1 ,0 ,0,
	         0 ,cos(roll),-sin(roll) ,
			 0 ,sin(roll),cos(roll);
	Ry<<cos(pitch) ,0,sin(pitch),
	        0 ,1,0,
			-sin(pitch) ,0,cos(pitch);
	Rz<<cos(yaw),-sin(yaw) ,0,
		    sin(yaw),cos(yaw),0,
			0 ,0,1;	
}
	void EulerKinematicsZYX(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {2,1,0};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;		
		//rotm2eulm(R0,Rz0,Ry0,Rx0); 
		//rotm2eulm(RT,RzT,RyT,RxT); 
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 
		//37us
		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);


		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);
		
		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		v0 = v0;
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		vT = vT;
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];




		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*Ry0T*vec_wz0T;

		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*vec_wy0T;
		Vector3d JwTz=  RxTT*RyT*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();
		
		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();
		Vector3d ds0_w = pinvJw0*w0;
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		
		Vector3d dsT_w = pinvJwT*wT;
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rys*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  vec_wx0T;
		Vector3d Jwy=  RxsT*vec_wy0T;
		Vector3d Jwz=  RxsT*RysT*vec_wz0T;
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}
		
	}
	void EulerKinematicsZXY(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {2,0,1};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 

		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);

		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];
		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();

		//CALC Jw0

		Vector3d Jw0x=  Ry0T*vec_wx0T;
		Vector3d Jw0y=  vec_wy0T;
		Vector3d Jw0z=  Ry0T*Rx0T*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;

		//CALC JwT
		Vector3d JwTx=  RyTT*vec_wx0T;
		Vector3d JwTy=  vec_wy0T;
		Vector3d JwTz=  RyTT*RxTT*vec_wz0T;		
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;

		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();

		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();

		Vector3d ds0_w = pinvJw0*w0;
		Vector3d dsT_w = pinvJwT*wT;
		
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		//Calc Cw0
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		//Calc CwT
		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rzs*Rxs*Rys;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  RysT*vec_wx0T;
		Vector3d Jwy=  vec_wy0T;
		Vector3d Jwz=  RysT*RxsT*vec_wz0T;		
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}
				
	}
	void EulerKinematicsYXZ(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {1,0,2};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 

		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);

		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];
		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 Rz0T = Rz0.transpose();

		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();
		SO3 RzTT = RzT.transpose();
		//CALC Jw0

		Vector3d Jw0x=  Rz0T*vec_wx0T;
		Vector3d Jw0y=  Rz0T*Rx0T*vec_wy0T;
		Vector3d Jw0z=  vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;

		//CALC JwT
		Vector3d JwTx=  RzTT*vec_wx0T;
		Vector3d JwTy=  RzTT*RxTT*vec_wy0T;
		Vector3d JwTz=  vec_wz0T;		
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;

		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();

		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();

		Vector3d ds0_w = pinvJw0*w0;
		Vector3d dsT_w = pinvJwT*wT;
		
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		//Calc Cw0
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		//Calc CwT
		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rys*Rxs*Rzs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();
		SO3 RzsT = Rzs.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  RzsT*vec_wx0T;
		Vector3d Jwy=  RzsT*RxsT*vec_wy0T;
		Vector3d Jwz=  vec_wz0T;		
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}
					
	}
	void EulerKinematicsYZX(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {1,2,0};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 

		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);

		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];
		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 Rz0T = Rz0.transpose();
		
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();
		SO3 RzTT = RzT.transpose();

		//CALC Jw0

		Vector3d Jw0x=  vec_wx0T;
		Vector3d Jw0y=  Rx0T*Rz0T*vec_wy0T;
		Vector3d Jw0z=  Rx0T*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;

		//CALC JwT
		Vector3d JwTx=  vec_wx0T;
		Vector3d JwTy=  RxTT*RzTT*vec_wy0T;
		Vector3d JwTz=  RxTT*vec_wz0T;		
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;

		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();

		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();

		Vector3d ds0_w = pinvJw0*w0;
		Vector3d dsT_w = pinvJwT*wT;
		
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		//Calc Cw0
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		//Calc CwT
		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rys*Rzs*Rxs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();
		SO3 RzsT = Rzs.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  vec_wx0T;
		Vector3d Jwy=  RxsT*RzsT*vec_wy0T;
		Vector3d Jwz=  RxsT*vec_wz0T;		
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}		
	}
	void EulerKinematicsXYZ(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {0,1,2};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 

		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);

		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];
		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 Rz0T = Rz0.transpose();
		
		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();
		SO3 RzTT = RzT.transpose();

		//CALC Jw0

		Vector3d Jw0x=  Rz0T*Ry0T*vec_wx0T;
		Vector3d Jw0y=  Rz0T*vec_wy0T;
		Vector3d Jw0z=  vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;
		
		//CALC JwT
		Vector3d JwTx=  RzTT*RyTT*vec_wx0T;
		Vector3d JwTy=  RzTT*vec_wy0T;
		Vector3d JwTz=  vec_wz0T;		
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;


		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();

		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();

		Vector3d ds0_w = pinvJw0*w0;
		Vector3d dsT_w = pinvJwT*wT;
		
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		//Calc Cw0
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		//Calc CwT
		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rxs*Rys*Rzs;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();
		SO3 RzsT = Rzs.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  RzsT*RysT*vec_wx0T;
		Vector3d Jwy=  RzsT*vec_wy0T;
		Vector3d Jwz=  vec_wz0T;		
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];
		cout<<"s : "<<s.transpose()<<endl;
		cout<<"ds : "<<ds.transpose()<<endl;
		cout<<"dds : "<<dds.transpose()<<endl;
		cout<<"Jw0 \n "<<Jw0<<endl;			
		cout<<"JwT \n "<<JwT<<endl;
		cout<<"Jw \n "<<Jw<<endl;
		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}				
	}
	void EulerKinematicsXZY(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd){
		int order[3] = {0,2,1};
		SO3 R0 = TransToR(Xstart);
		SO3 RT = TransToR(Xend);
		SO3 Rx0,Ry0,Rz0;
		SO3 RxT,RyT,RzT;
		rotm2eulm_(R0,Rx0,Ry0,Rz0,order); 
		rotm2eulm_(RT,RxT,RyT,RzT,order); 

		so3 wz0T = MatrixLog3(Rz0.transpose()*RzT);
		so3 wy0T = MatrixLog3(Ry0.transpose()*RyT);
		so3 wx0T = MatrixLog3(Rx0.transpose()*RxT);

		Vector3d vec_wz0T = so3ToVec(wz0T);
		Vector3d vec_wy0T = so3ToVec(wy0T);
		Vector3d vec_wx0T = so3ToVec(wx0T);

		Vector3d p0 = TransToP(Xstart);
		Vector3d pT = TransToP(Xend);
		Vector3d p0T = pT-p0;
		Vector3d w0,wT,v0,vT,wdot0,wdotT,vdot0,vdotT;
		w0 << Vstart[0],Vstart[1],Vstart[2];
		v0 << Vstart[3],Vstart[4],Vstart[5];
		wdot0 << Vdotstart[0],Vdotstart[1],Vdotstart[2];
		vdot0 << Vdotstart[3],Vdotstart[4],Vdotstart[5];
		wT << Vend[0],Vend[1],Vend[2];
		vT << Vend[3],Vend[4],Vend[5];
		wdotT << Vdotend[0],Vdotend[1],Vdotend[2];
		vdotT << Vdotend[3],Vdotend[4],Vdotend[5];
		Matrix3d Jw0,JwT;
		Matrix3d Cw0,CwT;
		SO3 Rx0T = Rx0.transpose();
		SO3 Ry0T = Ry0.transpose();
		SO3 Rz0T = Rz0.transpose();


		SO3 RxTT = RxT.transpose();
		SO3 RyTT = RyT.transpose();
		SO3 RzTT = RzT.transpose();

		//CALC Jw0

		Vector3d Jw0x=  Ry0T*Rz0T*vec_wx0T;
		Vector3d Jw0y=  vec_wy0T;
		Vector3d Jw0z=  Ry0T*vec_wz0T;
		Jw0.block<3,1>(0,0) = Jw0x;
		Jw0.block<3,1>(0,1) = Jw0y;
		Jw0.block<3,1>(0,2) = Jw0z;

		//CALC JwT
		Vector3d JwTx=  RyTT*RzTT*vec_wx0T;
		Vector3d JwTy=  vec_wy0T;
		Vector3d JwTz=  RyTT*vec_wz0T;		
		JwT.block<3,1>(0,0) = JwTx;
		JwT.block<3,1>(0,1) = JwTy;
		JwT.block<3,1>(0,2) = JwTz;

		Matrix3d Jw0T = Jw0.transpose();
		Matrix3d JwTT = JwT.transpose();

		Matrix3d pinvJw0 = Jw0T* (Jw0 *Jw0T).inverse();
		Matrix3d pinvJwT = JwTT * (JwT * JwTT).inverse();

		Vector3d ds0_w = pinvJw0*w0;
		Vector3d dsT_w = pinvJwT*wT;
		
		Vector3d ds0_wds0_w,dsT_wdsT_w;
		ds0_wds0_w<< ds0_w[0]*ds0_w[0],ds0_w[1]*ds0_w[1],ds0_w[2]*ds0_w[2];
		dsT_wdsT_w<< dsT_w[0]*dsT_w[0],dsT_w[1]*dsT_w[1],dsT_w[2]*dsT_w[2];

		//Calc Cw0
		Cw0.block<3,1>(0,0) = (VecToso3(Jw0y)+VecToso3(Jw0z))*Jw0x;
		Cw0.block<3,1>(0,1) = (VecToso3(Jw0z))*Jw0y;
		Cw0.block<3,1>(0,2) = Vector3d::Zero();

		//Calc CwT
		CwT.block<3,1>(0,0) = (VecToso3(JwTy)+VecToso3(JwTz))*JwTx;
		CwT.block<3,1>(0,1) = (VecToso3(JwTz))*JwTy;
		CwT.block<3,1>(0,2) = Vector3d::Zero();		


		Vector3d dds0_w = pinvJw0*wdot0-pinvJw0*Cw0*ds0_wds0_w;
		Vector3d ddsT_w = pinvJwT*wdotT-pinvJwT*CwT*dsT_wdsT_w;		
		Vector3d ds0_v,dsT_v,dds0_v,ddsT_v;

		ds0_v << v0[0]/p0T[0],v0[1]/p0T[1],v0[2]/p0T[2];
		dsT_v << vT[0]/p0T[0],vT[1]/p0T[1],vT[2]/p0T[2];
		dds0_v << vdot0[0]/p0T[0],vdot0[1]/p0T[1],vdot0[2]/p0T[2];
		ddsT_v << vdotT[0]/p0T[0],vdotT[1]/p0T[1],vdotT[2]/p0T[2];

		Vector6d ds0,dsT;
		Vector6d dds0,ddsT;
		ds0<<ds0_w[0] ,ds0_w[1],ds0_w[2], ds0_v[0], ds0_v[1], ds0_v[2];
		dsT<<dsT_w[0] ,dsT_w[1],dsT_w[2], dsT_v[0], dsT_v[1], dsT_v[2];
		dds0<<dds0_w[0] ,dds0_w[1],dds0_w[2], dds0_v[0], dds0_v[1], dds0_v[2];
		ddsT<<ddsT_w[0] ,ddsT_w[1],ddsT_w[2], ddsT_v[0], ddsT_v[1], ddsT_v[2];

		Vector6d s,ds,dds;
		for(int i = 0;i<6;i++){
			Vector3d s_ds_dds= QuinticTimeScalingKinematics(0,1,ds0[i],dsT[i],dds0[i],ddsT[i],Tf, t);
			s[i] = s_ds_dds[0];
			ds[i] = s_ds_dds[1];
			dds[i] = s_ds_dds[2];
		}
		so3 e_wx = MatrixExp3(wx0T*s[0]);
		so3 e_wy = MatrixExp3(wy0T*s[1]);
		so3 e_wz = MatrixExp3(wz0T*s[2]);

		SO3 Rxs = Rx0*e_wx;
		SO3 Rys = Ry0*e_wy;
		SO3 Rzs = Rz0*e_wz;

		SO3 Rs = Rxs*Rzs*Rys;
		Vector3d ps;
		ps<< p0[0] + s[3]*p0T[0], p0[1] + s[4]*p0T[1], p0[2] + s[5]*p0T[2];

		SO3 RxsT = Rxs.transpose();
		SO3 RysT = Rys.transpose();
		SO3 RzsT = Rzs.transpose();

		Matrix3d Jw;
		Vector3d Jwx=  RysT*RzsT*vec_wx0T;
		Vector3d Jwy=  vec_wy0T;
		Vector3d Jwz=  RysT*vec_wz0T;		
		Jw<<Jwx[0],Jwy[0],Jwz[0],
			Jwx[1],Jwy[1],Jwz[1],
			Jwx[2],Jwy[2],Jwz[2];

		Vector3d ds_w;
		Vector3d ds_wds_w;
		ds_w << ds[0],ds[1],ds[2];
		ds_wds_w<<ds[0]*ds[0],ds[1]*ds[1],ds[2]*ds[2];
		Vector3d ws = Jw*ds_w;
		Vector3d vs;
		vs<<ds[3]*p0T[0],ds[4]*p0T[1],ds[5]*p0T[2];

		
		Matrix3d Cw;
		Cw.block<3,1>(0,0) = (VecToso3(Jwy)+VecToso3(Jwz))*Jwx;
		Cw.block<3,1>(0,1) = (VecToso3(Jwz))*Jwy;
		Cw.block<3,1>(0,2) = Vector3d::Zero();	

		Vector3d dds_w;
		dds_w<<dds[0],dds[1],dds[2] ;
		Vector3d wdots = Jw*dds_w+Cw*ds_wds_w;
		Vector3d vdots;
		vdots<<dds[3]*p0T[0],dds[4]*p0T[1],dds[5]*p0T[2];

		Vector6d retAcc;
		retAcc<<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		Xd = RpToTrans(Rs,ps);
		Vd <<ws[0],ws[1],ws[2],vs[0],vs[1],vs[2];
		dVd <<wdots[0],wdots[1],wdots[2],vdots[0],vdots[1],vdots[2];
		if(t>Tf){
			Xd = Xend;
			Vd = Vend;
			dVd = Vdotend;
		}			
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
}
