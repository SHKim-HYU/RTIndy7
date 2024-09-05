/*
 * liegroup_robotics.cpp
 *
 *  Created on: May 29, 2024
 *      Author: Minchang Sung, Sunhong Kim
 */

#include "liegroup_robotics.h"


# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace lr {
	Matrix6d ad(const Vector6d& V) {
		Matrix6d result=Matrix6d::Zero();
		
		result<<      0, -V(5),  V(4),  0,-V(2),V(1),
			       V(5),     0, -V(3),  V(2),0,-V(0),
				  -V(4),  V(3),     0,  -V(1),V(0),0,
				   0,    0,   0, 0 ,-V(5) ,V(4),
				   0,    0,   0, V(5) ,0 ,-V(3),
				   0,	 0,	  0,-V(4),V(3) ,0;
		return result;
	}    
	Matrix6d Ad(const SE3& T) {
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret<<T(0,0),T(0,1),T(0,2),T(1,3)*T(2,0) - T(2,3)*T(1,0), T(1,3)*T(2,1) - T(2,3)*T(1,1), T(1,3)*T(2,2) - T(2,3)*T(1,2),
				T(1,0),T(1,1),T(1,2),T(2,3)*T(0,0) - T(0,3)*T(2,0), T(2,3)*T(0,1) - T(0,3)*T(2,1), T(2,3)*T(0,2) - T(0,3)*T(2,2),
				T(2,0),T(2,1),T(2,2),T(0,3)*T(1,0) - T(1,3)*T(0,0), T(0,3)*T(1,1) - T(1,3)*T(0,1), T(0,3)*T(1,2) - T(1,3)*T(0,2),
				0,0,0,T(0,0),T(0,1),T(0,2),
				0,0,0,T(1,0),T(1,1),T(1,2),
				0,0,0,T(2,0),T(2,1),T(2,2);
		return ad_ret;
	}
	// Matrix6d AdInv(const SE3& T) {
	// 	Matrix6d ad_ret = Ad(TransInv(T));
	// 	return ad_ret;
	// }				
	Matrix6d AdInv(const SE3& T) {
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret<<T(0,0), T(1,0), T(2,0), T(1,3)*T(2,0) - T(1,0)*T(2,3), T(0,0)*T(2,3) - T(0,3)*T(2,0), T(0,3)*T(1,0) - T(0,0)*T(1,3),
				T(0,1), T(1,1), T(2,1), T(1,3)*T(2,1) - T(1,1)*T(2,3), T(0,1)*T(2,3) - T(0,3)*T(2,1), T(0,3)*T(1,1) - T(0,1)*T(1,3),
				T(0,2), T(1,2), T(2,2), T(1,3)*T(2,2) - T(1,2)*T(2,3), T(0,2)*T(2,3) - T(0,3)*T(2,2), T(0,3)*T(1,2) - T(0,2)*T(1,3),
				0, 0, 0, T(0,0), T(1,0), T(2,0),
				0, 0, 0, T(0,1), T(1,1), T(2,1),
				0, 0, 0, T(0,2), T(1,2), T(2,2);
		return ad_ret;
	}	
	SE3 TransInv(const SE3& T) {
		SE3 ret=SE3::Identity();
		ret<<T(0,0), T(1,0), T(2,0), - T(0,0)*T(0,3) - T(1,0)*T(1,3) - T(2,0)*T(2,3),
		 	 T(0,1), T(1,1), T(2,1), - T(0,1)*T(0,3) - T(1,1)*T(1,3) - T(2,1)*T(2,3), 
			 T(0,2), T(1,2), T(2,2), - T(0,2)*T(0,3) - T(1,2)*T(1,3) - T(2,2)*T(2,3),
			 0,0,0,1;
		return ret;
	}	
	
	SO3 TransToR(const SE3& T) {
		return T.block<3,3>(0,0);
	}	
	Vector3d TransToP(const SE3& T) {
		return Vector3d(T(0,3), T(1,3),T(2,3));
	}		

	se3 VecTose3(const Vector6d& V) {
		SE3 m_ret;
		m_ret << 0,-V(5),V(4),V(0),
		         V(5),0,-V(3),V(1),
				-V(4),V(3),0,V(2),
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
		m_ret<<T(0,3),T(1,3),T(2,3),T(2,1),-T(2,0),T(1,0);
		return m_ret;
	}	
	bool NearZero(const double val) {
		return (abs(val) < .000001);
	}	

	SO3 MatrixExp3(const so3& so3mat) {
		Vector3d omgtheta = so3ToVec(so3mat);
		double theta = omgtheta.norm();
		if (NearZero(so3mat.norm())) {
			return SO3::Identity();
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
			return SO3::Identity() + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
		}
	}	


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
			Matrix3d expExpand = Matrix3d::Identity()* theta + (1 - cos(theta)) * omgmat + ((theta - sin(theta)) * (omgmatomgmat));
			Vector3d linear(se3mat(0, 3)/theta, se3mat(1, 3)/theta, se3mat(2, 3)/theta);
			Vector3d GThetaV(expExpand(0,0)*linear(0)+expExpand(0,1)*linear(1)+expExpand(0,2)*linear(2),
			expExpand(1,0)*linear(0)+expExpand(1,1)*linear(1)+expExpand(1,2)*linear(2),
			expExpand(2,0)*linear(0)+expExpand(2,1)*linear(1)+expExpand(2,2)*linear(2));
			m_ret.block<3,3>(0,0) = Matrix3d::Identity() + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
			m_ret.block<3,1>(0,3) = GThetaV;
			return m_ret;
		}
	}

	so3 MatrixLog3(const SO3& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		so3 m_ret =so3::Zero();	
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
			m_ret << SO3::Zero(), p,
				0, 0, 0, 0;
		}
		else {
			double theta = acos((R.trace() - 1) / 2.0);
			Matrix3d logExpand1 = SO3::Identity() - omgmat / 2.0;
			Matrix3d logExpand2 = (1.0 / theta - 1.0 / tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*p,
				0, 0, 0, 0;
		}
		return m_ret;
	}	

	SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = (ROBOT_DOF - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < ROBOT_DOF; i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}	
	Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = Slist;	
		SE3 T = SE3::Identity();
		for (int i = 1; i < ROBOT_DOF; i++) {
			T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetaList(i - 1)));
			Js.col(i) = Ad(T) * Slist.col(i);
		}
		return Js;
	}	
	Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = Blist;
		SE3 T = SE3::Identity();
		for (int i = ROBOT_DOF -2; i >= 0; i--) {
			T *= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * thetaList(i + 1)));
			Jb.col(i) = Ad(T) * Blist.col(i);
		}
		return Jb;
	}		
	SE3 RpToTrans(const Matrix3d& R, const Vector3d& p) {
		SE3 m_ret=SE3::Identity();
		m_ret.block<3,3>(0,0) = R;
		m_ret.block<3,1>(0,3) = p;
		return m_ret;
	}			
	bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinBody(M, Blist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vb = se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vb(3), Vb(4), Vb(5));
		Vector3d linear(Vb(0), Vb(1), Vb(2));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Jb_;
		Eigen::MatrixXd Jb;
		while (err && i < maxiterations) {
			Jb_ = JacobianBody(Blist, thetalist);
			Jb = Eigen::Map<Eigen::MatrixXd>(Jb_.data(),6,ROBOT_DOF);
						
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vb(3), Vb(4), Vb(5));
			linear = Vector3d(Vb(0), Vb(1), Vb(2));
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
		Vector6d Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vs(3), Vs(4), Vs(5));
		Vector3d linear(Vs(0), Vs(1), Vs(2));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Js_;
		Eigen::MatrixXd Js;
		while (err && i < maxiterations) {
			Js_ = JacobianSpace(Slist, thetalist);
			Js = Eigen::Map<Eigen::MatrixXd>(Js_.data(),6,ROBOT_DOF);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vs(3), Vs(4), Vs(5));
			linear = Vector3d(Vs(0), Vs(1), Vs(2));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}	


	JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
									const vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    // the size of the lists
		int n = ROBOT_DOF;

		SE3 Mi = SE3::Identity();
		Matrix6xn Ai = Matrix6xn::Zero();
		vector<Matrix6d> AdTi;
		for (int i = 0; i < n+1; i++) {
			AdTi.push_back(Matrix6d::Zero());
		}
		Matrix6xn_1 Vi = Matrix6xn_1::Zero();    // velocity
		Matrix6xn_1 Vdi = Matrix6xn_1::Zero();   // acceleration

		//Vdi.block(3, 0, 3, 1) = - g;
		Vdi.block(0, 0, 3, 1) = - g;
		AdTi[n] = Ad(TransInv(Mlist[n]));
		Vector6d Fi = Ftip;

		JVec taulist = JVec::Zero();

		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = Ad(TransInv(Mi))*Slist.col(i);
			

			AdTi[i] = Ad(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
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
	
	JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
									const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
	    // the size of the lists
		int n = ROBOT_DOF;

		SE3 Mi = SE3::Identity();
		Matrix6xn Ai = Matrix6xn::Zero();
		vector<Matrix6d> AdTi;
		for (int i = 0; i < n+1; i++) {
			AdTi.push_back(Matrix6d::Zero());
		}
		Matrix6xn_1 Vi = Matrix6xn_1::Zero();    // velocity
		Matrix6xn_1 Vdi = Matrix6xn_1::Zero();   // acceleration

		//Vdi.block(3, 0, 3, 1) = - g;
		Vdi.block(0, 0, 3, 1) = - g;
		AdTi[n] = Ad(TransInv(Mlist[n]));
		Vector6d Fi = Ftip;

		JVec taulist = JVec::Zero();
		Matrix6d Geef = Matrix6d::Identity()*eef_mass;
		Geef.block<3,3>(3,3) = Matrix3d::Zero();
		SE3 Meef = SE3::Identity();
		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = Ad(TransInv(Mi))*Slist.col(i);
			

			AdTi[i] = Ad(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
			          * TransInv(Mlist[i]));
			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}
		Vector6d Aeef = Ad(TransInv(Meef))*Slist.col(n-1);
		Matrix6d AdTeef = Ad( MatrixExp6(VecTose3(Aeef*0))*TransInv(Meef));
		Vector6d Veef= AdTeef* Vi.col(n) + Ai.col(n-1) * 0;
		Vector6d Vdeef= AdTeef* Vdi.col(n) + Ai.col(n-1) * 0+ad(Veef)*Ai.col(n-1)*0;

		// backward pass
		for (int i = n-1; i >= 0; i--) {
			if(i==n-1){
				Fi = AdTeef.transpose()*Fi + Geef*Vdeef-ad(Veef).transpose()*Geef*Veef;
			}
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}	
	JVec GravityForces(const JVec& thetalist, const Vector3d& g,
									const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector6d dummyForce = Vector6d::Zero();
		JVec grav = InverseDynamics(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist);
		return grav;
	}	

	JVec GravityForces(const JVec& thetalist, const Vector3d& g,
									const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
	    int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector6d dummyForce = Vector6d::Zero();
		JVec grav = InverseDynamics(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist,eef_mass);
		return grav;
	}
	
	MassMat MassMatrix(const JVec& thetalist,
                                const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = ROBOT_DOF;
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
	MassMat MassMatrix(const JVec& thetalist,
                                const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
		int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero();
		MassMat M = MassMat::Zero();
		for (int i = 0; i < n; i++) {
			JVec ddthetalist = JVec::Zero();
			ddthetalist(i) = 1;
			M.col(i) = InverseDynamics(thetalist, dummylist, ddthetalist,
                             dummyg, dummyforce, Mlist, Glist, Slist,eef_mass);
		}
		return M;
	}
	JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero(6);
		JVec c = InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
	}	
	
		JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
		int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero(6);
		JVec c = InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist,eef_mass);
		return c;
	}
	JVec EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		JVec JTFtip = InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}
	
		JVec EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const vector<SE3>& Mlist, const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
		int n = ROBOT_DOF;
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		JVec JTFtip = InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist,eef_mass);
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
	JVec ForwardDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
									const vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {

		JVec totalForce = taulist - VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist,eef_mass)
                 							 - GravityForces(thetalist, g, Mlist, Glist, Slist,eef_mass)
                                             - EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist,eef_mass);
		MassMat M = MassMatrix(thetalist, Mlist, Glist, Slist,eef_mass);
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

	void FKinBody(const SE3& M,const ScrewList& Blist, const JVec& q ,const JVec& dq, SE3 &T, Jacobian &Jb,Jacobian& dJb){
		Jb = Blist;
		dJb = Jacobian::Zero();
		T = SE3::Identity();
		SE3 T_ = SE3::Identity();
		Vector6d prev_dJidt=Blist.col(ROBOT_DOF-1)*dq(ROBOT_DOF-1);
		for(int i = ROBOT_DOF-2 ;i >= 0;i--){	
				T_*= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * q(i + 1)));
				Vector6d Jbi = Ad(T_)*Blist.col(i);
				Jb.col(i)=Jbi;
				dJb.col(i) = ad(Jbi)*prev_dJidt;
				prev_dJidt += Jbi*dq(i);			   
		}
		T_*= MatrixExp6(VecTose3(-1 * Blist.col(0) * q(0)));
		T = M*TransInv(T_);
		
	}	
       Jacobian dJacobianBody(const SE3& M,const ScrewList& Blist, const JVec& q ,const JVec& dq){
		Jacobian Jb = Blist;
		Jacobian dJb = Jacobian::Zero();
		SE3  T = SE3::Identity();
		SE3 T_ = SE3::Identity();
		Vector6d prev_dJidt=Blist.col(ROBOT_DOF-1)*dq(ROBOT_DOF-1);
		for(int i = ROBOT_DOF-2 ;i >= 0;i--){	
				T_*= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * q(i + 1)));
				Vector6d Jbi = Ad(T_)*Blist.col(i);
				Jb.col(i)=Jbi;
				dJb.col(i) = ad(Jbi)*prev_dJidt;
				prev_dJidt += Jbi*dq(i);			   
		}
		T_*= MatrixExp6(VecTose3(-1 * Blist.col(0) * q(0)));
		T = M*TransInv(T_);
		return dJb;
		
	}	


	Matrix3d dexp3(const Vector3d& xi) {
		const double eps = std::numeric_limits<double>::epsilon();
		if (xi.norm() < eps) {
			return Matrix3d::Identity();
		}
		
		Matrix3d ceil_xi = VecToso3(xi);
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
		Matrix3d ceil_xi = VecToso3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;
		double gamma = alpha / beta;
		
		Matrix3d dlog = Matrix3d::Identity() - 0.5 * VecToso3(xi) + (1 - gamma) / (norm_xi * norm_xi) * ceil_xi * ceil_xi;
		
		return dlog;
	}
	
	Matrix6d dexp6(const Vector6d& lambda) {
		const double eps = std::numeric_limits<double>::epsilon();

		Vector3d eta = lambda.segment(0, 3);
		Matrix3d ceil_eta = VecToso3(eta);

		Vector3d xi = lambda.segment(3, 3);
		double norm_xi = xi.norm();
		Matrix3d ceil_xi = VecToso3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;

		Matrix3d Cxi;
		if (xi.norm() < eps) {
			Cxi = 0.5 * VecToso3(eta);
		} else {
			Cxi = beta/2 * VecToso3(eta) +
				(1-alpha)/(norm_xi*norm_xi) * (VecToso3(eta)*VecToso3(xi) + VecToso3(xi)*VecToso3(eta)) +
				(alpha-beta)/(norm_xi*norm_xi) * xi.transpose()*eta*VecToso3(xi) -
				1/(norm_xi*norm_xi) * (3*(1-alpha)/(norm_xi*norm_xi) - beta/2) * xi.transpose()*eta*VecToso3(xi)*VecToso3(xi);
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
			return 0.5 * VecToso3(dxi);
		}

		Matrix3d ceil_xi = VecToso3(xi);
		double norm_xi = xi.norm();
		double s = sin(norm_xi / 2.0) / (norm_xi / 2.0);
		double c = cos(norm_xi / 2.0);
		double alpha = s * c;
		double beta = s * s;
		Vector3d eta = dxi;
		double norm_xi2 = norm_xi*norm_xi;
		Matrix3d Cxi = beta/2.0* VecToso3(eta) +
					   (1.0-alpha)/norm_xi2 * skew_sum(eta,xi) +
						(alpha-beta)/norm_xi2 * xi.dot(eta)  * VecToso3(xi) -
						1.0/norm_xi2 * (3.0*(1.0-alpha)/norm_xi2 - beta/2.0) * xi.dot(eta)* ceil_xi * ceil_xi;
		return Cxi;
	}	
	Matrix3d dddexp3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
	const double eps = std::numeric_limits<double>::epsilon();

	if (xi.norm() < eps) {
		return 0.5 * VecToso3(dy) + (1.0/6.0) * (VecToso3(dxi)*VecToso3(y) + VecToso3(y)*VecToso3(dxi));
	}
	Matrix3d dCxi;
	Matrix3d ceil_xi = VecToso3(xi);
	double norm_xi = xi.norm();
	double norm_xi2 = norm_xi * norm_xi;
	Matrix3d ceil_dxi = VecToso3(dxi);
	double s = sin(norm_xi / 2) / (norm_xi / 2);
	double c = cos(norm_xi / 2);
	double alpha = s * c;
	double beta = s * s;
	Matrix3d ceil_dy = VecToso3(dy);
	Matrix3d ceil_y = VecToso3(y);
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
		return VecToso3(a) * VecToso3(b) + VecToso3(b) * VecToso3(a);
	}

Matrix3d ddlog3(const Vector3d& xi, const Vector3d& dxi) {
		const double eps = std::numeric_limits<double>::epsilon();

		if (xi.norm() < eps) {
			return -0.5 * VecToso3(dxi);
		}

		double norm_xi = xi.norm();
		double norm_xi2 = norm_xi * norm_xi;
		Matrix3d ceil_xi = VecToso3(xi);
		double s = sin(norm_xi / 2) / (norm_xi / 2);
		double c = cos(norm_xi / 2);
		double alpha = s * c;
		double beta = s * s;
		double gamma = alpha / beta;

		Matrix3d D = -0.5 * VecToso3(dxi) + (1 - gamma) / norm_xi2 * skew_sum(dxi, xi) +
							1 / norm_xi2 * (1 / beta + gamma - 2) / norm_xi2 * xi.transpose() * dxi * VecToso3(xi) * VecToso3(xi);

		return D;
	}	
Matrix3d dddlog3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
    const double eps = std::numeric_limits<double>::epsilon();

    if (xi.norm() < eps) {
        return -0.5 * VecToso3(dy) + (1.0/12.0) * (VecToso3(dxi)*VecToso3(y) + VecToso3(y)*VecToso3(dxi));
    }

    Matrix3d ceil_xi = VecToso3(xi);
    double norm_xi = xi.norm();
    double norm_xi2 = norm_xi * norm_xi;
    Matrix3d ceil_dxi = VecToso3(dxi);
    double s = sin(norm_xi / 2) / (norm_xi / 2);
    double c = cos(norm_xi / 2);
    double alpha = s * c;
    double beta = s * s;
    Matrix3d ceil_dy = VecToso3(dy);
    Matrix3d ceil_y = VecToso3(y);
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
    Matrix3d dDxi = -0.5 * VecToso3(dy) + 2 / norm_xi2 * (1 - gamma / beta) / norm_xi2 * zeta * ceil_xi2 +
                           Gamma4 * (ceil_dy_xi + ceil_y_dxi) + Gamma5 * delta3;

    return dDxi;
}
Matrix6d dlog6(const Vector6d& lambda) {
    const double eps = std::numeric_limits<double>::epsilon();

    Vector3d eta = lambda.segment(0, 3);
    Vector3d xi = lambda.segment(3, 3);

    double norm_xi = xi.norm();
    Matrix3d ceil_xi = VecToso3(xi);
    double s = sin(norm_xi / 2) / (norm_xi / 2);
    double c = cos(norm_xi / 2);
    double alpha = s * c;
    double beta = s * s;
    double gamma = alpha / beta;

    Matrix3d dlog_3 = dlog3(xi);
    Matrix3d O = Matrix3d::Zero();
    double norm_xixi = norm_xi * norm_xi;
    Matrix3d D = -0.5 * VecToso3(eta) + (1 - gamma) / norm_xixi * (VecToso3(eta) * VecToso3(xi) + VecToso3(xi) * VecToso3(eta)) +
                        1 / norm_xixi * (1 / beta + gamma - 2) / norm_xixi * xi.transpose() * eta * VecToso3(xi) * VecToso3(xi);

    if (norm_xi < eps) {
        D = -0.5 * VecToso3(eta);
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
void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,double t,SE3& T_des,Vector6d& V_des,Vector6d& V_des_dot){
	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
	lambda_0 = Vector6d::Zero();
	lambda_T = se3ToVec(MatrixLog6(TransInv(X0)*XT));
	dlambda_0 = V0;
	dlambda_T = dlog6(-lambda_T)*VT;
	ddlambda_0 = dV0;
	ddlambda_T = dlog6(-lambda_T)*dVT +ddlog6(-lambda_T,-dlambda_T)*VT;
	lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();
	for(int j = 0;j<6;j++){
		Vector3d ret = QuinticTimeScalingKinematics(lambda_0(j),lambda_T(j),dlambda_0(j),dlambda_T(j),ddlambda_0(j),ddlambda_T(j),Tf,t) ;
		lambda_t(j) = ret(0);
		dlambda_t(j) = ret(1);
		ddlambda_t(j) = ret(2);
	}
	V_des = dexp6(-lambda_t)*dlambda_t;
	V_des_dot = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
	T_des = X0*MatrixExp6(VecTose3(lambda_t));
	if (t>=Tf){
		V_des = VT;
		V_des_dot = dVT;
		T_des = XT;

	}
}	
// void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,vector<SE3>&Xd_list,vector<Vector6d>&Vd_list,vector<Vector6d>&dVd_list){
// 	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
// 	lambda_0 = Vector6d::Zero();
// 	lambda_T = se3ToVec(MatrixLog6(TransInv(X0)*XT));
// 	dlambda_0 = V0;
// 	dlambda_T = dlog6(-lambda_T)*VT;
// 	ddlambda_0 = dV0;
// 	ddlambda_T = dlog6(-lambda_T)*dVT +ddlog6(-lambda_T,-dlambda_T)*VT;
// 	double timegap = Tf /(N/1.0 - 1.0);
// 	for (int i = 0;i<N;i++){
// 		lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();
// 		double t= timegap*(i-1);
// 		for(int j = 0;j<6;j++){
// 			Vector3d ret = QuinticTimeScalingKinematics(lambda_0(j),lambda_T(j),dlambda_0(j),dlambda_T(j),ddlambda_0(j),ddlambda_T(j),Tf,t) ;
// 			lambda_t(j) = ret(0);
// 			dlambda_t(j) = ret(1);
// 			ddlambda_t(j) = ret(2);
// 		}
// 		Vector6d V = dexp6(-lambda_t)*dlambda_t;
// 		Vector6d dV = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
// 		SE3 T = X0*MatrixExp6(VecTose3(lambda_t));
// 		Xd_list.push_back(T);
// 		Vd_list.push_back(V);
// 		dVd_list.push_back(dV);
// 	}
// }	

void JointTrajectory(const JVec q0, const JVec qT, double Tf, double t , int method , JVec& q_des, JVec& q_dot_des, JVec& q_ddot_des) {
	if(t>Tf)t = Tf;
	if(t<0) t= 0;
	double st;
	double dst;
	double ddst;
	Vector3d ret = QuinticTimeScalingKinematics(0,1,0,0,0,0,Tf,t) ;
	st= ret(0);
	dst = ret(1);
	ddst= ret(2);
		
	q_des =st * qT + (1 - st)*q0;
	q_dot_des =dst * qT  - dst*q0;
	q_ddot_des =ddst * qT  - ddst*q0;
	
}

	// SE3 RelFKinSpace(const SE3& M, const RelScrewList& Slist, const RelJVec& thetaList) {
	// 	SE3 T = M;
	// 	for (int i = (RELJOINTNUM - 1); i > -1; i--) {
	// 		T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
	// 	}
	// 	return T;
	// }

	// SE3 RelFKinBody(const SE3& M, const RelScrewList& Blist, const RelJVec& thetaList) {
	// 	SE3 T = M;
	// 	for (int i = 0; i < RELJOINTNUM; i++) {
	// 		T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
	// 	}
	// 	return T;
	// }


	// RelJacobian RelJacobianSpace(const RelScrewList& Slist, const RelJVec& thetaList) {
	// 	RelJacobian Js = Slist;	
	// 	SE3 T = SE3::Identity();
	// 	for (int i = 1; i < RELJOINTNUM; i++) {
	// 		T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetaList(i - 1)));
	// 		Js.col(i) = Ad(T) * Slist.col(i);
	// 	}
	// 	return Js;
	// }
	// RelJacobian RelJacobianBody(const RelScrewList& Blist, const RelJVec& thetaList) {
	// 	RelJacobian Jb = Blist;
	// 	SE3 T = SE3::Identity();
	// 	for (int i = RELJOINTNUM -2; i >= 0; i--) {
	// 		T *= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * thetaList(i + 1)));
	// 		Jb.col(i) = Ad(T) * Blist.col(i);
	// 	}
	// 	return Jb;
	// }
	
    //    RelJacobian ReldJacobianBody(const SE3& M,const RelScrewList& Blist, const RelJVec& q ,const RelJVec& dq){
	// 	RelJacobian Jb = Blist;
	// 	RelJacobian dJb = RelJacobian::Zero();
	// 	SE3  T = SE3::Identity();
	// 	SE3 T_ = SE3::Identity();
	// 	Vector6d prev_dJidt=Blist.col(RELJOINTNUM-1)*dq(RELJOINTNUM-1);
	// 	for(int i = RELJOINTNUM-2 ;i >= 0;i--){	
	// 			T_*= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * q(i + 1)));
	// 			Vector6d Jbi = Ad(T_)*Blist.col(i);
	// 			Jb.col(i)=Jbi;
	// 			dJb.col(i) = ad(Jbi)*prev_dJidt;
	// 			prev_dJidt += Jbi*dq(i);			   
	// 	}
	// 	T_*= MatrixExp6(VecTose3(-1 * Blist.col(0) * q(0)));
	// 	T = M*TransInv(T_);
	// 	return dJb;
		
	// }		
	// bool RelIKinSpace(const RelScrewList& Slist, const SE3& M, const SE3& T,
	// 	RelJVec& thetalist, double eomg, double ev) {
	// 	int i = 0;
	// 	int maxiterations = 20;
	// 	SE3 Tfk = RelFKinSpace(M, Slist, thetalist);
	// 	SE3 Tdiff = TransInv(Tfk)*T;
	// 	Vector6d Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
	// 	Vector3d angular(Vs(3), Vs(4), Vs(5));
	// 	Vector3d linear(Vs(0), Vs(1), Vs(2));

	// 	bool err = (angular.norm() > eomg || linear.norm() > ev);
	// 	RelJacobian Js_;
	// 	Eigen::MatrixXd Js;
	// 	while (err && i < maxiterations) {
	// 		Js_ = RelJacobianSpace(Slist, thetalist);
	// 		Js = Eigen::Map<Eigen::MatrixXd>(Js_.data(),6,RELJOINTNUM);
	// 		thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
	// 		i += 1;
	// 		// iterate
	// 		Tfk = RelFKinSpace(M, Slist, thetalist);
	// 		Tdiff = TransInv(Tfk)*T;
	// 		Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
	// 		angular = Vector3d(Vs(3), Vs(4), Vs(5));
	// 		linear = Vector3d(Vs(0), Vs(1), Vs(2));
	// 		err = (angular.norm() > eomg || linear.norm() > ev);
	// 	}
	// 	return !err;
	// }	
	
	// bool RelIKinBody(const RelScrewList& Blist, const SE3& M, const SE3& T,
	// 	RelJVec& thetalist, double eomg, double ev) {
	// 	int i = 0;
	// 	int maxiterations = 20;
	// 	SE3 Tfk = RelFKinBody(M, Blist, thetalist);
	// 	SE3 Tdiff = TransInv(Tfk)*T;
	// 	Vector6d Vb = se3ToVec(MatrixLog6(Tdiff));
	// 	Vector3d angular(Vb(3), Vb(4), Vb(5));
	// 	Vector3d linear(Vb(0), Vb(1), Vb(2));

	// 	bool err = (angular.norm() > eomg || linear.norm() > ev);
	// 	RelJacobian Jb_;
	// 	Eigen::MatrixXd Jb;
	// 	while (err && i < maxiterations) {
	// 		Jb_ = RelJacobianBody(Blist, thetalist);
	// 		Jb = Eigen::Map<Eigen::MatrixXd>(Jb_.data(),6,RELJOINTNUM);
						
	// 		thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
	// 		i += 1;
	// 		// iterate
	// 		Tfk = RelFKinBody(M, Blist, thetalist);
	// 		Tdiff = TransInv(Tfk)*T;
	// 		Vb = se3ToVec(MatrixLog6(Tdiff));
	// 		angular = Vector3d(Vb(3), Vb(4), Vb(5));
	// 		linear = Vector3d(Vb(0), Vb(1), Vb(2));
	// 		err = (angular.norm() > eomg || linear.norm() > ev);
	// 	}
	// 	return !err;
	// }	

}

