#include "ModernRobotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace mr {
	so3 Identity3d=Eigen::Matrix3d::Identity();
	SE3 Identity4d=Eigen::Matrix4d::Identity();
	so3 Zero3d=Eigen::Matrix3d::Zero();
	Matrix6d ad(const Vector6d& V) {
		Matrix3d omgmat = VecToso3(Vector3d(V(0), V(1), V(2)));
		Matrix6d result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Zero3d;
		result.bottomLeftCorner<3, 3>() = VecToso3(Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
	}

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
	SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
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
	SO3 RotInv(const SO3& rotMatrix) {
		return rotMatrix.transpose();
	}
	Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X){
		Vector6d vec=se3ToVec(MatrixLog6(TransInv(X)*Xd));
		return vec;
	}
	SE3 RpToTrans(const Matrix3d& R, const Vector3d& p) {
		SE3 m_ret(4, 4);
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
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
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    // the size of the lists
		int n = thetalist.size();

		SE3 Mi = SE3::Identity();
		Matrix6xn Ai = Matrix6xn::Zero();
		std::vector<ScrewList> AdTi;
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
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    int n = thetalist.size();
		JVec dummylist = JVec::Zero();
		Vector6d dummyForce = Vector6d::Zero();
		JVec grav = InverseDynamics(thetalist, dummylist, dummylist, g,
                                                dummyForce, Mlist, Glist, Slist);
		return grav;
	}	

	MassMat MassMatrix(const JVec& thetalist,
                                const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = thetalist.size();
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

	JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = thetalist.size();
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		Vector6d dummyforce = Vector6d::Zero(6);
		JVec c = InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
	}	
	JVec EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		int n = thetalist.size();
		JVec dummylist = JVec::Zero();
		Vector3d dummyg = Vector3d::Zero();
		JVec JTFtip = InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}
	JVec ForwardDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {

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
		const Vector3d& g, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist,
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
}
