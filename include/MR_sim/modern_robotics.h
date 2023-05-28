#pragma once

#include <Eigen/Dense>
#include <vector>
#define JOINTNUM 6
namespace mr {
    typedef Eigen::Matrix<double, JOINTNUM, 1> JVec;
    typedef Eigen::Matrix<double, 4, 4> SE3;
    typedef Eigen::Matrix<double, 3, 3> SO3;
    typedef Eigen::Matrix<double, 4, 4> se3;
    typedef Eigen::Matrix<double, 3, 3> so3;
    typedef Eigen::Matrix<double, 6, JOINTNUM> ScrewList;
    typedef Eigen::Matrix<double, 6, JOINTNUM> Jacobian;
    typedef Eigen::Matrix<double, JOINTNUM,6 > pinvJacobian;
    typedef Eigen::Matrix<double, 6*JOINTNUM, JOINTNUM> DerivativeJacobianVec;
    typedef Eigen::Matrix<double, 6*JOINTNUM, 1> vecJVec;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;   
    typedef Eigen::Matrix<double, 3, 1> Vector3d;   
    typedef Eigen::Matrix<double, 4, 1> Vector4d;  
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;  
    typedef Eigen::Matrix<double, 6, JOINTNUM> Matrix6xn;
    typedef Eigen::Matrix<double, 6, JOINTNUM+1> Matrix6xn_1;
    typedef Eigen::Matrix<double, JOINTNUM, JOINTNUM> MassMat;
    Vector6d se3ToVec(const se3& T);
    Matrix6d ad(const Vector6d&V);
    se3 VecTose3(const Vector6d& V) ;
    so3 VecToso3(const Vector3d& omg);
    Vector3d so3ToVec(const so3& so3mat) ;
    Vector4d AxisAng3(const Vector3d& expc3);
    bool NearZero(const double val);
    SO3 MatrixExp3(const so3& so3mat);
    SE3 MatrixExp6(const se3& se3mat);//11us
    Vector3d Normalize(Vector3d V);
    SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList);
    Matrix6d Adjoint(const SE3& T);
    pinvJacobian pinvJacobianBody(const ScrewList& Blist, const JVec& thetaList);
    Vector3d TransToP(const SE3& T);
    SO3 TransToR(const SE3& T);
    Vector3d TransToP(const SE3& T);
    Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList);
    pinvJacobian pinvJacobianBody(const ScrewList& Blist, const JVec& thetaList) ;
    Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) ;
    Jacobian AnalyticJacobianBody(SE3 M,const ScrewList& Blist, const JVec& thetaList);
    Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X,double dt);
    Vector6d VelSE3toSE3(const SE3 &Xd,const SE3 &X,double dt);
    so3 MatrixLog3(const SO3& R);
    se3 MatrixLog6(const SE3& T) ;
    SE3 RpToTrans(const Matrix3d& R, const Vector3d& p) ;
    SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList);
    SO3 RotInv(const SO3& rotMatrix) ;
    bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev);
    bool IKinSpace(const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev);
    JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    JVec GravityForces(const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) ;                                    
    MassMat MassMatrix(const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) ;                                    
    JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) ;
    JVec EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
	JVec ForwardDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist);    
    void EulerStep(JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt);                                    
    JVec ComputedTorque(const JVec& thetalist, const JVec& dthetalist, const JVec& eint,
		const Vector3d& g, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist,
		const ScrewList& Slist, const JVec& thetalistd, const JVec& dthetalistd, const JVec& ddthetalistd,
		double Kp, double Ki, double Kd);
    double CubicTimeScaling(double Tf, double t);
	  double CubicTimeScalingDot(double Tf, double t);   
    double CubicTimeScalingDdot(double Tf, double t);   
    double QuinticTimeScaling(double Tf, double t);
    double QuinticTimeScalingDot(double Tf, double t);
    double QuinticTimeScalingDdot(double Tf, double t);
    JVec Desired_q(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method);
    JVec Desired_dq(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method) ;
    JVec Desired_ddq(const JVec& thetastart, const JVec& thetaend, double t, double Tf, int method) ;
    SE3 ScrewT(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method);
    Vector6d ScrewTwist(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method);
    Vector6d ScrewTwistDot(const SE3& Xstart, const SE3& Xend, double t, double Tf, int method);
    SE3 CartesianT(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method);
    Vector6d CartesianVel(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method);
    Vector6d CartesianAcc(const SE3& Xstart, const SE3& Xend, double t , double Tf, int method);
    void rotm2eulm(SO3 R,SO3& Rz,SO3& Ry,SO3& Rx);
    so3 LieBracket(const so3& A,const so3& B);
    Vector3d QuinticTimeScalingKinematics(double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) ;
    SE3 TransInv(const SE3& transform);
    
    Vector6d AccVeltoVel(const Vector6d &Vd,const Vector6d &V,double dt);
    DerivativeJacobianVec DerivativeVectorizeJacobianBody(const ScrewList& Blist, const JVec& thetaList);
    vecJVec vec(const JVec& thetaList);
    DerivativeJacobianVec DerivativeVectorizeAnalyticJacobianBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList);
    Jacobian dJacobianBody(const Jacobian& Jb ,const JVec& dthetaList);
    Jacobian dAnalyticJacobianBody(const SE3&M, const ScrewList& Blist, const JVec& thetaList ,const JVec& dthetaList);
    void JointTrajectory(const JVec q0, const JVec qT, double Tf, double t , int method , JVec& q_des, JVec& dq_des, JVec& ddq_des);
    double wrapTo2PI(double angle);
    double wrapToPI(double angle);
    JVec wrapTo2PI(const JVec& angles);
    JVec wrapToPI(const JVec& angles);
    Vector6d CartesianError(const SE3& X,const SE3& Xd );
    pinvJacobian pinvAnalyticJacobianBody(SE3 M, const ScrewList& Blist, const JVec& thetaList) ;
  void FkinBody(SE3 M,ScrewList Blist, const JVec& q ,const JVec& dq, SE3 &T, Jacobian &Jb,Jacobian& dJb);
  Matrix3d dexp3(const Vector3d& xi);
  Matrix3d dlog3(const Vector3d& xi);
  Matrix3d skew3(const Vector3d& xi) ;
  Matrix6d dexp6(const Vector6d& lambda);
  Matrix3d ddexp3(const Vector3d& xi, const Vector3d& dxi);
  Matrix3d dddexp3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
  Matrix6d ddexp6(const Vector6d& lambda, const Vector6d& lambda_dot);
  Matrix3d skew_sum(const Vector3d& a, const Vector3d& b);
  Matrix3d ddlog3(const Vector3d& xi, const Vector3d& dxi);
  Matrix3d dddlog3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
  Matrix6d dlog6(const Vector6d& lambda);
  Matrix6d ddlog6(const Vector6d& lambda, const Vector6d& lambda_dot) ;
  void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,std::vector<SE3>&Xd_list,std::vector<Vector6d>&Vd_list,std::vector<Vector6d>&dVd_list);
}


