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
    SE3 EulerT(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf);
    void AngularJacobian(const SO3& R0,const SO3& RT,Matrix3d& Jw0, Matrix3d& JwT);
    so3 LieBracket(const so3& A,const so3& B);
    void DerivativeAngularJacobian(const Matrix3d& Jw0, const Matrix3d& JwT, Matrix3d& Cw0,Matrix3d& CwT);
    Vector3d QuinticTimeScalingKinematics(double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) ;
    SE3 TransInv(const SE3& transform);
    Vector6d EulerVel(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf);
    Vector6d EulerAcc(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf);
     //void EulerKinematics(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd) ;
    void EulerKinematics(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd ,int* order) ;
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
    void EulerKinematicsZYX(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);
    void EulerKinematicsZXY(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);
    void EulerKinematicsYXZ(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);
    void EulerKinematicsYZX(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);
    void EulerKinematicsXYZ(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);
    void EulerKinematicsXZY(const SE3& Xstart, const SE3& Xend, const Vector6d& Vstart, const Vector6d& Vdotend, const Vector6d& Vdotstart, const Vector6d& Vend, double t, double Tf, SE3& Xd, Vector6d& Vd, Vector6d& dVd);

}

