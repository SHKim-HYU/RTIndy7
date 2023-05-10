#pragma once

#include "../eigen/Eigen/Dense"
#include <vector>
#define JOINTNUM 12
namespace mr {
    typedef Eigen::Matrix<double, 1, JOINTNUM> JVec;
    typedef Eigen::Matrix<double, 4, 4> SE3;
    typedef Eigen::Matrix<double, 3, 3> SO3;
    typedef Eigen::Matrix<double, 4, 4> se3;
    typedef Eigen::Matrix<double, 3, 3> so3;
    typedef Eigen::Matrix<double, 6, JOINTNUM> ScrewList;
    typedef Eigen::Matrix<double, 6, JOINTNUM> Jacobian;
    typedef Eigen::Matrix<double, JOINTNUM,6 > pinvJacobian;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;   
    typedef Eigen::Matrix<double, 3, 1> Vector3d;   
    typedef Eigen::Matrix<double, 4, 1> Vector4d;  
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;  

    se3 VecTose3(const Vector6d& V) ;
    so3 VecToso3(const Vector3d& omg);
    Vector3d so3ToVec(const so3& so3mat) ;
    Vector4d AxisAng3(const Vector3d& expc3);
    bool NearZero(const double val);
    SO3 MatrixExp3(const so3& so3mat) ;
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
    Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X);
    so3 MatrixLog3(const SO3& R);
    se3 MatrixLog6(const SE3& T) ;
}

