/*
 * Kinematics.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: spec
 */

#include "PoEKinematics.h"

namespace HYUMotionKinematics {

    PoEKinematics::PoEKinematics() {
        isInfoUpdated = 0;
        RobotDoF = 6;
    }

    PoEKinematics::PoEKinematics(int DoF)
            : isInfoUpdated(0) {
        RobotDoF = DoF;
    }

    PoEKinematics::~PoEKinematics() {
    }

// LEFT //
    void PoEKinematics::UpdateKinematicInfo(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num) {
        M[0] << 1, 0, 0, 163e-3,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        _M[_link_num - 1][_link_num] = this->GetM(_l);
        v_se3[_link_num - 1] = this->GetTwist(_w, this->GetV(_w, _p));

        if (_link_num == 1) {
            M[_link_num] = _M[_link_num - 1][_link_num];
        } else {
            M[_link_num] = M[_link_num - 1] * _M[_link_num - 1][_link_num];
        }

        A[_link_num] = AdjointMatrix(inverse_SE3(M[_link_num])) * v_se3[_link_num - 1];


        isInfoUpdated = 1;
    }

// RIGHT //
    void PoEKinematics::UpdateKinematicInfo_R(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num) //_l : length of link
    {
        M[0].setIdentity();
        v_se3[0].setZero();
        Exp_S[0].setZero();
        _LinJaco_dot.setZero();
        linjacobian_old.setZero();


        M[_link_num] = this->GetM(_l);
        v_se3[_link_num] = this->GetTwist(_w, this->GetV(_w, _p));  //get screw AXis

        _M[_link_num - 1][_link_num] = M[_link_num - 1].inverse() * M[_link_num]; //MR p.291
        _M[_link_num][_link_num - 1] = M[_link_num].inverse() * M[_link_num - 1];

        A[_link_num] = AdjointMatrix(inverse_SE3(M[_link_num])) * v_se3[_link_num];

        _BaseT << -1, 0, 0, 0,
                0, -1, 0, -0.232,
                0, 0, 1, 1.567,
                0, 0, 0, 1;
//    _EndT<< 1,  0,  0,  -0.00229,
//            0,  1,  0,  0,
//            0,  0,  1,  0.00324,
//            0,  0,  0,  1;
        _EndT << 0, 0, -1, 0,
                1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 0, 1;
        isInfoUpdated = 1;
    }

    Vector3d PoEKinematics::GetV(Vector3d _w, Vector3d _p) {
        return (-SkewMatrix(_w)) * _p;    //MR p.139
    }

    SE3 PoEKinematics::GetM(Vector3d _link) {
        SE3 res;
        res.setIdentity();
        //EE
        //res.block<3, 3>(0, 0) << 1, 0, 0,
        //                                          0, -1, 0,
        //                                          0, 0, 1;

        res.block<3, 1>(0, 3) << _link;
        return res;
    }

    se3 PoEKinematics::GetTwist(Vector3d _w, Vector3d _v) {
        twist;

        twist.head(3) = _w;
        twist.tail(3) = _v;

        return twist;
    }

    void PoEKinematics::HTransMatrix(double q[]) {

        for (int end = 1; end <= ROBOT_DOF; ++end) {
            Exp_S[end] = SE3Matrix(v_se3[end], q[end - 1]);

        }
        q_=Map<VectorXd>(q,ROBOT_DOF);
        /*
        T[0][1] = M[0]*Exp_S[0]*M[1];
        T[0][2] = M[0]*Exp_S[0]*Exp_S[1]*M[2];
        T[0][3] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*M[3];
        T[0][4] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*M[4];
        T[0][5] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*M[5];
        T[0][6] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*Exp_S[5]*M[6];
        */
        T[0][1] = Exp_S[1] * M[1];
        T[0][2] = Exp_S[1] * Exp_S[2] * M[2];
        T[0][3] = Exp_S[1] * Exp_S[2] * Exp_S[3] * M[3];
        T[0][4] = Exp_S[1] * Exp_S[2] * Exp_S[3] * Exp_S[4] * M[4];
        T[0][5] = Exp_S[1] * Exp_S[2] * Exp_S[3] * Exp_S[4] * Exp_S[5] * M[5];
        T[0][6] = Exp_S[1] * Exp_S[2] * Exp_S[3] * Exp_S[4] * Exp_S[5] * Exp_S[6] * M[6];
        //T[0][5] = T[0][5] * _EndT;

//	T[0][6] = Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*Exp_S[5]*M[6];



        for (int i = 1; i < ROBOT_DOF; ++i) {              //MR P.92
            for (int j = i + 1; j <= ROBOT_DOF; ++j)
                T[i][j] = inverse_SE3(T[0][i]) * T[0][j];
        }
        isInfoUpdated = 1;
    }

    se3 PoEKinematics::Twistout(int i) {
        se3 res;
        res = v_se3[i];
        return res;
    }

    Jaco PoEKinematics::SpaceJacobian(void) {
        Jacobian.setZero();

        Jacobian.block<6, 1>(0, 0) = v_se3[1];

        Jacobian.block<6, 1>(0, 1) = AdjointMatrix(Exp_S[1]) * v_se3[2];

        Jacobian.block<6, 1>(0, 2) = AdjointMatrix(Exp_S[1] * Exp_S[2]) * v_se3[3];

        Jacobian.block<6, 1>(0, 3) = AdjointMatrix(Exp_S[1] * Exp_S[2] * Exp_S[3]) * v_se3[4];

        Jacobian.block<6, 1>(0, 4) = AdjointMatrix(Exp_S[1] * Exp_S[2] * Exp_S[3] * Exp_S[4]) * v_se3[5];

        Jacobian.block<6, 1>(0, 5) = AdjointMatrix(Exp_S[1] * Exp_S[2] * Exp_S[3] * Exp_S[4] * Exp_S[5]) * v_se3[6];

        return Jacobian;
    }

    Jaco PoEKinematics::BodyJacobian(void) {
        _SpaceJacobian = SpaceJacobian();
        _BodyJacobian = AdjointMatrix(inverse_SE3(T[0][ROBOT_DOF])) * _SpaceJacobian;
        return _BodyJacobian;
    }

    Jaco PoEKinematics::AnalyticJacobian() {  //////// Angle : Exponential Coordinate, Position : Cartesian
        _AnalyticJacobian.setZero();

        //Angular Portion
        Matrix3d r, A;
        A.setIdentity();
        r=MatrixLog3(RotMat);
        A = A - (1 - cos(r.norm())) / pow(r.norm(), 2) * r + (r.norm() - sin(r.norm())) / pow(r.norm(), 3) * r * r;
        _AnalyticJacobian.block<3, ROBOT_DOF>(0, 0) = A.inverse() * BodyJacobian().block<3, ROBOT_DOF>(0, 0);
        //Linear Portion
        _AnalyticJacobian.block<3, ROBOT_DOF>(3, 0) =
                T[0][ROBOT_DOF].block<3, 3>(0, 0) * BodyJacobian().block<3, ROBOT_DOF>(3, 0);

        return _AnalyticJacobian;
    }

    LinJaco PoEKinematics::LinearJacobian(void) {
        linjacobian.setZero();
/*    Matrix<double,4,4> tmp;
    tmp.setZero();
    tmp=_BaseT*T[0][ROBOT_DOF]*_EndT;
    linjacobian=tmp.block<3,3>(0,0)*BodyJacobian().block<3,ROBOT_DOF>(3,0);*/
        linjacobian = T[0][ROBOT_DOF].block<3, 3>(0, 0) * BodyJacobian().block<3, ROBOT_DOF>(3, 0);
        return linjacobian;
    }

    InvJaco PoEKinematics::Pinv(Jaco _j) {
        InvJaco PJ;
        Jaco J;
        J << _j;

        //moore-penrose inverse
        int m = 0, n = 0;
        m = J.rows();
        n = J.cols();

        if (n > m)//Fat
        {
            PJ = J.transpose() * ((J * J.transpose()).inverse());
        } else if (m > n)//tall
        {
            PJ = ((J.transpose() * J).inverse()) * J.transpose();
        } else if (m == n)//Square
        {
            PJ << J.inverse();
        }

        //QR-Decomposition
/*	CompleteOrthogonalDecomposition<MatrixXf> cod(J);
	cod.setThreshold(1e-5);
	PJ = cod.pseudoInverse();*/

        //SVD


        return PJ;
    }

/*    Matrix<double, 6, 3> PoEKinematics::Pinv(Matrix<double, 3, 6> _j) {
        Matrix<double, 6, 3> PJ;
        Matrix<double, 3, 6> J;
        J << _j;
        int m = 0, n = 0;
        m = J.rows();
        n = J.cols();

        if (n > m)//Fat
        {
            PJ = J.transpose() * ((J * J.transpose()).inverse());
        } else if (m > n)//tall
        {
            PJ = ((J.transpose() * J).inverse()) * J.transpose();
        } else if (m == n)//Square
        {
            PJ << J.inverse();
        }
        return PJ;
    }
*/
    PinvLJaco PoEKinematics::Pinv(LinJaco _j) {
        Matrix<double, ROBOT_DOF, 3> PJ;
        Matrix<double, 3, ROBOT_DOF> J;
        J << _j;
        int m = 0, n = 0;
        m = J.rows();
        n = J.cols();

        if (n > m)//Fat
        {
            PJ = J.transpose() * ((J * J.transpose()).inverse());
        } else if (m > n)//tall
        {
            PJ = ((J.transpose() * J).inverse()) * J.transpose();
        } else if (m == n)//Square
        {
            //PJ << J.inverse();
        }
        return PJ;
    }

    PinvLJaco PoEKinematics::DPI(LinJaco _j) {
        Matrix<double, ROBOT_DOF, 3> PJ;
        Matrix<double, 3, ROBOT_DOF> J;
        J << _j;
        JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
        double p;
        int m = 0, n = 0;
        m = J.rows();
        n = J.cols();

        MatrixXf damp;
        damp.resize(3, 3);
        damp.setIdentity();
        Manipulability(_j);
        Condition_Number(_j);

        if(w>=0.001)
        {
            p=0.0;
        }
        else
        {
            p=0.001;
            p=pow((1-(w/0.001)),2)*p;
        }

        damp = p * damp;

        if (n > m)//Fat
        {
            PJ = J.transpose() * ((J * J.transpose() + damp).inverse());
        } else if (m > n)//tall
        {
            PJ = ((J.transpose() * J).inverse()) * J.transpose();
        } else if (m == n)//Square
        {
            //PJ << J.inverse();
        }
        return PJ;
    }

    InvJaco PoEKinematics::DPI(Jaco _j) {
        Matrix<double, ROBOT_DOF, 6> PJ;
        Matrix<double, 6, ROBOT_DOF> J;
        J << _j;
        JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
        double p;
        int m = 0, n = 0;
        m = J.rows();
        n = J.cols();

        MatrixXf damp;
        damp.resize(6, 6);
        damp.setIdentity();
        Manipulability(_j);
        //Condition_Number(_j);

        if(w>=0.001)
        {
            p=0.0;
        }
        else
        {
            p=0.001;
            p=pow((1-(w/0.001)),2)*p;
        }

        damp = p * damp;

        if (n > m)//Fat
        {
            PJ = J.transpose() * ((J * J.transpose() + damp).inverse());
        } else if (m > n)//tall
        {
            PJ = ((J.transpose() * J).inverse()) * J.transpose();
        } else if (m == n)//Square
        {
            PJ << J.inverse();
        }
        return PJ;
    }

    Vector3d PoEKinematics::ForwardKinematics(void) {
        //Matrix<double,4,4> _tmp;
        //_tmp.setZero();

        if (isInfoUpdated)
            //_tmp=_BaseT*T[0][ROBOT_DOF]*_EndT;
            //RotMat = _tmp.block<3,3>(0,0);
            //return _tmp.block<3,1>(0,3);
            RotMat = T[0][ROBOT_DOF].block<3, 3>(0, 0);
        return T[0][ROBOT_DOF].block<3, 1>(0, 3);
    }

    Vector3d PoEKinematics::GetEulerAngle(void) {
        Vector3d rpy;

/*        rpy(0) = 180/3.141592*atan2(RotMat(2, 1), RotMat(2, 2));
        rpy(1) = -180/3.141592*asin(RotMat(2, 0));
        rpy(2) = 180/3.141592*atan2(RotMat(1, 0) / cos(rpy(1)), RotMat(0, 0) / cos(rpy(1)));*/

        rpy(0) = 180/3.141592*atan2(RotMat(2, 1), RotMat(2, 2));
        rpy(1) = -180/3.141592*atan2(RotMat(2, 0),RotMat(0,0));
        rpy(2) = 180/3.141592*atan2(RotMat(1, 0), RotMat(0, 0));

        if (isInfoUpdated)
            return rpy;
    }

    Vector4d PoEKinematics::GetQuaternion(void){
        //Vector4d quat;  //{w,x,y,z}
        double tr=RotMat.trace();
        double S;
        if(tr>0)
        {
            quat(0)=sqrt(abs(1+tr))/2;
            quat(1)=(RotMat(2,1)-RotMat(1,2))/(4*quat(0));
            quat(2)=(RotMat(0,2)-RotMat(2,0))/(4*quat(0));
            quat(3)=(RotMat(1,0)-RotMat(0,1))/(4*quat(0));
        }
        else if((RotMat(0,0)>RotMat(1,1)) && (RotMat(0,0)>RotMat(2,2)))
        {
            S=sqrt(abs(1+RotMat(0,0)-RotMat(1,1)-RotMat(2,2)))*2;
            quat(0)=(RotMat(2,1)-RotMat(1,2))/S;
            quat(1)=S/4;
            quat(2)=(RotMat(0,1)+RotMat(1,0))/S;
            quat(3)=(RotMat(0,2)+RotMat(2,0))/S;
        }
        else if(RotMat(1,1)>RotMat(2,2))
        {
            S=sqrt(abs(1+RotMat(1,1)-RotMat(0,0)-RotMat(2,2)))*2;
            quat(0)=(RotMat(0,2)-RotMat(2,0))/S;
            quat(1)=(RotMat(0,1)+RotMat(1,0))/S;
            quat(2)=S/4;
            quat(3)=(RotMat(1,2)+RotMat(2,1))/S;
        }
        else
        {
            S=sqrt(abs(1+RotMat(2,2)-RotMat(0,0)-RotMat(1,1)))*2;
            quat(0)=(RotMat(1,0)-RotMat(0,1))/S;
            quat(1)=(RotMat(0,2)+RotMat(2,0))/S;
            quat(2)=(RotMat(1,2)+RotMat(2,1))/S;
            quat(3)=S/4;
        }
        return quat;

    }
    Matrix3d PoEKinematics::Rot(void) {
        if (isInfoUpdated)
            return T[0][ROBOT_DOF].block<3, 3>(0, 0);
    }

    SE3 PoEKinematics::GetTMat(int _i, int _j) {
        if (isInfoUpdated)
            return T[_i][_j];
    }

    double PoEKinematics::Manipulability(LinJaco _J)
    {
        w=sqrt((_J*_J.transpose()).determinant());
        return w;
    }
    double PoEKinematics::Manipulability(Jaco _J)
    {
        w=sqrt((_J*_J.transpose()).determinant());
        return w;
    }

    double PoEKinematics::Condition_Number(LinJaco _J)
    {
        JacobiSVD<MatrixXf> svd(_J, ComputeThinU|ComputeThinV);
        k=svd.singularValues()(0)/svd.singularValues()(2);
        return k;
    }

    LinJaco PoEKinematics::Jacobian_l_dot()
    {
        _LinJaco_dot=(2*0.01-0.001)/(2*0.01+0.001)*_LinJaco_dot+2/(2*0.01+0.001)*(linjacobian-linjacobian_old);
        linjacobian_old=linjacobian;
        return _LinJaco_dot;
    }

    void PoEKinematics::Unflag_isInfoupdate(){
        if(isInfoUpdated)
            isInfoUpdated = 0;
    }

} /* namespace HYUSpesA */
