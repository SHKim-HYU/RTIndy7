#ifndef MR_INDY7_H
#define MR_INDY7_H
#include "include/NRMKSDK/json/json/json.h"
#include "iostream"
#include "modern_robotics.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class MR_Indy7 {
public:
    MR_Indy7();  // Constructor
    MatrixXd Slist;
    MatrixXd Blist;
    MatrixXd M;

	vector<MatrixXd> Glist;	
	vector<MatrixXd> Mlist;	
    VectorXd q;
    VectorXd dq;
    VectorXd ddq;

    VectorXd q_des;
    VectorXd dq_des;
    VectorXd ddq_des;
    

    VectorXd g;
    VectorXd torq;

    MatrixXd Kp;
    MatrixXd Kv;
    MatrixXd Ki;

    void MRSetup();
    void Gravity( double *q, double *toq);
    void ComputedTorqueControl( double *q,double *dq,double *qdes,double *dqdes,  double *toq);
};

#endif // MR_INDY7_H
