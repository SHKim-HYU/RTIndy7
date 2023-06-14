#ifndef MR_DualArm_H
#define MR_DualArm_H

#include "jsoncpp/json/json.h"
#include "iostream"
#include "modern_robotics.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
class MR_DualArm {
public:
typedef Eigen::Matrix<double, 12, 1> relJVec;
typedef Eigen::Matrix<double, 6, 12> relScrewList;

    MR_DualArm();  // Constructor
    void MRSetup();
    relScrewList Slist;
    relScrewList Blist;
    mr::SE3 M;
    mr::SE3 Tbr;
    mr::SE3 Tbl;
};

#endif // MR_DualArm_H
