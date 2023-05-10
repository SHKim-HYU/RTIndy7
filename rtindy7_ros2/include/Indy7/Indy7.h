#ifndef INDY7_SETUP_H
#define INDY7_SETUP_H

#include <ModernRobotics.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
using namespace mr;
class Indy7
{
	    
	int robotId;
	int actuated_joint_num;
	int eef_num;
	int right_eef_num;
	int left_eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;

	vector<int> right_actuated_joint_id;
	vector<string> right_actuated_joint_name;

	vector<int> left_actuated_joint_id;
	vector<string> left_actuated_joint_name;
    
public:
	ScrewList Slist;
	ScrewList Blist;
	SE3 M;

	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;			

    vector<MatrixXd> Glist2;	
	vector<MatrixXd> Mlist2;			

	Indy7(const std::string mr_json_path);
	void MRSetup(const std::string mr_json_path);
	virtual ~Indy7();

};
#endif  //INDY7_SETUP_H