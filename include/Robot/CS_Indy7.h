#ifndef CS_INDY7_H
#define CS_INDY7_H


#include "iostream"
#include <Eigen/Dense>
#include <dlfcn.h>

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;
using namespace mr;

class CS_Indy7 {
public:
	CS_Indy7(const string _robotModel);
	~CS_Indy7(){};

	load_casadi_function();


private:
	JVec FD


private:
	string robotModel
	int n_dof;
	
}


    


#endif // CS_INDY7_H