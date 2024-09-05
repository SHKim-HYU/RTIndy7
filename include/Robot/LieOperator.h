/*
 * LieOperator.h
 *
 *  Created on: Aug 13, 2018
 *      Author: spec
 */

#ifndef LIEOPERATOR_H_
#define LIEOPERATOR_H_

#include <Eigen/Dense>
#include "PropertyDefinition.h"
using namespace Eigen;
using namespace std;

class LieOperator {
public:
	LieOperator();
	virtual ~LieOperator();

public:
	bool NearZero(double near);
	SE3 inverse_SE3( SE3 _SE3 );
	Matrix3d SkewMatrix( Vector3d _Vec3 );
	Matrix3d SkewMatrixSquare( Vector3d _Vec3 );
	Matrix6d AdjointMatrix( SE3 _SE3 );
	Matrix6d AdjointDualMatrix( SE3 _SE3 );
	Matrix6d adjointMatrix( Twist _Twist );
	Matrix6d adjointDualMatrix( Twist _Twist );
	Matrix3d MatrixLog3(Matrix3d& R);
	Vector3d so3ToVec(Matrix3d& so3mat);

	SE3 SE3Matrix(Twist _Twist, double _q);
//	Matrix<double, 7, 1> AxisAng6(Twist expc6);
//	Matrix4d MatrixExp6(Twist _s, double _q);
private:


};

#endif /* LIEOPERATOR_H_ */
