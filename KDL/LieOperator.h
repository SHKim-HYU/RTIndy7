/*
 * LieOperator.h
 *
 *  Created on: Aug 13, 2018
 *      Author: spec
 */

#ifndef LIEOPERATOR_H_
#define LIEOPERATOR_H_

#include <Eigen/Dense>
using namespace Eigen;

typedef Matrix<float, 6, 1> se3;
typedef Matrix<float, 4, 4> SE3;
typedef Matrix<float, 6, 6> Adjoint;
typedef Matrix<float, 6, 6> adjoint;

namespace HYUMotionBase {

class LieOperator {
public:
	LieOperator();
	virtual ~LieOperator();

public:
	bool NearZero(float near);
	SE3 inverse_SE3( SE3 _SE3 );
	Matrix3f SkewMatrix( Vector3f _Vec3 );
	Matrix3f SkewMatrixSquare( Vector3f _Vec3 );
	Adjoint AdjointMatrix( SE3 _SE3 );
	Adjoint AdjointDualMatrix( SE3 _SE3 );
	adjoint adjointMatrix( se3 _se3 );
	adjoint adjointDualMatrix( se3 _se3 );
	Matrix3f MatrixLog3(Matrix3f& R);
	Vector3f so3ToVec(Matrix3f& so3mat);

	SE3 SE3Matrix(se3 _Twist, float _q);
//	Matrix<float, 7, 1> AxisAng6(se3 expc6);
//	Matrix4f MatrixExp6(se3 _s, float _q);
private:


};

} /* namespace HYUMotion */

#endif /* LIEOPERATOR_H_ */
