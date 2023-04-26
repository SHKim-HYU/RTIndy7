/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include <vector>

#include "LieGroup/LieGroup.h"

#include "NRMKHelper.h"

namespace NRMKHelper
{

class NRMKHelper_API PluckerAxis 
{
public:
	PluckerAxis(LieGroup::Displacement const & pos = LieGroup::Displacement(), 
		    LieGroup::Vector3D const & dir = LieGroup::Vector3D(0, 0, 1));

	void set(LieGroup::Displacement const & pos, LieGroup::Vector3D const & dir);
	
	double reciprocal(PluckerAxis const & rhs) const;
	
	double distanceWithSkew(PluckerAxis const & rhs) const;
	double distanceWithParallel(PluckerAxis const & rhs) const;

	void commonNormalWithSkew(PluckerAxis const & rhs, LieGroup::Vector3D & normal, LieGroup::Displacement & foot1, LieGroup::Displacement & foot2) const;
	void commonNormalWithIntersecting(PluckerAxis const & rhs, LieGroup::Vector3D & normal, LieGroup::Displacement & foot) const;
	void commonNormalWithParallel(PluckerAxis const & rhs, LieGroup::Vector3D & normal) const;

	LieGroup::Vector3D const & l() const { return _l; }
	LieGroup::Vector3D const & m() const { return _m; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	LieGroup::Vector3D _l;
	LieGroup::Vector3D _m;
};

class NRMKHelper_API AxisRelationship
{
public:
	enum RELATIONSHIP
	{
		IDENTICAL = 0, 
		PARALLEL,
		INTERSECT,
		SKEW
	};

	RELATIONSHIP test(PluckerAxis const & axis1, PluckerAxis const & axis2, LieGroup::Vector3D const & auxNormal, LieGroup::Displacement const & auxPosition);

	double dist() const { return _dist; }
	double theta() const { return _theta; }
	LieGroup::Displacement const & p1() const { return _p1; }
	LieGroup::Displacement const & p2() const { return _p2; }
	LieGroup::Vector3D const & n() const { return _n; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	double _dist;
	double _theta;
	LieGroup::Displacement _p1;
	LieGroup::Displacement _p2; 
	LieGroup::Vector3D _n;
};

class NRMKHelper_API MDHModeling
{
public:
	MDHModeling(int dof);

	void generate(LieGroup::HTransform const & T0, LieGroup::HTransform const & Tf);
	void addAxis(int k, LieGroup::Displacement const & p = LieGroup::Displacement(), LieGroup::Vector3D const & z = LieGroup::Vector3D(0, 0, 1));
	
	double a(int k) const { return _a[k]; }
	double alpha(int k) const { return _alpha[k]; }
	double d0(int k) const { return _d0[k]; }
	double theta0(int k) const { return _theta0[k]; }

	LieGroup::HTransform const & T0() const { return _T0; }
	//PluckerAxis const & axis(int k) const { return _axis[k]; }

private:
	typedef Eigen::Array<double, 1, Eigen::Dynamic> ParamaterArray;

	int _dof;

	std::vector<PluckerAxis> _axis;	

	//std::vector<JointAxisRelationship::RELATIONSHIP> _relation;

	LieGroup::HTransform _T0; 

	ParamaterArray _a;
	ParamaterArray _alpha;
	ParamaterArray _d0;
	ParamaterArray _theta0;
};

} // namespace NRMKHelper