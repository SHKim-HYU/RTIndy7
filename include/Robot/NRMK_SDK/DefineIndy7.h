/*
 * DefineIndy7.h
 */

#include "LieGroup/LieGroup.h"
#include <stdio.h>
#include <Indy/Indy6D.h>
#include <Poco/ClassLibrary.h>
#include <DefineConstant.h>
#include <Framework/CompositeHTransformTask.h>
#include "MDHModeling.h"
#include "XML/Markup.h"

typedef NRMKFoundation::IndySDK::IndyBase6D IndyBase6D;

class defineIndy7 : public IndyBase6D
{
public:
	enum
	{
		REF_BODY = -1, // -1 stands for grounded base system
		TARGET_BODY = 6,
	};
	typedef Eigen::Matrix<int, JOINT_DOF, 1> StateVec;	//!< Typedef of state vector
	typedef Eigen::Matrix<double, 2, 1> Vector2D;
	typedef NRMKFoundation::IndySDK::IndyBase6D::JointMat JointMat;
	typedef NRMKFoundation::IndySDK::IndyBase6D::JointVec JointVec;
	
	typedef NRMKFoundation::CompositeHTransformTaskPosition TaskPosition;
	
	typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskVelocity;
	
	typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskAcceleration;

	typedef Eigen::Matrix<double, 6, 1> TaskVec;		//!< Typedef of task vector
	typedef Eigen::Matrix<double, 6, 6> TaskJacobian;	//!< Typedef of task matrix

public:
	defineIndy7()
    : IndyBase6D()
    , _qhome(JointVec::Zero())
    {
    	using namespace LieGroup;
		HTransform Offset[NUM_JOINTS];

		Offset[0].set(Rotation(1,0,0,0,1,0,0,0,1),	Displacement( 0.00000000, 0.00000006, 0.07400000));
		Offset[1].set(Rotation(0,1,0,0,0,1,1,0,0),	Displacement( 0.00000000, 0.08500000, 0.14600000));
		Offset[2].set(Rotation(1,0,0,0,1,0,0,0,1),	Displacement( 0.38000000, 0.00000000,-0.00700000));
		Offset[3].set(Rotation(0,0,1,1,0,0,0,1,0),	Displacement( 0.12300000, 0.00000000,-0.07800000));
		Offset[4].set(Rotation(0,1,0,0,0,1,1,0,0),	Displacement( 0.00000000, 0.06500000, 0.29700000));
		Offset[5].set(Rotation(0,0,1,1,0,0,0,1,0),	Displacement( 0.10700000, 0.00000000,-0.06500000));

		for(int i = 0 ; i < NUM_JOINTS ; i++)
			_joint[i].setJointFrame(Offset[i]);

//		double damping[NUM_TOTAL_JOINTS]  	= { 0 };

		for (int k = 0; k < NUM_BODIES; k++)
			addBody(k, _body[k]);

		connect(-1, 0, _joint0);

		for (int k = 1; k < NUM_BODIES; k++)
			connect(k - 1, k, _joint[k - 1]);


		update();
		_setToolProperties();

		NRMKFoundation::Inertia inertia[NUM_BODIES];

		double mass_cad[NUM_BODIES] = {0.50223663, 1.04741291, 0.87519000, 0.63327075, 0.46587016, 0.36036121, 0.06342095};
		double mass_act[NUM_BODIES] = {1.7, 8.2, 2.6, 4.4, 1.5, 2.6, 0.06};
		double mratio[NUM_BODIES];

		for (int k = 0 ; k < NUM_BODIES ; k++)
			mratio[k] = 1.8 * mass_act[k] / mass_cad[k];

		/// Inertia values for robot
		inertia[0].set(mratio[0]*0.50223663 , 0.00241275, -0.00062885, 0.02728383, 0.00113268*mratio[0], 0.00108773*mratio[0], 0.00191617*mratio[0], 0.00001074*mratio[0], 0.00000338*mratio[0], -0.00001418*mratio[0]);
		inertia[1].set(mratio[1]*1.04741291, -0.00007052, 0.01866594, 0.17020009, 0.00754106*mratio[1], 0.00627459*mratio[1], 0.00423543*mratio[1], 0.00000579*mratio[1], 0.00099611*mratio[1], -0.00000220*mratio[1]);
		inertia[2].set(mratio[2]*0.87519000, 0.00000159, 0.12534821, 0.38875263 + 0.2, 0.01786426*mratio[2], 0.01818226*mratio[2], 0.00140325*mratio[2], 0.00000011*mratio[2], -0.00064134*mratio[2], -0.00000112*mratio[2]);
		inertia[3].set(mratio[3]*0.63327075, -0.00002098, 0.01625117, 0.63980951 + 0.1, 0.00335866*mratio[3], 0.00263495*mratio[3], 0.00190939*mratio[3], -0.00000244*mratio[3], -0.00043164*mratio[3], -0.00000004*mratio[3]);
		inertia[4].set(mratio[4]*0.46587016, 0.00005118, 0.05908018, 0.85628136 + 0.1, 0.00585870*mratio[4], 0.00522781*mratio[4], 0.00129356*mratio[4], 0.00000080*mratio[4], 0.00149168*mratio[4], 0.00000247*mratio[4]);
		inertia[5].set(mratio[5]*0.36036121, 0.00000702, 0.01328063, 1.05607647, 0.00132168*mratio[5], 0.00100181*mratio[5], 0.00070744*mratio[5], 0.00000002*mratio[5], -0.00017721*mratio[5], 0.00000009*mratio[5]);
#if 1
		inertia[6].set(mratio[6]*0.06342095 + 3, 0.00004392, 0.00000611, 1.15331413, 0.00003621*mratio[6], 0.00003619*mratio[6], 0.00006089*mratio[6], 0.00000000*mratio[6], 0.00000000*mratio[6], 0.00000000*mratio[6]);
#else
		inertia[6].set(mratio[6]*0.06342095, 0.00004392, 0.00000611, 1.15331413, 0.00003621*mratio[6], 0.00003619*mratio[6], 0.00006089*mratio[6], 0.00000000*mratio[6], 0.00000000*mratio[6], 0.00000000*mratio[6]);
#endif
		for (int k = 0; k < NUM_BODIES; k++)
			_body[k].addInertia(inertia[k], _body[k].T().inverse());

		if (!setJointHome())
			_defineJointHome();

		_defineSettingData();
    }
    ~defineIndy7(){}

    inline void setInitialConfiguration()
	{
	}
	
	bool setJointHome()
	{
		printf("Home Position Set\n");

		CMarkup xmlConfiguration; // Object for XML

		if (!xmlConfiguration.Load("/home/user/release/RobotSpecs/HomePosIndy5.xml")) //Load XML file
			return false;

		xmlConfiguration.FindElem("HomePosition");
		xmlConfiguration.IntoElem();

		while (xmlConfiguration.FindElem("homepos"))
		{
			int _axisIdx = atoi(xmlConfiguration.GetAttrib("index").c_str());
			float hPos = atof(xmlConfiguration.GetAttrib("value").c_str());
			_qhome[_axisIdx] = hPos*DEGREE;
		}

		return true;
	}

	float const toolWeight()	const	{	return (float)_tWeight;	};
	float const toolXcom()		const	{	return (float)_tXcom;	};
	float const toolYcom()		const	{	return (float)_tYcom;	};
	float const toolZcom()		const	{	return (float)_tZcom;	};

private:
	void _defineJointHome()
	{
//		_qhome << 0, -45, -90, 0, 45, 0;
		_qhome << 0, -21.36, -127, 0, 60.25, -30;
		_qhome *= DEGREE;
	}

	void _defineSettingData()
	{
	}

	void _setToolProperties()
	{
		CMarkup xmlConfiguration; // Object for XML

		xmlConfiguration.Load("/home/user/release/RobotSpecs/ToolPropertiesIndy5.xml"); //Load XML file

		xmlConfiguration.FindElem("ToolProperties");
		xmlConfiguration.IntoElem();

		while (xmlConfiguration.FindElem("properties"))
		{
			_tWeight = atof(xmlConfiguration.GetAttrib("weight").c_str());
			_tXcom = atof(xmlConfiguration.GetAttrib("xcom").c_str());
			_tYcom = atof(xmlConfiguration.GetAttrib("ycom").c_str());
			_tZcom = atof(xmlConfiguration.GetAttrib("zcom").c_str());
		}
	}   

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	NRMKFoundation::RigidJoint	_joint0;

	NRMKFoundation::RevoluteJoint _joint[NUM_TOTAL_JOINTS];

	NRMKFoundation::Body _body[NUM_BODIES];

	StateVec _qState;
	StateVec _qBrake;
	JointVec _qhome;

	JointVec _pulsePerRev;
	JointVec _qDirection;
	JointVec _tDirection;
	JointVec _ratedTorque;

	float _gearRatio;

	LieGroup::HTransform _Ttarget;
	LieGroup::HTransform _Tref;

	LieGroup::HTransform _Tft;

	LieGroup::HTransform _Ttask_init;

	double _tWeight, _tXcom, _tYcom, _tZcom;

}