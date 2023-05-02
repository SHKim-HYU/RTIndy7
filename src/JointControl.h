#ifndef JointControl_H_
#define JointControl_H_

#include <NRMKFramework/Components/AbstractJointController.h>
#include <NRMKFramework/Components/AbstractRobot6D.h>
#include <Indy/DynamicAnalysis6D.h>
#include <NRMKFramework/Indy/SharedMemory/SharedData.h>


class JointControl : public AbstractJointController<AbstractRobot6D>
{
	enum
	{
		JOINT_DOF = AbstractRobot6D::JOINT_DOF
	};
	typedef AbstractJointController<AbstractRobot6D> AbstractController;
	typedef AbstractRobot6D ROBOT;
	typedef typename ROBOT::JointVec JointVec;
    typedef typename ROBOT::JointMat JointMat;
	typedef NRMKFoundation::IndySDK::DynamicAnalysis6D DynamicAnalysis;


public:
	JointControl();
	~JointControl();

	void initialize(ROBOT & robot, double delt);

	void setPassivityMode(bool enable);
	void setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki);

	void reset();
	void reset(int jIndex);

	int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque);
	int computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque);
	void tanhVec( JointVec& err);

private:
	ROBOT * _robotNom;

    bool reset_ctrl;         // for controller reset
	double t;
	double dt;

    JointVec _kp, _kv, _ki;  // for control gain settings
	JointVec eint;
	
};

#endif /* JointControl_H_ */
