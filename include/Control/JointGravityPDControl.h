#ifndef JointGravityPDControl_H_
#define JointGravityPDControl_H_

#include <AbstractIndy7JointController.h>
#include <Robot/robotIndy7.h>

class JointGravityPDControl : public AbstractIndy7JointController<robotIndy7>
{
	enum
	{
		JOINT_DOF = robotIndy7::JOINT_DOF
	};
	typedef AbstractIndy7JointController<robotIndy7> AbstractController;
	typedef robotIndy7 ROBOT;
	typedef typename ROBOT::JointVec JointVec;
	typedef typename ROBOT::JointMat JointMat;

public:
	JointGravityPDControl();
	~JointGravityPDControl();

	void initialize(ROBOT & robot, double delt);

	void setPassivityMode(bool enable);
	void setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki);

	void reset();
	void reset(int jIndex);

	int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque);
	int computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque);

private:
	ROBOT * _robotNom;
};

#endif /* JointGravityPDControl_H_ */

