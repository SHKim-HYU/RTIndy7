/***** License Information *****/
#define USERNAME "NeuromekaDev"
#define EMAIL "dev@neuromeka.com"
#define SERIAL "nrmk13766"
/******************************/

#include "JointControl.h"

JointControl::JointControl()
: AbstractController(USERNAME, EMAIL, SERIAL)
, _robotNom(NULL)
{
	t=0;
	dt = 0.00025;
	traj_flag=0;

}

JointControl::~JointControl()
{
	if (_robotNom != NULL) delete _robotNom;
	reset_ctrl = true;


}

void JointControl::initialize(ROBOT & robot, double delt)
{
	//_robotNom = AbstractController::createRobotObj();
	 //   robot.initHinfController(delt);
}

void JointControl::setPassivityMode(bool enable)
{
}

void JointControl::reset()
{
	    reset_ctrl = true;

}

void JointControl::reset(int jIndex)
{
}

void JointControl::setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki)
{
	    _kp = kp;
    _kv = kv;
    _ki = ki;
}

int JointControl::computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque)
{
   return 0;
}
void JointControl::tanhVec( JointVec& err){
	for(int i = 0;i<6;i++)err(i) = tanh(err(i) );
}
int JointControl::computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque)
{
	return 0;
}


typedef JointControl JointControl;

POCO_BEGIN_MANIFEST(AbstractJointController<AbstractRobot6D>)
	POCO_EXPORT_CLASS(JointControl)
POCO_END_MANIFEST
