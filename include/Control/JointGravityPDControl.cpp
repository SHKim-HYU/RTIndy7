
#include "JointGravityPDControl.h"

JointGravityPDControl::JointGravityPDControl()
: _robotNom(NULL)
{
}

JointGravityPDControl::~JointGravityPDControl()
{
	if (_robotNom != NULL) delete _robotNom;
}

void JointGravityPDControl::initialize(ROBOT & robot, double delt)
{
	_robotNom = AbstractController::createRobotObj();
}

void JointGravityPDControl::setPassivityMode(bool enable)
{
}

void JointGravityPDControl::reset()
{
}

void JointGravityPDControl::reset(int jIndex)
{
}

void JointGravityPDControl::setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki)
{
}

int JointGravityPDControl::computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque)
{
	
	JointMat kp, kd;
	JointVec tauGrav;
	JointVec tauPD;

	// Initialize local variables
	kp.Zero();
	kd.Zero();
	tauPD.Zero();
	tauGrav.Zero();

	// Set PD control gains
	for (int i = 0; i < JOINT_DOF; ++i)
	{
		switch(i)
		{
		case 0:
		case 1:
			kp(i,i) = 140;
			kd(i,i) = 55;
			break;
		case 2:
			kp(i,i) = 80;
			kd(i,i) = 30;
			break;
		case 3:
		case 4:
			kp(i,i) = 50;
			kd(i,i) = 15;
			break;
		case 5:
			kp(i,i) = 36;
			kd(i,i) = 3;
			break;
		}
	}

	// Gravitational force calculation using inverse dynamics
	robot.idyn_gravity(LieGroup::Vector3D(0,0,-GRAV_ACC));
	tauGrav = robot.tau();

	// Joint-space PD control input
	tauPD = kp*(qDesired - robot.q()) - kd*robot.qdot() + tauGrav;
		
	// Update torque control input
	//torque = tauPD;
	torque = tauGrav;
	
	static unsigned int print_count;
	if(++print_count>4000){
		std::cout<<torque.transpose()<<std::endl;
		std::cout<<tauPD.transpose()<<std::endl;
		print_count = 0;
	}
	
	return 0;
}

int JointGravityPDControl::computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque)
{
    // Compute gravitational torque
    JointVec tauGrav;
    robot.idyn_gravity(LieGroup::Vector3D(0, 0, -GRAV_ACC));
    tauGrav = robot.tau();

    // Update torque control input
    torque = tauGrav;

    return true;
}


typedef JointGravityPDControl JointGravityPDControl;

POCO_BEGIN_MANIFEST(AbstractIndy7JointController<robotIndy7>)
	POCO_EXPORT_CLASS(JointGravityPDControl)
POCO_END_MANIFEST

