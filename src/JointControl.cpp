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

}

JointControl::~JointControl()
{
	if (_robotNom != NULL) delete _robotNom;
	reset_ctrl = true;


}

void JointControl::initialize(ROBOT & robot, double delt)
{
	_robotNom = AbstractController::createRobotObj();
	    robot.initHinfController(delt);


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
    if (reset_ctrl)
    {
            robot.resetHinfController();
            reset_ctrl = false;
    }

    robot.setHinfControlGain(_kp, _kv, _ki);
    robot.HinfController(gravDir, qDesired, qdotDesired, qddotDesired);
}
void JointControl::tanhVec( JointVec& err){
	for(int i = 0;i<6;i++)err(i) = tanh(err(i) );
}
int JointControl::computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque)
{
	JointMat M, C;
	JointVec g;
	
	DynamicAnalysis::JointDynamics(robot, M, C, g, LieGroup::Vector3D(0,0,-GRAV_ACC));

	//possible to use AbstractRobot6D's library: computeDynamicsParams
	//instead of using DynanmicAnalysis's library: JointDynamics
	//robot.computeDynamicsParams(robot,M,C,g,LieGroup::Vector3D(0,0,-GRAV_ACC));

	JointMat Kp, Kv,Ki;
	JointVec tauCTM;
	JointVec qddot_ref;

	Kp.setZero();
	Kv.setZero();
	Ki.setZero();

	Kp(0,0) = 70*20;
	Kv(0,0) = 55*20;
	Ki(0,0) = 0;

	Kp(1,1) = 70*20;
	Kv(1,1) = 55*20;
	Ki(1,1) = 0;

	Kp(2,2) = 40*20;
	Kv(2,2) = 30*20;
	Ki(2,2) = 0;

	Kp(3,3) = 25*100;
	Kv(3,3) = 10*100;
	Ki(3,3) = 0;

	Kp(4,4) = 25*90;
	Kv(4,4) = 15*90;
	Ki(4,4) = 0;

	Kp(5,5) = 18*1000;
	Kv(5,5) =  10*1000;
	Ki(5,5) = 0;

	JointVec qDesired,qddotDesired,qdotDesired;

	JointVec err=qDesired - robot.q();
	JointVec edot=qdotDesired - robot.qdot();
	//tanhVec(eint);
	JointVec sum_err = Ki*eint;

	qddot_ref = qddotDesired + Kv*edot + Kp*err +sum_err;
	tauCTM = M*qddot_ref + C*robot.qdot() + g;
	eint+=err*dt;
	torque = tauCTM;
	t+=dt;
	static int print_count = 0;
	int timeCount = 4000;
	if(++print_count>timeCount){
		std::cout<<t<<"--err : "<<err.transpose()<<std::endl;
		std::cout<<t<<"--qDesired : "<<qDesired.transpose()<<std::endl;
		print_count = 0;
	}	
}


typedef JointControl JointControl;

POCO_BEGIN_MANIFEST(AbstractJointController<AbstractRobot6D>)
	POCO_EXPORT_CLASS(JointControl)
POCO_END_MANIFEST
