#pragma once

// For joint controller and interpolator
#include "../Controller/PositionController.h"
#include "../Interpolator/Interpolator.h"

namespace NRMKFoundation
{

// USER COMMANDS DEFINITION
#define RCMD_BUTTON         (0x0001)
#define RCMD_USER           (0x1000)
#define RCMD_RESERVED       (0x2000)

// Reserved commands
// servo commands
// #define RESERVED_CMD_SERVO_FIRST		RCMD_RESERVED
// #define RESERVED_CMD_SERVO_ON			(RESERVED_CMD_SERVO_FIRST+1)
// #define RESERVED_CMD_SERVO_OFF			(RESERVED_CMD_SERVO_FIRST+2)
#define RESERVED_CMD_GO_HOME			(RCMD_RESERVED + 3)

// USER COMMAND3
//#define DEFAULT_CMD					(RCMD_USER + 1)

#define USER_CMD_SELECT_JOINTS		(RCMD_USER + 1)
#define USER_CMD_SELECT_KEEP_AXIS	(RCMD_USER + 2)

#define USER_CMD_TASK_CTRL			(RCMD_USER + 10)
#define USER_CMD_TASK_MOVE			(RCMD_USER + 11)


// SERVO STATE
#define SERVO_OFF						0x0000
#define SERVO_ON						0x1000
#define SERVO_HOLD						0x2000

template 
<
	typename SubsysType, 	
	typename DerivedType, 
	typename JointInterpolator = VectorInterpolator<SubsysType::JOINT_DOF, NRMKFoundation::internal::_QuinticPolynomialInterpolator> 
>
class JointControl
{
public:
	typedef typename SubsysType::JointVec JointVec;

	enum
	{
		JOINT_DOF = SubsysType::TOTAL_JOINT_DOF,
	};		

	//! \returns Reference to the derived object 
	inline DerivedType& derived() { return *static_cast<DerivedType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const DerivedType& derived() const { return *static_cast<const DerivedType*>(this); }

public:
	inline JointControl()
		: _robot()
		, _cmode(0)
		, _home()
		, _q0(JointVec::Zero())
		, _qhome(JointVec::Zero())
		, _t(0)
		, _servo(SERVO_OFF)
	{
		_robot.tau().setZero();
	}

	inline ~JointControl()
	{
	}

	inline void setJointGains(JointVec const & kv, JointVec const & kp = JointVec::Zero(), JointVec const & ki = JointVec::Zero())
	{
		_home.setVelGain(kv);
		_home.setPosGain(kp);
		//_home.setIntGain(ki);
	}

// 	inline void setHome(JointVec const & qhome)
// 	{
// 		_qhome = qdhome;
// 	}

	inline void setPeriod(double dT)
	{
		_delT = dT;
		_home.setPeriod(_delT);
	}

// 	inline void initController()
// 	{
// // 		JointController::TaskVec kp;
// // 		JointController::TaskVec kv;
// // 
// // 		kp << 100, 100, 100, 25, 10, 10;
// // 		kv << 20, 20, 20, 10, 4, 4;
// // 
// // 		// 	_home.setPosGain(100);
// // 		// 	_home.setVelGain(20);
// // 		_home.setPosGain(kp);
// // 		_home.setVelGain(kv);
// 
// 		derived().initController();
// 	}

// 	void initSystem(JointVec const & q0, JointVec const & qdot0)
// 	{
// 		reflectJointState(q0, qdot0);
// 
// // 		_robot.q() = q0;
// // 		_robot.qdot() = qdot0;
// // 
// // 		_robot.update();
// // 
// // 		// FIXME: Is it necessary to call this ?
// // 		//derived().initSystem(q0, qdot0);
// 	}

	inline void reflectJointState(JointVec const & q, JointVec const & qdot)
	{
		_robot.q() = q;
		_robot.qdot() = qdot;

		_robot.update();
	}

	JointVec const & tau() const 
	{ 
		return _robot.tau(); 
	}

	inline void kbCommand(int key)
	{
		switch (key)
		{
		case VK_TAB:
			if (_servo == SERVO_ON)
				_servoHold();

			else //if (_servo == SERVO_HOLD)
				_servoOn();

			break;

		case VK_ESCAPE:
			_servoOff();
			break;

		// by default 16 joints can be selected
		case VK_0:
		case VK_1:
		case VK_2:
		case VK_3:
		case VK_4:
		case VK_5:
		case VK_6:
		case VK_7:
		case VK_8:
		case VK_9:
		case VK_A:
		case VK_B:
		case VK_C:
		case VK_D:
		case VK_E:
		case VK_F:
		case VK_G:
			command(USER_CMD_SELECT_JOINTS, key);

		case VK_H:
		case VK_I:
		case VK_ADD:
		case VK_SUBTRACT:
			command(RESERVED_CMD_GO_HOME, key);
			break;

		default:
	//		derived().kbCommand(key);
			break;
		}
	}

	inline int command(const short& cmd, const int& arg = 0)
	{
		if (_servo != SERVO_ON)
			return -1;

		static int joints = 0;
		static int axes = 0;

		switch (cmd)
		{
		case USER_CMD_SELECT_JOINTS:
			if (arg == VK_0)
				joints = 0;
			else
			{
				int joint; 
				if (arg <= VK_9)
					joint = (0x01 << (arg - VK_1));
				else
					joint = (0x01 << (arg - VK_A + 9));

				if (joints & joint)
					joints &= ~joint;
				else
					joints |= joint;
			}

			break;

		case RESERVED_CMD_GO_HOME:
			{
				JointVec qhome = _robot.q();	

				_jinterpolator.setInitialTraj(_t + _delT, qhome, JointVec::Zero(), JointVec::Zero());

				switch (arg)
				{
				case VK_I:
					qhome.setZero();
					break;

				case VK_H:
					qhome = _qhome; //.setConstant(90*DEGREE);
					break;

				case VK_ADD:
				case VK_SUBTRACT:
					{
						static const double increment = 30*DEGREE;

						for (int i = 0; i < JOINT_DOF; i++)
						{
							int joint = (0x01 << i);
							if (joints & joint)
								qhome[i] += (arg == VK_ADD) ? increment : -increment;
						}
					}
					break;
				}			

				_jinterpolator.setTargetTraj(_t + _delT + 1, qhome, JointVec::Zero(), JointVec::Zero());

				_cmode = 1;
				axes = 0;
			}

			break;

		default:
			break;
		}

	//	derived().command(cmd, arg);

		return 0;
	}

	inline void compute(double t)
	{
		JointVec  qdotref;
		JointVec  qddotref;

		switch (_cmode)
		{
		case 0:	
			_home.refAcceleration(_robot.q(), _robot.qdot(), _q0, JointVec::Zero(), JointVec::Zero(), qdotref, qddotref);
			_robot.idyn(_robot.qdot(), qddotref, LieGroup::Vector3D(0, 0, -GRAV_ACC));

			break;

		case 1: // joint control
			{
				JointVec  qd;
				JointVec  qdotd;
				JointVec  qddotd;

				_jinterpolator.traj(t, qd, qdotd, qddotd);

				_home.refAcceleration(_robot.q(), _robot.qdot(), qd, qdotd, qddotd, qdotref, qddotref);
			}

			_robot.idyn(_robot.qdot(), qddotref, LieGroup::Vector3D(0, 0, -GRAV_ACC));

			break;

		default: // task control 
			break;
		}	

	//	derived().compute(t);
	}

protected:
// 	inline void _estimate()
// 	{
// 		derived()._estimate();
// 	}

	//void _reflect();
	


	//void _arrangeJointDevices();

	inline void _servoOn() 
	{ 
		_servo = SERVO_ON; 
//		derived()._servoOn();
	}

	inline void _servoOff() 
	{ 
		_servo = SERVO_OFF; 
//		derived()._servoOff();
	}

	inline void _servoHold() 
	{ 
		_servo = SERVO_HOLD; 
//		derived()._servoHold();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	SubsysType			_robot; // nominal robot model

	int					_cmode; // = 0 None
								// = 1 joint control
								// = 2 task control
	
	//JointController _home;
	PositionController<SubsysType::JOINT_DOF> _home;
	JointInterpolator _jinterpolator;

	JointVec _q0;	// initial configuration
	JointVec _qhome; // home configuration

	double	_delT;
	double	_t;

	int _servo;
};

} // namespace NRMKFoundation
