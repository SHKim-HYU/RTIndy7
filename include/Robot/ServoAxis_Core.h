/*
 * ServoAxis.h
*/

#pragma once

#include <limits>	// for numerical limits

// Trapezoidal trajectory interpolation for NRMK EtherLab Configuration Tool
#include "Interpolator/BlendedPolynomialAlgorithm.h"
#include "TrajectoryDataList.h"

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

namespace NRMKHelper
{
	typedef uint32_t UINT32;
	typedef int32_t INT32;
	typedef int16_t INT16;
	typedef uint16_t UINT16;
	typedef uint8_t UINT8;
	typedef int8_t INT8;

	class ServoAxis
	{
	public:
		ServoAxis()
		{
			_t = 0;

			// Physical limits
			_qLimit[0] = PI;
			_qLimit[1] = -PI;

			_qdotLimit[0] = PI;
			_qdotLimit[1] = -PI;

			_tauLimit[0] = 2000;
			_tauLimit[1] = -2000;

			_zeroPos = 0;
			_dirQ = 1;
			_dirTau = 1;

			_pulsePerRevolution = 1;
			_gearRatio = 1;
			_gearEfficiency = 100;
			_tauK = 1;


			_radToCnt = 1;
			_NmToCnt = 1;
			_cntToRad = 1;
			_cntToNm = 1;

			// Trajectory boundary conditions
			_trajMaxVel = 100;
			_trajMaxAcc = 100;
			_trajMaxJerk = 100;

			_trajInitialized = false;
		}

		void setZeroPos(INT32 pos)
		{
			_zeroPos = pos;
		}

		void setTauK(double tau)
		{
			_tauK = tau;
			setConversionConstants();
		}
		
		void setTauADC(int adc)
		{
			_tauADC = adc;
			setConversionConstants();
		}
		double getTauK()		
		{		
			return _tauK;		
		}

		void setGearRatio(int ratio)
		{
			_gearRatio = ratio;
			setConversionConstants();
		}
		void setGearEfficiency(double efficiency)
		{
			_gearEfficiency = efficiency;
			setConversionConstants();
		}

		void setDirQ(int dir)
		{
			_dirQ = dir;
			setConversionConstants();
		}

		void setDirTau(int dir)
		{
			_dirTau = dir;
			setConversionConstants();
		}
		
		void setMaxTorInCnt(INT16 MaxTor)		
		{		
			_tauLimit[0] = abs(MaxTor);		
			_tauLimit[1] = -abs(MaxTor);		
		}

		void setPulsePerRevolution(int ppr)
		{
			_pulsePerRevolution = ppr;
			setConversionConstants();
		}

		void setConversionConstants()
		{
			_radToCnt = (_dirQ * _gearRatio * _pulsePerRevolution) / (PI2);
			_NmToCnt = (_dirTau*_tauADC)/(_tauK*_gearRatio*_gearEfficiency)*100.0;

			_cntToRad = 1.0 /_radToCnt;
			_cntToNm = 1.0 /_NmToCnt;
		}

		bool isLimitReached()
		{
			if ((_q > _qLimit[0]) || (_q < _qLimit[1]))
				return true;

			if ((_qdot > _qdotLimit[0]) || (_qdot < _qdotLimit[1]))
				return true;

			return false;
		}

		void setPosLimits(double qpos, double qneg)
		{
			_qLimit[0] = qpos;
			_qLimit[1] = qneg;
		}

		void setVelLimits(double qdotpos, double qdotneg)
		{
			_qdotLimit[0] = qdotpos;
			_qdotLimit[1] = qdotneg;
		}

		void setTorqueLimits(double taupos, double tauneg)
		{
			_tauLimit[0] = taupos;
			_tauLimit[1] = tauneg;
		}

		void setCurrentPosInCnt(INT32 ActPosInCnt)
		{
			_q = (double) (ActPosInCnt - _zeroPos) * _cntToRad;
		}
		void setCurrentVelInCnt(INT32 ActVelInCnt)
		{
			_qdot = (double) ActVelInCnt * _cntToRad;
		}
		void setCurrentTorInCnt(INT16 ActTorInCnt)
		{
			_tau = (double) ActTorInCnt * _cntToNm;
		}

		void setTarPosInCnt(INT32 TarPosInCnt)
		{
			_trapezoidal.setInitialTraj(_t, _q, _qdot);
			_trapezoidal.setBoundaryCond(_trajMaxVel, _trajMaxAcc);
			_trapezoidal.setTargetTraj(NRMKFoundation::internal::_traj_time_undefined, (double) (TarPosInCnt - _zeroPos) * _cntToRad);
			_trajInitialized = true;
		}
		void setTarVelInCnt(INT32 TarVelInCnt)
		{
			_qdotdes = (double) TarVelInCnt * _cntToRad;
		}
		void setTarTorInCnt(INT16 TarTorInCnt)
		{
			_taudes = (double) TarTorInCnt * _cntToNm;
		}

		void setCurrentTime(double t)
		{
			double _qddotd;

			//_qdes = _q;
			if (_trajInitialized)
				_trapezoidal.traj(t, _qdes, _qdotdes, _qddotd);

			_t = t;
		}

		double getCurrPosInRad()
		{
			return _q;
		}
		double getCurrVelInRad()
		{
			return _qdot;
		}
		double getCurrTorInNm()
		{
			return _tau;
		}

		void setDesPosInRad(double qd)
		{
			_qdes = qd;
		}
		void setDesVelInRad(double qddot)
		{
			_qdotdes = qddot;
		}
		void setDesTorInNm(double taud)
		{
			_taudes = taud;
		}
		void setTarPosInRad(double qtar)
		{
			_qtar = qtar;
		}
		void setTarVelInRad(double qdtar)
		{
			_qdottar = qdtar;
		}
		void setTarAccInRad(double qddtar)
		{
			_qddottar = qddtar;
		}
		void setDesPosInCnt(INT32 DesPosInCnt)
		{
			_qdes = (double) ((DesPosInCnt - _zeroPos)*_cntToRad);
		}
		INT32 getDesPosInCnt()
		{
			return (INT32) (_qdes*_radToCnt + _zeroPos);
		}
		INT32 getDesVelInCnt()
		{
			return (INT32) (_qdotdes*_radToCnt);
		}
		INT16 getDesTorInCnt()
		{
			return (INT16) (_taudes*_NmToCnt);
		}

		double getDesPosInRad()
		{
			return _qdes;
		}
		double getDesVelInRad()
		{
			return _qdotdes;
		}
		double getDesAccInRad()
		{
			return _qddotdes;
		}

		// Trapezoidal trajectory
		void resetTraj()
		{
			_trajInitialized = false;
		}

		bool trajInitialized() const
		{
			return _trajInitialized;
		}

		void setTrajPeriod(double t)
		{
			_trapezoidal.setPeriod(t);
		}

		void setTrajBoundaryCond(double max_vel, double max_acc, double max_jerk = 0)
		{
			_trajMaxVel = max_vel * _cntToRad;
			_trajMaxAcc = max_acc * _cntToRad;
			_trajMaxJerk = max_jerk * _cntToRad;
		}

		// Quintic Trajectory
		void setTrajInitialQuintic()
		{
			_quintic.setInitialTraj(_t, _q, _qdot, 0);
		}
		void setTrajTargetQuintic(double duration)
		{
			_quintic.setTargetTraj(_t+duration, _qtar, _qdottar, 0);
			_trajInitialized = true;
		}
		void TrajQuintic(){
			_quintic.traj(_t, _qdes, _qdotdes, _qddotdes);
			if(_quintic.isArrived())
				_trajInitialized = false;
		}

		void setTrajInitialCond(double t, double pos, double vel)
		{
			_trapezoidal.setInitialTraj(t, pos, vel);
		}

		void setTrajTargetCond(double pos)
		{
			_trapezoidal.setTargetTraj(NRMKFoundation::internal::_traj_time_undefined, pos);
			_trajInitialized = true;
		}

		void traj(double t, double qd, double qdotd, double qddotd)
		{
			_trapezoidal.traj(t, qd, qdotd, qddotd);
		}

		void setDataIn(UINT32 DataIn)
		{
			_dataIn = DataIn;
		}
		UINT32 getDataIn()
		{
			return _dataIn;
		}

		void setDataOut(UINT32 DataOut)
		{
			_dataOut = DataOut;
		}
		UINT32 getDataOut()
		{
			return _dataOut;
		}

	private:
		double _qLimit[2];	// _posLimit[0] = positive limit, _posLimit[1] = negative limit
		double _qdotLimit[2]; 	// _velLimit[0] = positive limit, _velLimit[1] = negative limit
		double _tauLimit[2];	// _torLimit[0] = positive limit, _torLimit[1] = negative limit

		double _radToCnt;	// = Cnt/Rad
		double _NmToCnt;	// = Cnt/Nm

		double _cntToRad;	// = Rad/Cnt
		double _cntToNm;	// = Nr/Cnt

		double _q;			// Current Angle in Radiant
		double _qdot;		// Current Velocity in Radiant
		double _qddot;		// Current Acceleration

		double _qdes;		// Desired Angle in Radiant
		double _qdotdes;	// Desired Vel in Radiant
		double _qddotdes;	// Desired Acc

		double _qtar;		// Target Angle in Radiant
		double _qdottar;	// Target Vel in Radiant
		double _qddottar;	// Target Acc

		double _tau;		// Current Torque in Nm
		double _taudes;		// Desired Torque in Nm

		UINT32 _dataIn;
		UINT32 _dataOut;

		int _dirQ;		// in order to reverse the positive rotation direction
		int _dirTau;	// in order to reverse the positive torque direction

		int _pulsePerRevolution;	// Number of pulses for single revolution
		int _gearRatio;				// Gear ratio (usually greater than 1)
		int _gearEfficiency;			// Gear efficiency (usually 60~75 [%])
		int _tauADC;				// ADC parameter (CORE 100,200=96, CORE 500=48)
		double _tauK;				// torque constant (read from motor catalogue)
		
		//int _tauRatio;				// = 1/_gearRatio

		int _zeroPos;				// the counter at the user-defined zero position

		// For trajectory interpolation
		double _t;
		double _trajMaxVel;
		double _trajMaxAcc;
		double _trajMaxJerk;

		bool _trajInitialized;

		NRMKFoundation::internal::_ClosedLoopTrapezoidalInterpolator _trapezoidal;
		NRMKFoundation::internal::_QuinticPolynomialInterpolator _quintic;
	};
}
