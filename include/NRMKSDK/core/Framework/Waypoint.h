//  ---------------------- Doxygen info ----------------------
//! \file Waypoint.h
//!
//! \brief
//! Header file for the class Waypoint (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements waypoints (for teaching) 
//! to be used for the interface of NRMKFoundation library
//! \n
//! \n
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! \n
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date June 2014
//! 
//! \version 1.7.1
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <coolcat@neuromeka.com>

#pragma once 

#include <iostream>
#include <fstream>
#include <string>

#include "LieGroup/LieGroup.h"

namespace NRMKFoundation
{
enum 
{
	// mode for tracing waypoint 
	MODE_JOINT_WAYPOINT = 0,	 //!< mode for tracing joint waypoints
	MODE_TASK_WAYPOINT = 1,  //!< mode for tracing task waypoints
};

//! \class JWaypoint
//!
//! \brief
//! This defines the waypoints for joint position only.
//! 
//! \details 
//!	 Single waypoint is defined in terms of the joint position as well as trajectory duration. 
//!
//! \tparam SubsysType Type of Subsys
//! \tparam _NUM_WAYPOINTS Number of maximum waypoints
template<typename SubsysType, int _NUM_WAYPOINTS>
class JWaypoint
{
public:
	enum 
	{
		NUM_WAYPOINTS = _NUM_WAYPOINTS*2,  //!< number of maximum waypoints taking into account resting at points
	};

public:
	//! \brief constructs a waypopint class
	JWaypoint() 
		:  _index(0)//, _mode(MODE_JOINT_WAYPOINT)
		, _dT_cruise(-1)
		, _dT_rest(0)
		, _restAtPoint(false)

	{
		memset(_dT, -1, sizeof(double)*NUM_WAYPOINTS);
	}

	// Added @20140813
	int read(SubsysType & robot, std::string const & file)
	{
		std::ifstream fs;
		fs.open(file.c_str());

		if (!fs.is_open())
			return -1;

		int numPoints = 0;
		fs >> numPoints;

		for (int i = 0; i < numPoints; i++)
		{
			double dt_cruise = 0;
			double dt_rest = 0;

			fs >> dt_cruise;
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			{
				fs >> robot.q()[k];
//				robot.q()[k] *= DEGREE;
			}

			fs >> dt_rest;

			_restAtPoint = (dt_rest > 0);
			insert(robot, dt_cruise); 
		}

		fs.close();

		return numPoints;
	}

	int save(std::string const & file)
	{
		std::ofstream fs;
		fs.open(file.c_str());

		if (!fs.is_open())
			return -1;

		fs << numWaypoints() << "\n";
		for (int i = 0; i < numWaypoints(); i++)
		{
			fs <<  _dT[i] << "\t";
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				fs << _q[i][k] << "\t";
//				fs << _q[i][k]/DEGREE << "\t";

			fs << 0 << "\n";
		}

		fs.close();

		return numWaypoints();
	}

	//! \brief sets the duration for resting at waypoint
	void setRestTime(double h)
	{
		if (h > 0)
		{
			_dT_rest = h;
			_restAtPoint = true;
		}
		else
		{
			_dT_rest = 0;
			_restAtPoint = false;
		}
	}

	void setCruiseTime(double h)
	{
		if (h > 0)
			_dT_cruise = h;
	}

	//! \brief resets the whole waypoints
	void reset() 
	{
		_index = 0;
	}

	//! \brief removes the last added waypoint
	void remove() 
	{
		if (_index > 0)
			_index--;

		// In case of active restAtPoint, delete one more point except the initial one
		if (_restAtPoint && _index > 0)
			_index--;
	}

	//! \brief insert a new joint waypoint 
	//!
	//! \param robot System object
	//! \param dT trajectory duration time. Default value of -1 implies "not-determined".
	void insert(SubsysType const & robot, double dT = -1) 
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add joint waypoint
		_q[_index] = robot.q();

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest. 
		if (_index > 1 && _restAtPoint)
		{
			_q[_index] = _q[_index - 1];
			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	//! \brief finish adding waypoints and move to the initial waypoint (by joint tracking) 
	//!
	//! \note
	// robot should be updated() before calling this function. 
	//! 
	//! \param robot System object
	//! \param interpolator Joint Interpolator object 	
	//! \param t the current time
	//! 
	//! \tparam JointInterpolatorType Type of Joint interpolator
	template<typename JointInterpolatorType>
	void finish(SubsysType const & robot, JointInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		interpolator.setInitialTraj(t, robot.q());
		interpolator.setTargetTraj(t + _dT[0], _q[0]);
	}

	//! \brief execute joint waypoints
	//!
	//! \note One has to set initial joint position before calling this function, 
	//!		e.g. _jinterpolator.setInitialTraj(_t, _robot.q());
	//! 
	//! \param interpolator Joint Interpolator object 	
	//! \param t the current time
	//! 
	//! \tparam JointInterpolatorType Type of Joint interpolator
	template<typename JointInterpolatorType>
	void executeJointWaypoints(JointInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());

		//interpolator.setInitialTraj(t, _q[0]);

		double tk = t;
		for (int k = 0; k < _index; k++)
		{
			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k]);
		}
	}

	//! \brief return the current number of waypoints
	int numWaypoints() const { return _index; }

	//! \brief return the total duration for executing waypoints
	double totalDuration() const 
	{
		double t = 0;

		// k = 0 is the initial point.
		for (int k = 0; k < numWaypoints(); k++)
			t += _dT[k];

		return t;
	}

	// 	//! \brief return the current number of waypoints
	// 	void setMode(int mode) 
	// 	{ 
	// 		_mode = mode; 
	// 	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//! \brief current number of waypoints
	int _index;

	//! \brief current mode for waypoint tracking
	int _mode;

	//! \brief joint waypoint vector
	typename SubsysType::JointVec _q[NUM_WAYPOINTS];

	//! \brief trajectory time vector
	double _dT[NUM_WAYPOINTS];	

	//! \brief stationary duration at each waypoint
	double _dT_cruise;

	//! \brief stationary duration at each waypoint
	double _dT_rest;

	//! \brief flag to activate intermediate stop
	bool _restAtPoint;
};

//! \class Waypoint
//!
//! \brief
//! This defines the waypoints.
//! 
//! \details 
//!	 Single waypoint is defined in terms of the joint position, task transformation as well as trajectory duration. 
//!
//! \tparam SubsysType Type of Subsys
//! \tparam TaskKinematicsType Type of TaskKinematics
//! \tparam _NUM_WAYPOINTS Number of maximum waypoints
template<typename SubsysType, typename TaskKinematicsType, int _NUM_WAYPOINTS>
class Waypoint
{
public:
	enum 
	{
		NUM_WAYPOINTS = _NUM_WAYPOINTS*2,  //!< number of maximum waypoints taking into account resting at points
	};

public:
	//! \brief constructs a waypopint class
	Waypoint() 
		:  _index(0)//, _mode(MODE_JOINT_WAYPOINT)
		, _t0(0)
		, _dT_cruise(-1)
		, _dT_rest(0)
		, _restAtPoint(false)
	{
		memset(_dT, -1, sizeof(double)*NUM_WAYPOINTS);
	}

	// Added @20140813
	int read(SubsysType & robot, /*int numPoints, */std::string const & file)
	{
		std::ifstream fs;		
		//fs.open("..\\Waypoint\\_Waypoint_" + file +".dat");
		fs.open(file.c_str());

		if (!fs.is_open())
			return -1;

		int numPoints = 0;
		fs >> numPoints;

		for (int i = 0; i < numPoints; i++)
		{
			double dt_cruise = 0;
			double dt_rest = 0;

			fs >> dt_cruise;
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			{
				fs >> robot.q()[k];
//				robot.q()[k] *= DEGREE;
			}

			fs >> dt_rest;

			_restAtPoint = (dt_rest > 0);
			insert(robot, dt_cruise); 
		}

		fs.close();

		return numPoints;
	}

	// Added @20140827
	int read(SubsysType & robot, TaskKinematicsType const & tkin, std::string const & file)
	{
		std::ifstream fs;
		//fs.open("..\\Waypoint\\_Waypoint_" + file +".dat");
		fs.open(file.c_str());

		if (!fs.is_open())
			return -1;

		int numPoints = 0;
		fs >> numPoints;

		for (int i = 0; i < numPoints; i++)
		{

			double dt_rest = 0;
			fs >> _dT[_index];

			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			{
				fs >> _q[_index][k];
			}

			fs >> dt_rest;

			_index++;
		}

		fs.close();

		return numPoints;
	}

	int save(std::string const & file)
	{
		std::ofstream fs;
		//fs.open("..\\Waypoint\\_Waypoint_" + file +".dat");
		fs.open(file.c_str());

		if (!fs.is_open())
			return -1;

		fs << numWaypoints() << "\n";
		for (int i = 0; i < numWaypoints(); i++)
		{
			fs <<  _dT[i] << "\t";
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				fs << _q[i][k] << "\t";
//				fs << _q[i][k]/DEGREE << "\t";

			fs << 0 << "\n";
		}

		fs.close();
		return numWaypoints();
	}

	//! \brief sets the duration for resting at waypoint
	void setRestTime(double h)
	{
		if (h > 0)
		{
			_dT_rest = h;
			_restAtPoint = true;
		}
		else
		{
			_dT_rest = 0;
			_restAtPoint = false;
		}
	}

	void setCruiseTime(double h)
	{
		if (h > 0)
			_dT_cruise = h;
	}

	//! \brief resets the whole waypoints
	void reset() 
	{
		_index = 0;
	}

	//! \brief removes the last added waypoint
	void remove() 
	{
		if (_index > 0)
			_index--;

		// In case of active restAtPoint, delete one more point except the initial one
		if (_restAtPoint && _index > 0)
			_index--;
	}

	//! \brief insert a new joint waypoint 
	//!
	//! \param robot System object
	//! \param dT trajectory duration time. Default value of -1 implies "not-determined".
	void insert(SubsysType const & robot, double dT = -1) 
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add joint waypoint
		_q[_index] = robot.q();

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest. 
		if (_index > 1 && _restAtPoint)
		{
			_q[_index] = _q[_index - 1];
			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	//! \brief insert a new waypoint 
	//!
	//! \note
	// robot should be updated() before calling this function. 
	//! 
	//! \param robot System object
	//! \param tkin Task kinematics object 	
	//! \param dT trajectory duration time. Default value of -1 implies "not-determined".
	void insert(SubsysType const & robot, TaskKinematicsType const & tkin, double dT = -1) 
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add joint waypoint
		_q[_index] = robot.q();

		// add task waypoint
		// This robot should be updated before by calling robot.update(true);
		tkin.kinematics(robot, _p[_index]);

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest. 
		if (_index > 1 && _restAtPoint)
		{
			_q[_index] = _q[_index - 1];
			_p[_index] = _p[_index - 1];

			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	void insert(typename SubsysType::JointVec const & qdes, double dT = -1)
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add joint waypoint
		_q[_index] = qdes;

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest.
		if (_index > 1 && _restAtPoint)
		{
			_q[_index] = _q[_index - 1];
			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	void insert(typename TaskKinematicsType::PosType const & pdes, double dT = -1)
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add task waypoint
		// This robot should be updated before by calling robot.update(true);
		_p[_index] = pdes;

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest.
		if (_index > 1 && _restAtPoint)
		{
			_p[_index] = _p[_index - 1];

			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	void insertJoint(typename SubsysType::JointVec const & qdes, double dT = -1)
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add joint waypoint
		_q[_index] = qdes;

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest.
		if (_index > 1 && _restAtPoint)
		{
			_q[_index] = _q[_index - 1];
			_dT[_index] = _dT_rest;

			_index++;
		}
	}

	void insertTask(typename TaskKinematicsType::PosType const & pdes, double dT = -1)
	{
		if (_index >= NUM_WAYPOINTS)
			return;

		// add task waypoint
		// This robot should be updated before by calling robot.update(true);
		_p[_index] = pdes;

		// add time duration
		_dT[_index] = (dT > 0) ? dT : _dT_cruise;

		// increment index
		_index++;

		// If restAtPoint is activated...
		// Note: At the initial waypoint, it does not rest.
		if (_index > 1 && _restAtPoint)
		{
			_p[_index] = _p[_index - 1];

			_dT[_index] = _dT_rest;

			_index++;
		}
	}


	//! \brief finish adding waypoints and move to the initial waypoint (by joint tracking) 
	//!
	//! \note
	// robot should be updated() before calling this function. 
	//! 
	//! \param robot System object
	//! \param interpolator Joint Interpolator object 	
	//! \param t the current time
	//! 
	//! \tparam JointInterpolatorType Type of Joint interpolator
	template<typename JointInterpolatorType>
	void finish(SubsysType const & robot, JointInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		interpolator.setInitialTraj(t, robot.q());
		interpolator.setTargetTraj(t + _dT[0], _q[0]);
	}

	//! \brief execute joint waypoints
	//!
	//! \note One has to set initial joint position before calling this function, 
	//!		e.g. _jinterpolator.setInitialTraj(_t, _robot.q());
	//! 
	//! \param interpolator Joint Interpolator object 	
	//! \param t the current time
	//! 
	//! \tparam JointInterpolatorType Type of Joint interpolator
#if 0
	template<typename JointInterpolatorType>
	void executeJointWaypoints(JointInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		
		//interpolator.setInitialTraj(t, _q[0]);

		_t0 = t;
		double tk = t;
		for (int k = 0; k < _index; k++)
		{
			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k]);
		}
	}
#elif 0
	template<typename JointInterpolatorType>
	void executeJointWaypoints(JointInterpolatorType & interpolator, double t)
	{
		_t0 = t;
		double tk = t;

		for (int k = 0; k < _index; k++)
		{
			typename SubsysType::JointVec __qPre;
			typename SubsysType::JointVec __qNext;
			typename SubsysType::JointVec __v;
			double __dt = 0;
			if(k == 0)
			{
				__qNext = _q[k+1];
				__dt = _dT[k];
				__v = (__qNext - _q[k])/__dt;
			}
			else
			{
				__qPre = _q[k-1];
				__qNext = _q[k+1];
				__dt = _dT[k] + _dT[k+1];
				__v = (__qNext - __qPre)/__dt;
			}

			///FIXME : 20170320
			for(int i = 0 ; i < 6/*JOINT_DOF*/ ; i++)
			{
				if((k != 0)&&(k != _index-1))
				{
					if( (_q[k][i] - _q[k-1][i]) * (_q[k+1][i] - _q[k][i]) < 0)
						__v[i] = 0;
				}
				if(k == _index-1)
					__v[i] = 0;
			}

			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k], __v);
		}
	}
#else
	template<typename JointInterpolatorType>
	void executeJointWaypoints(JointInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());

		//interpolator.setInitialTraj(t, _q[0]);

		_t0 = t;
		double tk = t;
		for (int k = 0; k < _index; k++)
		{
			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k]);
		}
	}

	template<typename JointInterpolatorType>
	void executeJointWaypoints(typename SubsysType::JointVec const & q0, JointInterpolatorType & interpolator, double t)
	{
		_t0 = t;
		double tk = t;

		if (_index == 1)
		{
			tk += _dT[0];
			interpolator.setTargetTraj(tk, _q[0]);
			return;
		}

		double __dt = 0;
		typename SubsysType::JointVec __qPre;
		typename SubsysType::JointVec __qNext;
		typename SubsysType::JointVec __v;
		for (int k = 0; k < _index; k++)
		{
#if 1
			if(k == 0)
			{
				__qNext = _q[k+1];
				__dt = _dT[k] + _dT[k+1];
//				__v = (__qNext - q0)/__dt;

				for(int i = 0 ; i < 6/*JOINT_DOF*/ ; i++)
				{
					if( (_q[k][i] - q0[i]) * (_q[k+1][i] - _q[k][i]) < 0 )
						__v[i] = 0;
					else
						__v[i] = (__qNext[i] - q0[i])/__dt;
				}
			}
			else if ( k == _index -1)
			{
				for(int i = 0 ; i < 6 /*JOINT_DOF*/ ; i++) __v[i] = 0;
			}
			else
			{
				__qPre = _q[k-1];
				__qNext = _q[k+1];
				__dt = _dT[k] + _dT[k+1];
//				__v = (__qNext - __qPre)/__dt;

				for(int i = 0 ; i < 6/*JOINT_DOF*/ ; i++)
				{
					if( (_q[k][i] - _q[k-1][i]) * (_q[k+1][i] - _q[k][i]) < 0)
						__v[i] = 0;
					else
						__v[i] = (__qNext[i] - __qPre[i])/__dt;
				}
			}

			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k], __v);
#else
			tk += _dT[k];
			interpolator.setTargetTraj(tk, _q[k]);
#endif
		}
	}
#endif

	//! \brief execute task waypoints
	//!
	//! \note One has to set initial joint position before calling this function, 
	//!		e.g. _jinterpolator.setInitialTraj(_t, _robot.q());
	//! 
	//! \param interpolator Task Interpolator object 	
	//! \param t the current time
	//! 
	//! \tparam TaskInterpolatorType Type of Task interpolator
	template<typename TaskInterpolatorType>
	void executeTaskWaypoints(TaskInterpolatorType & interpolator, double t)
	{
		// 		interpolator.setInitialTraj(t, robot.q(), typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// 		interpolator.setTargetTraj(t + dT, _q[0], typename ROBOT::JointVec::Zero(), typename ROBOT::JointVec::Zero());
		// interpolator.setInitialTraj(t, _p[0]);

		_t0 = t;
		double tk = t;
		for (int k = 0; k < _index; k++)
		{
			tk += _dT[k];
			interpolator.setTargetTraj(tk, _p[k]);
		}
	}

	//! \brief return the current number of waypoints
	int numWaypoints() const { return _index; }

	//! \brief return the total duration for executing waypoints
	double totalDuration() const 
	{
		double t = 0;

		// k = 0 is the initial point.
		for (int k = 0; k < numWaypoints(); k++)
			t += _dT[k];

		return t;
	}

	//! \brief return the k-th joint waypoint
	int lastIndex() const 
	{
		if (_index > 1 && _restAtPoint)
			return _index - 2;
		else
			return _index - 1;		
	}

	//! \brief return the k-th joint waypoint
	typename SubsysType::JointVec const & q(int k) const 
	 {
		 return _q[k];
	 }
	
	//! \brief return the k-th task waypoint
	typename TaskKinematicsType::PosType const & p(int k) const 
	 {
		 return _p[k];
	 }

	// 	//! \brief return the current number of waypoints
	// 	void setMode(int mode) 
	// 	{ 
	// 		_mode = mode; 
	// 	}

	void shift(double t)
	{
		int currIdx = 0, k = 0;
		double dtTemp[NUM_WAYPOINTS];

		double tk = _t0;
		for (k = 0; k < _index; k++)
		{
			if ((t >= tk) && (t < (tk+_dT[k])))
				break;

			tk += _dT[k];
		}

		for (int i = k; i < _index; i++)
		{
			dtTemp[currIdx] = _dT[i];
			qTemp[currIdx] = _q[i];
			pTemp[currIdx] = _p[i];
			currIdx++;
		}

		for (int i = 0; i < k; i++)
		{
			dtTemp[currIdx] = _dT[i];
			qTemp[currIdx] = _q[i];
			pTemp[currIdx] = _p[i];
			currIdx++;
		}

		for (int i = 0; i < _index; i++)
		{
			_dT[i] = dtTemp[i];
			_q[i] = qTemp[i];
			_p[i] = pTemp[i];
		}
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//! \brief current number of waypoints
	int _index;

	//! \brief current mode for waypoint tracking
	int _mode;

	//! \brief joint waypoint vector
	typename SubsysType::JointVec _q[NUM_WAYPOINTS];

	//! \brief task waypoint vector
	typename TaskKinematicsType::PosType _p[NUM_WAYPOINTS];

	typename SubsysType::JointVec qTemp[NUM_WAYPOINTS];
	typename TaskKinematicsType::PosType pTemp[NUM_WAYPOINTS];

	double _t0;
	//! \brief trajectory time vector
	double _dT[NUM_WAYPOINTS];	

	//! \brief stationary duration at each waypoint
	double _dT_cruise;

	//! \brief stationary duration at each waypoint
	double _dT_rest;

	//! \brief flag to activate intermediate stop
	bool _restAtPoint;
};

} // namespace NRMKFoundation
