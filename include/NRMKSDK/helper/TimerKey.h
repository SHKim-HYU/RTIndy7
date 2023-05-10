//  ---------------------- Doxygen info ----------------------
//! \file TimerKey.h
//!
//! \brief
//! Header file for the class TimerKey (Internal API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a controller base class
//! to be used for the interface of NRMKFoundation library.
//!  
//! \n
//! \n
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! \n
//! \n
//! Neuromeka Co., Ltd. \n
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
//! \note Copyright (C) 2013-2014 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

namespace NRMKHelper
{

class TimerKey
{
public:
	enum
	{
		Forever = (unsigned int) -1,
		Oneshot = (unsigned long) -1L,
		Continuous = (unsigned long) 0L,
	};

public:
	inline TimerKey() : _key(0), _tick(0L), _period(Oneshot), _run(false), _numExecution(Forever), _numExecuted(0)
	{
	}

	inline ~TimerKey()
	{
		_numTimers--;
	}

	inline void activate(unsigned char key, unsigned long period = Oneshot, unsigned int numExecution = 1) 
	{
		_key = key;
		
		_period = period;
		_tick = 0L;
		
		_numExecution = numExecution;
		_numExecuted = 0;

		// _tick = _period;
		_numTimers++;
	}

	inline void deactivate() 
	{
		_key = 0;

		_period = Oneshot;
		_tick = 0L;

		_numExecuted = _numExecution;

		stop();
		// _tick = _period;
		//_numTimers++;
	}

	inline void begin() 
	{
		_run = true;
		_tick = _period;
	}

	inline void halt()
	{
		_run = !_run;
	}

	inline void run()
	{
		_run = true;
	}

	inline void stop()
	{
		_run = false;
		_tick = _period;
	}

// 	void fire()
// 	{
// 		//_run = !_run;
// 		_tick = _period;
// 	}

	inline unsigned char key() 
	{
		if (!_run)
			return 0; 

		if (_tick == _period)
		{
			_tick = 0L;

			if (_numExecution != Forever)
				_numExecuted++;

			if (_numExecuted == _numExecution)
				stop();

			return _key;
		}
		else 
		{
			if (_period != Oneshot)
				_tick++;

			return 0;
		}
	}

	inline bool completedExecution() const
	{
		return _numExecuted >= _numExecution;
	}

	static inline int numTimers() 
	{
		return _numTimers;
	}

private:
	// FIXED by THACHDO 20150717
	unsigned char _key;
	unsigned long _tick;
	unsigned long _period;
	bool _run;
	unsigned int _numExecution;

	unsigned int _numExecuted;
	
	static int _numTimers;	
};

} // namespace NRMKFoundation
