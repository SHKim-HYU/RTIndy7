/*
 * EcatDataSocket.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: Thach Do
 */
 
#pragma once
#include "NRMKDataSocket.h"

/// TO DO: Define the proper number of (float) data (transmitted via data socket) in the template arguments.
enum
{
	NUM_JOINT = 5,
	NUM_DATASCOPE_SIGNALS = 2 * NUM_JOINT,
};

typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef unsigned int 	UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;
typedef NRMKHelper::NRMKDataSocket<NUM_DATASCOPE_SIGNALS> _ECAT_DATA_SOCKET;

class EcatDataSocket : public _ECAT_DATA_SOCKET
{
public:
	enum
	{
		NUM_DATA = _ECAT_DATA_SOCKET::NUM_DATA,
	};

public:
	inline EcatDataSocket() : _ECAT_DATA_SOCKET(_dataChannelArray, _sizeDataChannelArray)
	{
		_base_freq = 3.0;
		_tick = 0;
		_delT = 0.001;
	}

	void setPeriod(float delT)
	{
		_delT = delT;
	}

	void updateControlData(INT32 * Q, INT32 * Qdot)
	{
		for (int i=0; i < NUM_JOINT; i++)
		{
			q[i] = (float) Q[i];
			qdot[i] = (float) Qdot[i];
		}
	}

private:
	/// TO DO: Implement the data array.
	// The following is the default behavior sending the joint angles followed by the joint velocities.
	// Note that only float data is allowed for valid plot in Data Scope.
	virtual int _updatePacketData(float * const data)
	{
                // Prepare datas
		for (int k = 0; k < NUM_JOINT; k++)
		{
			data[k] = q[k];
			data[k+NUM_JOINT] = qdot[k];
		}

		_tick++;

		// Always return the number of bytes of the updated data packet
		return NUM_DATA;
	}

private:
	/// TO DO: Define the proper data channel format array in NRMKFrameworkSR5.cpp.
	static const unsigned int _dataChannelArray[];

	/// TO DO: Define the size of actual _dataChannelArray in NRMKFrameworkSR5.cpp.
	static const unsigned int _sizeDataChannelArray;

	float _base_freq;
	float _delT;
	long _tick;

	float q[NUM_JOINT];
	float qdot[NUM_JOINT];
};
