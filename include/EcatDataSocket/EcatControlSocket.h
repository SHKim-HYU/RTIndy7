/*
 * EcatControlSocket.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Thach Do
 */

#ifndef ECATCONTROLSOCKET_H_
#define ECATCONTROLSOCKET_H_

#pragma once

#include "Poco/Event.h"
#include "NRMKSocketBase.h"

namespace NRMKHelper
{
	template<int _NUM_JOINTS>
	class EcatControlSocket : public NRMKSocketBase
	{
	public:
		enum
		{
			NUM_JOINTS				= _NUM_JOINTS,
			NUM_BYTES_NUMJOINTS		= 1,	// UINT8
			NUM_BYTES_MODEOP		= 1,	// UINT8
			NUM_BYTES_STATUS		= 2,	// UINT16
			NUM_BYTES_ACTPOS		= 4,	// INT32
			NUM_BYTES_ACTVEL		= 4,	// INT32
			NUM_BYTES_ACTTOR		= 2,	// INT16
			NUM_BYTES_DATA_LENGTH	= NUM_BYTES_NUMJOINTS + NUM_JOINTS*(NUM_BYTES_MODEOP+NUM_BYTES_STATUS+NUM_BYTES_ACTPOS+NUM_BYTES_ACTVEL+NUM_BYTES_ACTTOR),

			NUM_BYTES_TARVAL		= 4,	// UINT32
			NUM_BYTES_MAXVEL		= 4,	// UINT32
			NUM_BYTES_MAXACC		= 4,	// UINT32
			NUM_BYTES_MAXJERK		= 4,	// UINT32
			NUM_BYTES_CMD_LENGTH	= NUM_BYTES_NUMJOINTS + NUM_JOINTS*(NUM_BYTES_MODEOP + NUM_BYTES_TARVAL + NUM_BYTES_MAXVEL + NUM_BYTES_MAXACC + NUM_BYTES_MAXJERK),
		};
	public:
		inline EcatControlSocket()
			: NRMKSocketBase()
			, _commandkey(0), _dataReceiveEvent(true), _hasConnection(false)
		{
		}

		// sendKey is used to send key input only
		inline void sendKey(char key)
		{
			//_key = key;
			WriteComm((LPBYTE) &key, 1, INFINITE);
		}

		inline void getKey(char & key)
		{
			if (_hasConnection) {
				_dataReceiveEvent.wait();
				key = _commandkey;
			}
			_commandkey = 0;
		}

		inline int sendMotionData(INT8 const * const ModeOp, UINT16 const * const Status, INT32 const * const ActPos, INT32 const * const ActVel, INT16 const * const ActTor)
		{
			if (!IsOpen() || !_hasConnection)
				return -1;

			unsigned char _numJoints = NUM_JOINTS;
			unsigned char _buf[NUM_BYTES_DATA_LENGTH + 4];
			int index = 0;

			memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);								//2 bytes, START_token
			index += NRMK_SOCKET_TOKEN_SIZE;

			memcpy(_buf + 2, &_numJoints, NUM_BYTES_NUMJOINTS);											//1 byte, num joints
			index += NUM_BYTES_NUMJOINTS;

			for (int i=0; i<NUM_JOINTS; i++)
			{
				memcpy(_buf + index, &ModeOp[i], NUM_BYTES_MODEOP);										// Mode Operation
				index += NUM_BYTES_MODEOP;

				memcpy(_buf + index, &Status[i], NUM_BYTES_STATUS);										// Status Word
				index += NUM_BYTES_STATUS;

				memcpy(_buf + index, &ActPos[i], NUM_BYTES_ACTPOS);										// Actual Position
				index += NUM_BYTES_ACTPOS;
				
				memcpy(_buf + index, &ActVel[i], NUM_BYTES_ACTVEL);										// Actual Velocity
				index += NUM_BYTES_ACTVEL;
				
				memcpy(_buf + index, &ActTor[i], NUM_BYTES_ACTTOR);										// Actual Torque
				index += NUM_BYTES_ACTTOR;
			}

			memcpy(_buf + index, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
			index += NRMK_SOCKET_TOKEN_SIZE;

			// Send Data
			WriteComm(_buf, index, INFINITE);

			return index;
		}

		inline int getMotionData(INT8 * ModeOp, float * TarVal, float * MaxVel, float * MaxAcc, float * MaxJerk)
		{
			if (IsOpen() && _hasConnection) {
				//_dataReceiveEvent.wait();
				if (_commandkey != 1) return 0;

				unsigned char _numJoints = 0;

				int index = 0;

				memcpy(&_numJoints, _cmdbuf + index, NUM_BYTES_NUMJOINTS);
				index += NUM_BYTES_NUMJOINTS;

				for (int i=0; i<_numJoints; i++)
				{
					memcpy(&ModeOp[i], _cmdbuf + index, NUM_BYTES_MODEOP);										// Mode Operation
					index += NUM_BYTES_MODEOP;

					memcpy(&TarVal[i], _cmdbuf + index, NUM_BYTES_TARVAL);										// Target Value
					index += NUM_BYTES_TARVAL;

					memcpy(&MaxVel[i], _cmdbuf + index, NUM_BYTES_MAXVEL);										// Maximum Velocity
					index += NUM_BYTES_MAXVEL;

					memcpy(&MaxAcc[i], _cmdbuf + index, NUM_BYTES_MAXACC);										// Maximum Acceleration
					index += NUM_BYTES_MAXACC;

					memcpy(&MaxJerk[i], _cmdbuf + index, NUM_BYTES_MAXJERK);									// Maximum Jerk
					index += NUM_BYTES_MAXJERK;
				}

				// Reset
				_commandkey = 0;
				return 1;
			}
			return 0;
		}
		inline virtual bool hasConnection() const { return _hasConnection; }
		inline void setConnected()
		{
			_hasConnection = true;
			// to initiate the connection
			sendKey('O');
		}
///???
		inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
		{
			//LockList();

			if (!_hasConnection)
			{
				_hasConnection = true;
				_commandkey = lpBuffer[0]; //first byte is Key
				_dataReceiveEvent.set();
				return;
			}

			if (dwCount == 1)
			{
				if (_hasConnection)
				{
					_commandkey = lpBuffer[0]; //first byte is Key
					_dataReceiveEvent.set();
				}
			}
			else if (dwCount >= 3)
			{
				if (_hasConnection)
				{
					if ((lpBuffer[0] != 'N') || (lpBuffer[1] != 'S'))  return;
					//assert(dwCount == 30);
					_commandkey = 1;
					memcpy(_cmdbuf, &lpBuffer[2], NUM_BYTES_CMD_LENGTH);
					//_dataReceiveEvent.set();
				}
			}
			else
			{
				// wrong format packet
			}

			//UnlockList();
		}

		inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
		{
			switch( uEvent )
			{
			case EVT_CONSUCCESS:
				std::cout << "One client has connected to the Command Server..." << std::endl;
				break;

			case EVT_CONDROP:
				std::cout << "Command Server Connection abandoned " << std::endl;

				_hasConnection = false;
				_commandkey = 0;

				StopComm();

				if (IsServer())
					waitForConnection(0);

				break;

			default:
				NRMKSocketBase::OnEvent(uEvent, lpvData);
				break;
			}
		}

	private:
		//COMMAND
		volatile char _commandkey;
		Poco::Event _dataReceiveEvent;
		//DATA
		//unsigned char _databuf[NUM_BYTES_DATA_LENGTH];
		unsigned char _cmdbuf[NUM_BYTES_CMD_LENGTH];

		volatile bool _hasConnection; // moved from TESTSocketBase
		// Do not try to move this to the base class
		// Nor to delete volatile attribute. (Then release mode does not work properly).
};
} // namespace NRMKHelper


#endif /* ECATCONTROLSOCKET_H_ */
