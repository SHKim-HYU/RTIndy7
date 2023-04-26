/* NRMKFoundation, Copyright 2014- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKSocketBase.h"
//#include "Eigen/Eigen"

namespace NRMKHelper
{
	template<int _NUM_CMD_INT_DATA, int _NUM_CMD_FLOAT_DATA, int _NUM_MSG_INT_DATA, int _NUM_MSG_FLOAT_DATA>
	class NRMKPendantSocketBase : public NRMKSocketBase
	{
	public:
		enum
		{
			NUM_CMD_INT_DATA = _NUM_CMD_INT_DATA,
			NUM_CMD_FLOAT_DATA = _NUM_CMD_FLOAT_DATA,
			NUM_MSG_INT_DATA = _NUM_MSG_INT_DATA,
			NUM_MSG_FLOAT_DATA = _NUM_MSG_FLOAT_DATA,

			NUM_BYTES_CMD_DATA = NUM_CMD_INT_DATA*sizeof(int) + NUM_CMD_FLOAT_DATA*sizeof(float),
			NUM_BYTES_MSG_DATA = NUM_MSG_INT_DATA*sizeof(int) + NUM_MSG_FLOAT_DATA*sizeof(float),

			NUM_BYTES_CMD_BUF = NUM_BYTES_CMD_DATA + 3*sizeof(int) + 2*NRMK_SOCKET_TOKEN_SIZE + 1,
			NUM_BYTES_MSG_BUF = NUM_BYTES_MSG_DATA + sizeof(float) + 2*sizeof(int) + 2*NRMK_SOCKET_TOKEN_SIZE + 1,
		};

	public:
		inline NRMKPendantSocketBase()
			: NRMKSocketBase()	
			, _hasConnection(false)
			, _msgAvailable(false)
			, _cmdAvailable(false)
			, _msgAvailableEvent(true)
			, _clientCount(0)
		{
		}

		// Added by HoTam, 20141006
		inline int getClientCount(void)
		{
			return _clientCount;
		}

		inline int sendCommand(int command,  int iCount, int fCount, int const * const idata = NULL, float const * const fdata = NULL)
		{
			if (!IsOpen())
				return -1;

			int cur = 0;		
			
			memcpy(_commandbuf + cur, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	 // 2 bytes, START_token
			cur += NRMK_SOCKET_TOKEN_SIZE;
			
			BYTE packet_type = NRMK_SOCKET_PACKET_PENDANT_CMD;
			memcpy(_commandbuf + cur, &packet_type, 1);				// 1 byte, Packet type
			cur += 1;

			memcpy(_commandbuf + cur, &command, sizeof(int));	// 4 byte, command 			
			cur += sizeof(int);

			cur += _fillData(_commandbuf + cur, iCount, fCount, idata, fdata); 

			memcpy(_commandbuf + cur, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	// 2 bytes, END_token
			cur += NRMK_SOCKET_TOKEN_SIZE;

			// Send Data
			WriteComm(_commandbuf, cur, INFINITE);

			return cur;
		}

		inline int sendMessage(float time, int iCount, int fCount, int const * const idata = NULL, float const * const fdata = NULL)
		{
			if (!IsOpen())
				return -1;

			int cur = 0, tcur=0;

			memcpy(_msgbuf + cur, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);								// 2 bytes, START_token
			cur += NRMK_SOCKET_TOKEN_SIZE;

			BYTE packet_type = NRMK_SOCKET_PACKET_PENDANT_MSG;
			memcpy(_msgbuf + cur, &packet_type, 1);															// 1 byte, Packet type
			cur += 1;

			memcpy(_msgbuf + cur, &time, sizeof(float));																	// 1 byte, command key			
			cur += sizeof(float);

			tcur=cur;
			cur += _fillData(_msgbuf + tcur, iCount, fCount, idata, fdata);

			memcpy(_msgbuf + cur, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	// 2 bytes, END_token
			cur += NRMK_SOCKET_TOKEN_SIZE;

			// Send Data
			WriteComm(_msgbuf, cur, INFINITE);

			return cur;
		}

		inline int getCommand(int & command, int & iCount, int & fCount, int const * & idata, float const * & fdata) 
		{
			if (_hasConnection && _cmdAvailable) 
			{
				int cur = 0;

				command = *(int const * const)  (_commandbuf + cur);
				cur += sizeof(int);

				_getData(_commandbuf + cur, iCount, fCount, idata, fdata);

				_cmdAvailable = false;

				return 0;
			}
			else
			{
				idata = NULL;
				fdata = NULL;

				return 1; 
			}
		}

		inline int getMessage(float & time, int & iCount, int & fCount, int const * & idata, float const * & fdata)
		{
			if (_hasConnection && _msgAvailable) 
			{
				int cur = 0;
				
				time = *(float const * const) (_msgbuf + cur);
				cur += sizeof(float);

				_getData(_msgbuf + cur, iCount, fCount, idata, fdata);
				
				_msgAvailable = false;

				//printf("%f\n", time);
				return 0;
			}
			else
			{
				idata = NULL;
				fdata = NULL;

				return 1; 
			}
		}	

		inline bool msgAvailable()  
		{
			_msgAvailableEvent.wait();
			return _msgAvailable;
		}	

		inline bool cmdAvailable() const 
		{
			return _cmdAvailable;
		}

		inline virtual bool hasConnection() const 
		{ 
			return _hasConnection; 
		}

		inline void setConnected() 
		{ 
			_hasConnection = true; 
			// to initiate the connection
			//sendKey('O'); 	
			char key = 'O';
			WriteComm((LPBYTE) &key, 1, INFINITE);
		}

		inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
		{
			//LockList();

			if (!_hasConnection)
			{
				char key = lpBuffer[0];
				//printf("RequestKey: %i\n", RequestKey);
				switch (key)
				{
				case 'O':
					if (!_hasConnection)
						_hasConnection = true;
					
					break;
				}

				return;
			}

			//
			char startToken[2];				
			memcpy(startToken, lpBuffer, NRMK_SOCKET_TOKEN_SIZE);	// start token				
			//if ((startToken[0] != 'N') || (startToken[1] != 'S'))  return;
			if (memcmp(startToken, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE) != 0) 
				return;
				
			unsigned char packet_type;		
			memcpy(&packet_type, &lpBuffer[2], 1); 	// packet type
		
			if (packet_type == NRMK_SOCKET_PACKET_PENDANT_MSG)
			{
				assert(dwCount >= NUM_BYTES_MSG_BUF);
				memcpy(_msgbuf, &lpBuffer[dwCount-NUM_BYTES_MSG_BUF+3], NUM_BYTES_MSG_BUF - 3 - 2);
				_msgAvailable = true;
				_msgAvailableEvent.set();

				/*
				float tt1, tt2;
				memcpy(&tt1, &lpBuffer[3], 4);
				memcpy(&tt2, _msgbuf, 4);
				printf("%d, %d\n", dwCount, NUM_BYTES_MSG_BUF);
				printf("%f, %f\n", tt1, tt2);
				*/
			}
			else if (packet_type == NRMK_SOCKET_PACKET_PENDANT_CMD)			
			{
				assert(dwCount >= NUM_BYTES_CMD_BUF);
				memcpy(_commandbuf, &lpBuffer[3], NUM_BYTES_CMD_BUF - 3 - 2);

				_cmdAvailable = true;
			}

		}	

		inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
		{
			switch( uEvent )
			{
			case EVT_CONSUCCESS:
				++_clientCount;	// Added by Ho Tam, 20141006
				std::cout << "One client has connected to the Pendant Server..." << std::endl;
				break;

			case EVT_CONDROP:
				std::cout << "Pendant Server Connection abandoned " << std::endl;
				
				// Added@20140822
				_hasConnection = false;
				_msgAvailable = false;
				_cmdAvailable = false;

				--_clientCount; // Added by Ho Tam, 20141006


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
		inline int _fillData(BYTE * const buf, int iCount, int fCount, int const * const idata = NULL, float const * const fdata = NULL)
		{
			int cur = 0;

			memcpy(buf + cur, &iCount, sizeof(int));
			cur += sizeof(int);

			memcpy(buf + cur, &fCount, sizeof(int));
			cur += sizeof(int);

			if (iCount > 0 && idata)
			{
				int bytes = iCount*sizeof(int);
				memcpy(buf + cur, idata, bytes);
				cur += bytes;
			}

			if (fCount > 0 && fdata) 
			{
				int bytes = fCount*sizeof(float);
				memcpy(buf + cur, fdata, bytes);
				cur += bytes;
			}

			return cur;
		}

		inline void _getData(BYTE const * const buf, int & iCount, int & fCount, int const * & idata, float const * & fdata) const
		{
			int cur = 0;

			iCount = *(int const * const) (buf + cur);
			cur += sizeof(int);

			fCount = *(int const * const) (buf + cur);
			cur += sizeof(int);

			if (iCount > 0)
			{
				idata = (int const *) (buf + cur);
				cur += iCount * sizeof(int);
			}

			if (fCount > 0)
			{
				fdata = (float const *) (buf + cur);
			}
		}
		
	private:
		// COMMAND
		unsigned char _commandbuf[NUM_BYTES_CMD_BUF];	// 	DOF + KEY + POS1 + POS2 + SPEED
		
		// MSG
		unsigned char _msgbuf[NUM_BYTES_MSG_BUF]; 			// DOF + TIME + POS + VEC + TAU
		
		volatile bool _hasConnection; // moved from NRMKSocketBase
		// Do not try to move this to the base class
		// Nor to delete volatile attribute. (Then release mode does not work properly).

		// FIXED by THACHDO 20150717
		// For sync		
		volatile bool _msgAvailable;
		volatile bool _cmdAvailable;
		Poco::Event _msgAvailableEvent;

		volatile int _clientCount; // Added by Ho Tam, 20141006
};

template<int JOINT_DOF>
class NRMKAnimationSocket : public NRMKPendantSocketBase<0, 0, 0, JOINT_DOF>
{
public:
	inline int getJointPos(float & time, float const * & q)  
	{
		int iCount, fCount;
		int const * idata = NULL;

		if (msgAvailable())
		{
			return getMessage(time, iCount, fCount, idata, q);
		}
		else
			return 1;
		//assert(iCount == 0);
		//assert(fCount == _JOINT_DOF);
	}

	inline int setJointPos(float time, float const * const q)
	{
		sendMessage(time, 0, JOINT_DOF, NULL, q);

		return 0;
	}

public:
	using NRMKPendantSocketBase<0, 0, 0, JOINT_DOF>::sendMessage;
	using NRMKPendantSocketBase<0, 0, 0, JOINT_DOF>::getMessage;
	using NRMKPendantSocketBase<0, 0, 0, JOINT_DOF>::msgAvailable;
	using NRMKPendantSocketBase<0, 0, 0, JOINT_DOF>::cmdAvailable;

};

template<int JOINT_DOF>
class NRMKControlStateSocket : public NRMKPendantSocketBase<0, 0, 0, 3*JOINT_DOF>
{
public:
	inline int getControlState(float & time, float const * & x)  
	{
		int iCount, fCount;
		int const * idata = NULL;

		if (msgAvailable())
		{
			return getMessage(time, iCount, fCount, idata, x);
		}
		else
			return 1;
		//assert(iCount == 0);
		//assert(fCount == _JOINT_DOF);
	}

	inline int setControlState(float time, float const * const x)
	{
		sendMessage(time, 0, 3*JOINT_DOF, NULL, x);

		return 0;
	}

public:
	using NRMKPendantSocketBase<0, 0, 0, 3*JOINT_DOF>::sendMessage;
	using NRMKPendantSocketBase<0, 0, 0, 3*JOINT_DOF>::getMessage;
	using NRMKPendantSocketBase<0, 0, 0, 3*JOINT_DOF>::msgAvailable;
	using NRMKPendantSocketBase<0, 0, 0, 3*JOINT_DOF>::cmdAvailable;
};

} // namespace NRMKHelper
