/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKSocketBase.h"
//#include "NRMKSyncObject.h"

namespace NRMKHelper
{

class NRMKKeyCommandSocket : public NRMKSocketBase
{
public:
	inline NRMKKeyCommandSocket()
		: NRMKSocketBase()	
		, _hasConnection(false), _requestKey(0)
	{
	}

	// sendKey is used to send key input only
	inline void sendKey(unsigned char key) 
	{
		//_key = key;
		WriteComm((LPBYTE) &key, 1, INFINITE);
	}

	inline void getKey(unsigned char & key) 
	{
		key = (_hasConnection) ? _requestKey : 0;
		_requestKey = 0;
	}

	inline virtual bool hasConnection() const { return _hasConnection; }
	inline void setConnected() 
	{ 
		_hasConnection = true; 
		// to initiate the connection
		sendKey('O'); 	
	}

	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{
		//LockList();

		if (!_hasConnection)
		{
			_requestKey = lpBuffer[0];
			//printf("RequestKey: %i\n", RequestKey);
			switch (_requestKey)
			{
			case 'O':
				if (!_hasConnection)
				{
					_hasConnection = true;
					_requestKey = 0;
				}

				break;
			}

			return;
		}

		if (dwCount == 1)
		{
			_requestKey = lpBuffer[0]; //first byte is Key
		}
		else 
		{
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
			
			// Added@20140822
			_hasConnection = false;
			_requestKey = 0;

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
	//COMMAND _command;

	// FIXED by THACHDO 20150717
	volatile bool _hasConnection; // moved from NRMKSocketBase
	volatile unsigned char _requestKey;
	
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).
};

template<typename COMMAND>
class NRMKCommandSocket : public NRMKKeyCommandSocket
{
public:
	enum
	{
		NUM_BYTES_COMMAND_BUF = sizeof(COMMAND), 
	};

public:
	inline NRMKCommandSocket()
		: NRMKKeyCommandSocket()	
	{
	}
	
	inline int getCommand(COMMAND & command) 
	{
		//LockList();
		memcpy(&command, &_command, NUM_BYTES_COMMAND_BUF);
		//UnlockList();

		return 0; 
	}

	inline int sendCommand(COMMAND const & command)
	{
		if (!IsOpen())
			return -1;
		
		//LockList();
		//_command = command;
		WriteComm((LPBYTE) &command, NUM_BYTES_COMMAND_BUF, INFINITE);
		//UnlockList();

		return NUM_BYTES_COMMAND_BUF;	
	}

	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{

		//LockList();

		if (!_hasConnection)
		{
			_requestKey = lpBuffer[0];
			//printf("RequestKey: %i\n", RequestKey);
			switch (_requestKey)
			{
			case 'O':
				if (!_hasConnection)
				{
					_hasConnection = true;
					_requestKey = 0;
				}

				break;
			}

			return;
		}

		if (dwCount == 1)
		{
			_requestKey = lpBuffer[0]; //first byte is Key
		}
		else 
		{
			// FIXME
			if (IsServer())
			{ 

			}
			else
			{
				assert (dwCount > NUM_BYTES_COMMAND_BUF);
				memcpy(&_command, lpBuffer, NUM_BYTES_COMMAND_BUF);
			}
		} 

		//UnlockList();
	}	

private:
	COMMAND _command;
	
	//volatile char _requestKey;
	//volatile bool _hasConnection; // moved from NRMKSocketBase
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).
};


} // namespace NRMKHelper
