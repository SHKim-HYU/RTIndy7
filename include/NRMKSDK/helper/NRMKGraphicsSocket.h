/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKSocketBase.h"
#include "NRMKGObject.h"

namespace NRMKHelper
{
template <typename SubsysType, int _NUM_GOBJECTS, int _MAX_PACKET_BYTES>
class NRMKGraphicsSocket : public NRMKSocketBase
{
public:
	enum 
	{
		NUM_GOBJECTS = _NUM_GOBJECTS,
		MAX_PACKET_BYTES = _MAX_PACKET_BYTES,
	};

public:
	inline NRMKGraphicsSocket(SubsysType & robot)
		: NRMKSocketBase(), _sys(robot), _numUpdatedGObjects(0)
		, _hasConnection(false), _requestKey(0)
	{
		_initBuf();
	}

	inline virtual bool hasConnection() const { return _hasConnection; }

	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{
		_requestKey = lpBuffer[0];
		//printf("RequestKey: %i\n", RequestKey);
		switch (_requestKey)
		{
		case 'O':
			//if (!_hasConnection)
			{
				init();
				//Sleep(100); //should wait for finishing sendBodyIndexArray

				// temporary
				//_hasConnection = true;
			}

			break;

		case 'R': // for acknowledgment
			if (!_hasConnection)
			{
				_hasConnection = true;
				// now ready to run...
			}

			break;


		/// TO DO: Define custom keyboard handler
		}
	}

	inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
	{
		switch( uEvent )
		{
		case EVT_CONSUCCESS:
			std::cout << "One client has connected to the Custom Graphics Server..." << std::endl;
			break;

		case EVT_CONDROP:
			std::cout << "Custom Graphics Server Connection abandoned " << std::endl;

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

	inline void init(int numUpdatedGObjects = 0, int num_bytes = 0, unsigned char const * const data = NULL)
	{
		// initialize every GObjects as many as NUM_GOBJECTS
		// make socket data beginning at _buf + 7 and update offset
		_numUpdatedGObjects = 0;
		int offset = _initPacketData(_buf + PACKET_DATA_OFFSET + sizeof(unsigned int));

		if (data)
			memcpy(_buf + PACKET_DATA_OFFSET + sizeof(unsigned int) + offset, data, num_bytes);

		_numUpdatedGObjects += numUpdatedGObjects;
		offset += num_bytes;
	
		_sendData(_numUpdatedGObjects, offset/*, _buf + 7*/); 
	}

	inline void update(int numUpdatedGObjects = 0, int num_bytes = 0, unsigned char const * const data = NULL)
	{
		_numUpdatedGObjects = 0;

		// update GObjects and _numUpdatedGObjects
		// make socket data beginning at _buf + 7 and update offset
		int offset = _updatePacketData(_buf + PACKET_DATA_OFFSET + sizeof(unsigned int));
		
		if (numUpdatedGObjects > 0 && num_bytes > 0)
		{
			if (data)
				memcpy(_buf + PACKET_DATA_OFFSET + sizeof(unsigned int) + offset, data, num_bytes);

			_numUpdatedGObjects += numUpdatedGObjects;
			offset += num_bytes;
		}

		if (hasConnection())
			_sendData(_numUpdatedGObjects, offset/*, _buf + 7*/); 
	}

private:
	inline int _sendData(int numObject, int packetLength)
	{
		if (!IsOpen())
			return -1;

		if (numObject == 0)
			return 0;

		//memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		//_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_GRAPHICS_DATA;			//1 unsigned char, Packet type
		
		memcpy(_buf + PACKET_DATA_OFFSET, &numObject, sizeof(unsigned int));			//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)

		// FIXME @ 20130902 Is the following correct?
		// data is already prescribed in init/updateGObjects()
		//memcpy(buf + PACKET_DATA_OFFSET, data, packetLength);				//nLen bytes, data
// 		memcpy(_buf + 7 + packetLength, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
// 
// 		memcpy(_buf + packetLength + 9, "\r\n", 2);

		// FIXME @20131016 The following two cannot be handled here due to variable packet data length.
		// This situation can be solved in either way: 
		//   First, change the packet format (is it okay?)
		//	 Second, define concrete derived class, where the size can be fixed. 
		memcpy(_buf + 7 + packetLength, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
		memcpy(_buf + packetLength + 9, "\r\n", 2);

		WriteComm(_buf, MAX_PACKET_BYTES + NUM_BYTES_DATA_PACKET_FILLIN, INFINITE);

		return 0;
	}	

	inline void _initBuf()
	{
		memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_GRAPHICS_DATA;			//1 unsigned char, Packet type
		
		//memcpy(_buf + 3, &numObject, sizeof(int));			//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)
		// FIXME @ 20130902 Is the following correct?
		// data is already prescribed in init/updateGObjects()
		//memcpy(buf + PACKET_DATA_OFFSET, data, packetLength);				//nLen bytes, data

// 		memcpy(_buf + 7 + packetLength, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
// 		memcpy(_buf + packetLength + 9, "\r\n", 2);
	}	

	virtual int _initPacketData(unsigned char * const data)
	{
		return 0;
	}

	virtual int _updatePacketData(unsigned char * const data)
	{
		//_numUpdatedGObjects = 0;
		return 0;
	}

public:
	enum 
	{
		PACKET_DATA_OFFSET = 3,	
		NUM_BYTES_DATA_PACKET_FILLIN = PACKET_DATA_OFFSET + sizeof(unsigned int) + 4,
	};

protected:
	SubsysType & _sys;

	int _numGObjects;
	int _numUpdatedGObjects;

	//unsigned char _buf[MAX_PACKET_BYTES + 100];
	unsigned char _buf[MAX_PACKET_BYTES + NUM_BYTES_DATA_PACKET_FILLIN];

	// FIXED by THACHDO 20150717
	volatile bool _hasConnection; // moved from NRMKSocketBase
	char _requestKey;
	
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).
};

} // namespace NRMKHelper