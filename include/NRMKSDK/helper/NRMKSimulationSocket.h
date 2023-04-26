/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "LieGroup/LieGroup.h"

#include "NRMKSocketBase.h"
#include "vkcode.h"

namespace NRMKHelper
{

class PhysicalInteractivity
{
public:
	enum INTERACTIVITY_MODE
	{
		NONE = 0,
		APPLY_FORCE,
		APPLY_MOMENT,
	};

public:
	inline PhysicalInteractivity()	: _body(-1), _mode(NONE)  { }

	inline unsigned int body() const { return _body; }
	inline bool hasInteractivity() const { return _mode != NONE; }

	inline LieGroup::Displacement const &  at() const { return _at; }
	inline LieGroup::Displacement &  at()  { return _at; }
	
	inline LieGroup::Wrench const &  F() const { return _F; }
	inline void setForce(LieGroup::Vector3D & f, INTERACTIVITY_MODE mode = APPLY_FORCE)
	{
		_mode = mode;
		if (mode == APPLY_FORCE)
		{
			_F.f() = f;
			_F.n().setZero();
		}
		else if (mode == APPLY_MOMENT)
		{
			_F.f().setZero();
			_F.n() = f;
		}
	}

	inline void init(int body = -1)
	{
		if (body != -1)
			_body = body;

		_mode = NONE;

		_at.setZero();
		_F.setZero();
	}

private:
	int _body;
	INTERACTIVITY_MODE _mode;

	LieGroup::Displacement _at;
	LieGroup::Wrench	_F;
};

template <typename SubsysType>
class NRMKSimulationSocket : public NRMKSocketBase
{
public:
	inline NRMKSimulationSocket(SubsysType & robot, unsigned int const * const bodyIndex = NULL)
		: NRMKSocketBase(), _sys(robot)//, _numBodies(SubsysType::NUM_BODIES)
		, _hasConnection(false), _requestKey(0)
		, _activeBody(-1)
	{
		_setBodyIndex(bodyIndex);
		// FIXED @ 20130821: Prepare zero configuration data when initializing
		makeSocketTransformData(_socket_data_zero_config);

		for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			_interativity[k].init(k);
	}

	//NRMKSimulationSocket(LPCTSTR sIP, LPCTSTR sPort)

	//~NRMKSimulationSocket();
	
	inline virtual bool hasConnection() const { return _hasConnection; }
	
	inline PhysicalInteractivity const & interactivity(int k) const { return _interativity[k]; }

	// this is the main routine when data com, overriding original one
	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{
		_requestKey = lpBuffer[0];
		//printf("RequestKey: %i\n", RequestKey);
		switch (_requestKey)
		{
		case 'O':
			if (!_hasConnection)
			{
				init();
			}

			break;

		case 'R': // for acknowledgment
			if (!_hasConnection)
			{
				_hasConnection = true;
				_requestKey = 0;
				// now ready to run...
			}

			break;

		default:
			if (dwCount >= 17)	// 17 accounting for 13 for header and 4 for the tail.
			{
				/*
				int ti, num_floats;
				float tf;
				BYTE tb;
				printf("0x%x ", lpBuffer[3]);//command
				printf("0x%x ", lpBuffer[4]);//virtual key
				memcpy(&ti, &lpBuffer[5], sizeof(int));//body index
				printf("%i ", ti);
				memcpy(&ti, &lpBuffer[9], sizeof(int));//model index
				printf("%i ", ti);
				num_floats= (dwCount - 17)/sizeof(float);	
				for (int i = 0; i < num_floats; ++i)
				{
					memcpy(&tf, &lpBuffer[13+4*i], 4);//x, y, or z
					printf("%f, ", tf);
				}
				printf("\n");
				*/

				unsigned char cmd;
				memcpy(&cmd, &lpBuffer[3], 1);	//command

				int body;
				memcpy(&body, &lpBuffer[5], sizeof(int));	//body index

				if (cmd == 0)
				{
					_interativity[body].init();
					// This is only temporary. Currently one can apply only one force/moment.
					if (_activeBody != -1)
						_interativity[_activeBody].init();

					_activeBody = body;
				}
				else
				{
					unsigned char key;
					memcpy(&key, &lpBuffer[4], 1);		//virtual key

					// set begin position
					if (cmd == 3 || cmd == 1)
					{
						float begin[3];

						memcpy(begin, &lpBuffer[13], 3*sizeof(float));
						_interativity[body].at().set((double) begin[0], (double) begin[1], (double) begin[2]);
					}

					// set force/moment
					if (cmd != 1)
					{
						float end[3] = {0, 0, 0};
						
						if (cmd == 3)
							memcpy(end, &lpBuffer[25], 3*sizeof(float));
						else if (cmd == 2)
							memcpy(end, &lpBuffer[13], 3*sizeof(float));

						LieGroup::Vector3D f((double) end[0], (double) end[1], (double) end[2]);
						f -= _interativity[body].at();
					
						switch (key)
						{
						case VK_SHIFT:
							_interativity[body].setForce(f, PhysicalInteractivity::APPLY_FORCE);
							break;

						case VK_CONTROL:
							_interativity[body].setForce(f, PhysicalInteractivity::APPLY_MOMENT);
							break;
						}
					}
				}

			}
		}
	}

	inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
	{
		switch( uEvent )
		{
		case EVT_CONSUCCESS:
			std::cout << "One client has connected to the Simulation Server..." << std::endl;
			break;

		case EVT_CONDROP:
			std::cout << "Simulation Server Connection abandoned " << std::endl;

			// Added@20140822
			_hasConnection = false;
			_requestKey = 0;
			_activeBody = -1;

			StopComm();

			if (IsServer())
				waitForConnection(0);

			break;

		default:
			NRMKSocketBase::OnEvent(uEvent, lpvData);
			break;
		}
	}

	inline void getKey(unsigned char & key) 
	{
		key = (_hasConnection) ? _requestKey : 0;
		_requestKey = 0;
	}

	inline void init()
	{
		sendBodyIndexArray(); //send index array
		Sleep(100); //should wait for finishing sendBodyIndexArray

		// FIXED @ 20130821: Send the prepared zero configuration data
		//			Now CADKitViewer can connect during simulation safely.
		sendZeroConfig(); //send initial configuration

		// FIXME @20131015
		_initBuf();
	}

	inline void makeSocketTransformData(float (& socket_data)[12*SubsysType::NUM_BODIES]) 
	{
		for (int k = 0, cur = 0; k < SubsysType::NUM_BODIES; k++)
		{
			double * data = _sys.body(k).T().data();
			for (int i = 0; i < 4; i++)
			{
				int dest = cur + 3*i;
				int src = 4*i;

				socket_data[dest] = (float) data[src];
				socket_data[dest + 1] = (float) data[src + 1];
				socket_data[dest + 2] = (float) data[src + 2];
			}	

			cur += 12;
		}
	}

	inline void updateTransform(float t)
	{
		memcpy(_buf + PACKET_DATA_OFFSET, &t, sizeof(float));
		_makeSocketTransformData();

		_sendTransform();
	}

	//-----Simulation data communication-----------------
	inline int sendBodyIndexArray()
	{
		if (!IsOpen())
			return -1;

		unsigned int num_bodies = SubsysType::NUM_BODIES;

		//unsigned char buf[BODYINDEX_PACKET_DATA_LENGTH + 100];
		unsigned char buf[BODYINDEX_PACKET_DATA_LENGTH + 11]; // 11 for PACKET_DATA_OFFSET + sizeof(unsigned int) + 4

		memcpy(buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	// 2 bytes, NRMK_SOCKET_START_TOKEN
		buf[2] = (unsigned char) NRMK_SOCKET_PACKET_INDEX;						// 1 unsigned char, Packet type
		memcpy(buf + 3, &num_bodies, sizeof(unsigned int));			// 4 bytes, Number of integer data (bytes = NumOfData*4)
		memcpy(buf + 7, _bodyIndex, BODYINDEX_PACKET_DATA_LENGTH);		// nLen bytes, data
		memcpy(buf + 7 + BODYINDEX_PACKET_DATA_LENGTH, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE);		// 2 bytes, NRMK_SOCKET_END_TOKEN
		
		memcpy(buf + BODYINDEX_PACKET_DATA_LENGTH + 9, "\r\n", 2);
		WriteComm(buf, BODYINDEX_PACKET_DATA_LENGTH + 11, INFINITE);

		return BODYINDEX_PACKET_DATA_LENGTH + 11;
	}

	inline int sendZeroConfig()
	{
		if (!IsOpen())
			return -1;

		//stMessageProxy msgProxy;

		memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_ZERO_CONFIG;				//1 unsigned char, Packet type
		//memcpy(_buf + 3, &_numBodies, sizeof(int));			//4 bytes, Number of integer data (bytes = NumOfData*4)
		memcpy(_buf + 3, _socket_data_zero_config, TRANSFORM_PACKET_DATA_LENGTH);		//nLen bytes, data
		memcpy(_buf + 3 + TRANSFORM_PACKET_DATA_LENGTH, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);		//2 bytes, NRMK_SOCKET_TOKEN_UPDATE

		memcpy(_buf + TRANSFORM_PACKET_DATA_LENGTH + 5, "\r\n", 2);
		WriteComm(_buf, TRANSFORM_PACKET_DATA_LENGTH + 7, INFINITE);

		return TRANSFORM_PACKET_DATA_LENGTH + 7;
	}

private:
	inline void _makeSocketTransformData()
	{
		// FIXME @20131015: sizeof(unsigned int) should be deleted
		float * const socket_data = (float * const)(_buf + PACKET_DATA_OFFSET + sizeof(float)); // sizeof(float) for time

		for (int k = 0, cur = 0; k < SubsysType::NUM_BODIES; k++)
		{
			double * data = _sys.body(k).T().data();
			for (int i = 0; i < 4; i++)
			{
				int dest = cur + 3*i;
				int src = 4*i;

				socket_data[dest] = (float) data[src];
				socket_data[dest + 1] = (float) data[src + 1];
				socket_data[dest + 2] = (float) data[src + 2];
			}	

			cur += 12;
		}
	}


	// transform data should have been prepared in _buf...
	inline int _sendTransform()
	{
		if (!IsOpen())
			return -1;

		//memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		//_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_TRANSFORM;			//1 unsigned char, Packet type
		//memcpy(_buf + 3, &curTime, sizeof(float));				//8 bytes (double), current time (absolute)
		// FIXME @20131015 _numBodies should be deleted
		//memcpy(_buf + 7, &_numBodies, sizeof(int));			//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)
		//memcpy(_buf + 7, f_Transform, TRANSFORM_PACKET_DATA_LENGTH);		//nLen bytes, data
		//memcpy(_buf + 7 + TRANSFORM_PACKET_DATA_LENGTH, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token

		//memcpy(_buf + TRANSFORM_PACKET_DATA_LENGTH + 9, "\r\n", 2);

		WriteComm(_buf, TRANSFORM_PACKET_DATA_LENGTH + NUM_BYTES_DATA_PACKET_FILLIN, INFINITE);

		return TRANSFORM_PACKET_DATA_LENGTH + NUM_BYTES_DATA_PACKET_FILLIN;
	}

	// this is used to fill in the packet header and tails for routine data transmission
	inline void _initBuf() 
	{
		memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_TRANSFORM;			//1 unsigned char, Packet type

		// The next bytes of size TRANSFORM_PACKET_DATA_LENGTH are for transformation data

		//memcpy(_buf + 3, &curTime, sizeof(float));				//8 bytes (double), current time (absolute)
		// FIXME @20131015 _numBodies should be deleted
		//memcpy(_buf + 7, &_numBodies, sizeof(unsigned int));			//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)
		//memcpy(_buf + 7, f_Transform, TRANSFORM_PACKET_DATA_LENGTH);		//nLen bytes, data
		
		memcpy(_buf + 7 + TRANSFORM_PACKET_DATA_LENGTH, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
		memcpy(_buf + TRANSFORM_PACKET_DATA_LENGTH + 9, "\r\n", 2);

		//WriteComm(_buf, TRANSFORM_PACKET_DATA_LENGTH + 11, INFINITE);
	}

	inline void _setBodyIndex(unsigned int const * const bodyIndex)
	{
		if (bodyIndex)
		{
			for (int i = 0; i < SubsysType::NUM_BODIES; i++)
				_bodyIndex[i] = bodyIndex[i];
		}
		else
		{
			for (int i = 0; i < SubsysType::NUM_BODIES; i++)
				_bodyIndex[i] = i;
		}
	}

private:
	enum
	{
		PACKET_DATA_OFFSET = 3,
		TRANSFORM_PACKET_DATA_LENGTH = sizeof(float) * 12 * SubsysType::NUM_BODIES, // 12 for HTransforma data
		BODYINDEX_PACKET_DATA_LENGTH = sizeof(int) * SubsysType::NUM_BODIES,
		NUM_BYTES_DATA_PACKET_FILLIN = PACKET_DATA_OFFSET + sizeof(float) + 4,
	};

private:
	SubsysType & _sys;
	//int _numBodies;

	//unsigned char _buf[TRANSFORM_PACKET_DATA_LENGTH + 100];
	unsigned char _buf[TRANSFORM_PACKET_DATA_LENGTH + NUM_BYTES_DATA_PACKET_FILLIN];

	unsigned int _bodyIndex[SubsysType::NUM_BODIES];
	//float _socket_data[12*SubsysType::NUM_BODIES];
	float _socket_data_zero_config[12*SubsysType::NUM_BODIES];

	// FIXED by THACHDO 20150717
	volatile bool _hasConnection; // moved from NRMKSocketBase
	unsigned char _requestKey;
	
	// Added for v1.6
	PhysicalInteractivity _interativity[SubsysType::NUM_BODIES];
	int _activeBody;
	
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).	
};

} // namespace NRMKHelper
