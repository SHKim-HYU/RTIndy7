/*
 * IndyDCP.h
 *
 *  Created on: 2019. 1. 10.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SERVICE_INDYDCP_H_
#define NRMKINDY_SERVICE_INDYDCP_H_
#pragma once

#if defined (__CYGWIN__)
#ifndef WINDOWS_CYGWIN
#define WINDOWS_CYGWIN
#endif
#elif defined (_WIN32) || defined (_WIN32) || defined (WIN64) || defined (_WIN64)
#ifndef WINDOWS
#define WINDOWS
#endif
#elif defined (__linux) || defined (__linux__) || defined (linux)
#ifndef LINUX
#define LINUX
#endif
#elif defined (__unix) || defined (__unix__)
#ifndef UNIX
#define UNIX
#endif
#else
#error "Unsupported OS! If you want to use, please refine this codes."
#endif //defined _WIN32 ...
#if defined (__posix) || defined (_POSIX_VERSION)
#ifndef POSIX
#define POSIX
#endif
#endif	//__posix

#include <stdint.h>

namespace NRMKIndy
{
namespace Service
{
namespace DCP
{

extern const char * ROBOT_INDYRP;
extern const char * ROBOT_INDYRP2;
extern const char * ROBOT_INDY3;
extern const char * ROBOT_INDY5;
extern const char * ROBOT_INDY10;
extern const char * ROBOT_INDY7;
extern const char * ROBOT_INDY15;
extern const char * ROBOT_OPTI5;
extern const char * ROBOT_OPTI10;

enum
{
	SIZE_DCP_HEADER = 52,
	SIZE_DCP_COMMAND = 4,
	SIZE_DCP_HEADER_COMMAND = 56,
	SIZE_DCP_DATA_TCP_MAX = 200,
	SIZE_DCP_DATA_ASCII_MAX = 32,
	SIZE_DCP_PACKET = 256
};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
struct HeaderCommandStruct	// 56byte
{
	char robotName[20];
	char robotVersion[12];
	unsigned char stepInfo;
	unsigned char sof;		//source of Frame
	int invokeId;
	int dataSize;
//	char reserved[10];	//10byte
	uint32_t status;	//4byte
	char reserved[6];	//6byte
	int cmdId;
};
#pragma pack(pop)   /* restore original alignment from stack */

union HeaderCommand			// 56byte
{
	unsigned char byte[SIZE_DCP_DATA_TCP_MAX];
	HeaderCommandStruct val;
};

union Data					// 200byte
{
	unsigned char byte[SIZE_DCP_DATA_TCP_MAX];
	char asciiStr[SIZE_DCP_DATA_ASCII_MAX+1];
	char str[200];
	char charVal;
	bool boolVal;
	short shortVal;
	int intVal;
	float floatVal;
	double doubleVal;
	uint8_t byteVal;
	int16_t wordVal;
	uint16_t uwordVal;
	int32_t dwordVal;
	int64_t lwordVal;
	char bool6dArr[6];
	char bool7dArr[7];
	bool boolArr[200];
	char char2dArr[2];
	char char3dArr[3];
	char char6dArr[6];
	char char7dArr[7];
	char charArr[200];
	int int2dArr[2];
	int int3dArr[3];
	int int6dArr[6];
	int int7dArr[7];
	int intArr[50];
	float float3dArr[3];
	float float6dArr[6];
	float float7dArr[7];
	float floatArr[50];
	double double3dArr[3];
	double double6dArr[6];
	double double7dArr[7];
	double doubleArr[25];
	uint8_t byteArr[200];
	int16_t wordArr[100];
	uint16_t uwordArr[100];
	int32_t dwordArr[50];
	int64_t lwordArr[25];
};

struct Packet				// 256byte
{
	HeaderCommand header;
	Data data;
};

enum Command : int
{
	CMD_CHECK		 			= 0,
	CMD_EMERGENCY_STOP 			= 1,
	CMD_RESET_ROBOT 			= 2,
	CMD_SET_SERVO 				= 3,
	CMD_SET_BRAKE 				= 4,
	CMD_STOP 					= 5,
	CMD_MOVE					= 6,
	CMD_MOVE_HOME				= 7,
	CMD_MOVE_ZERO				= 8,
	CMD_JOINT_MOVE_TO			= 9,
	CMD_JOINT_MOVE_BY			= 10,
	CMD_TASK_MOVE_TO			= 11,
	CMD_TASK_MOVE_BY			= 12,

	CMD_START_CURRENT_PROGRAM	= 14,
	CMD_PAUSE_CURRENT_PROGRAM	= 15,
	CMD_RESUME_CURRENT_PROGRAM	= 16,
	CMD_STOP_CURRENT_PROGRAM	= 17,
	CMD_START_DEFAULT_PROGRAM	= 18,
	CMD_REGISTER_DEFAULT_PROGRAM_IDX		= 19,
	CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX	= 20,

	CMD_IS_ROBOT_RUNNING		= 30,	//refined ...
	CMD_IS_READY				= 31,
	CMD_IS_EMG					= 32,
	CMD_IS_COLLIDED				= 33,
	CMD_IS_ERR					= 34,
	CMD_IS_BUSY					= 35,
	CMD_IS_MOVE_FINISEHD		= 36,
	CMD_IS_HOME					= 37,
	CMD_IS_ZERO					= 38,
	CMD_IS_IN_RESETTING			= 39,
	CMD_IS_DIRECT_TECAHING		= 60,
	CMD_IS_TEACHING				= 61,
	CMD_IS_PROGRAM_RUNNING		= 62,
	CMD_IS_PROGRAM_PAUSED		= 63,
	CMD_IS_CONTY_CONNECTED		= 64,

	CMD_CHANGE_DIRECT_TEACHING	= 80,
	CMD_FINISH_DIRECT_TEACHING	= 81,	//... refined

	CMD_JOINT_PUSH_BACK_WAYPOINT_SET	= 90,
	CMD_JOINT_POP_BACK_WAYPOINT_SET		= 91,
	CMD_JOINT_CLEAR_WAYPOINT_SET		= 92,
	CMD_JOINT_EXECUTE_WAYPOINT_SET		= 94,
	CMD_TASK_PUSH_BACK_WAYPOINT_SET		= 95,
	CMD_TASK_POP_BACK_WAYPOINT_SET		= 96,
	CMD_TASK_CLEAR_WAYPOINT_SET			= 97,
	CMD_TASK_EXECUTE_WAYPOINT_SET		= 99,

	CMD_SET_DEFAULT_TCP			= 100,
	CMD_RESET_DEFAULT_TCP		= 101,
	CMD_SET_COMP_TCP			= 102,
	CMD_RESET_COMP_TCP			= 103,
	CMD_SET_REFFRAME			= 104,
	CMD_RESET_REFFRAME			= 105,
	CMD_SET_COLLISION_LEVEL		= 106,
	CMD_SET_JOINT_BOUNDARY		= 107,
	CMD_SET_TASK_BOUNDARY		= 108,
	CMD_SET_JOINT_WTIME			= 111,
	CMD_SET_TASK_WTIME			= 112,
	CMD_SET_TASK_CMODE			= 113,
	CMD_SET_JOINT_BLEND_RADIUS	= 116,
	CMD_SET_TASK_BLEND_RADIUS	= 117,

	CMD_GET_DEFAULT_TCP			= 200,
	CMD_GET_COMP_TCP			= 201,
	CMD_GET_REFFRAME			= 202,
	CMD_GET_COLLISION_LEVEL		= 203,
	CMD_GET_JOINT_BOUNDARY		= 204,
	CMD_GET_TASK_BOUNDARY		= 205,
	CMD_GET_JOINT_WTIME			= 208,
	CMD_GET_TASK_WTIME			= 209,
	CMD_GET_TASK_CMODE			= 210,
	CMD_GET_JOINT_BLEND_RADIUS	= 213,
	CMD_GET_TASK_BLEND_RADIUS	= 214,

	CMD_GET_RUNNING_TIME		= 300,
	CMD_GET_CMODE				= 301,
	CMD_GET_JOINT_STATE			= 302,
	CMD_GET_JOINT_POSITION		= 320,
	CMD_GET_JOINT_VELOCITY		= 321,
	CMD_GET_TASK_POSITION		= 322,
	CMD_GET_TASK_VELOCITY		= 323,
	CMD_GET_TORQUE				= 324,

	CMD_GET_LAST_EMG_INFO		= 380,	//new

	CMD_GET_SMART_DI			= 400,
	CMD_GET_SMART_DIS			= 401,
	CMD_SET_SMART_DO			= 402,
	CMD_SET_SMART_DOS			= 403,
	CMD_GET_SMART_AI			= 404,
	CMD_SET_SMART_AO			= 405,
	CMD_GET_SMART_DO			= 406,
	CMD_GET_SMART_DOS			= 407,
	CMD_GET_SMART_AO			= 408,

	CMD_GET_EXTIO_FTCAN_ROBOT_RAW	= 420,
	CMD_GET_EXTIO_FTCAN_ROBOT_TRANS	= 421,
	CMD_GET_EXTIO_FTCAN_CB_RAW		= 422,
	CMD_GET_EXTIO_FTCAN_CB_TRANS	= 423,

	CMD_READ_DIRECT_VARIABLE		= 460,
	CMD_READ_DIRECT_VARIABLES		= 461,
	CMD_WRITE_DIRECT_VARIABLE		= 462,
	CMD_WRITE_DIRECT_VARIABLES		= 463,

	CMD_FOR_EXTENDED				= 800,
	CMD_FOR_STREAMING				= 801,

	CMD_SEND_KEYCOMMAND			= 9996,
	CMD_READ_MEMORY				= 9997,
	CMD_WRITE_MEMORY			= 9998,
	CMD_ERROR					= 9999
};

enum ExtCommand : int
{
	EXT_CMD_MOVE_TRAJ_BY_DATA		= 1,
	EXT_CMD_MOVE_TRAJ_BY_TXT_DATA	= 2,
	EXT_CMD_MOVE_TRAJ_BY_FILE		= 3,
	EXT_CMD_MOVE_TRAJ_BY_TXT_FILE	= 4,

	EXT_CMD_JMOVE_ABS_WAYPOINT_SET		= 11,
	EXT_CMD_TMOVE_ABS_WAYPOINT_SET		= 12,
};

enum ErrorCode : int
{
	ERR_NONE = 0,
	ERR_NO_MATCHED_ROBOT = 1,
	ERR_NO_MATCHED_STEP = 2,
	ERR_HEADER_FORMAT = 4,
	ERR_OVER_DATA_SIZE = 5,
	ERR_NOT_SUPPORT_COMMAND = 6,
	ERR_UNKNOWN_COMMAND = 7,
	ERR_UNKNOWN_DATA = 8,
	ERR_PROCESS_FAILED = 9,
	ERR_PARSE_FAILED = 10,
	ERR_NO_MATCHED_PARAMETER = 11,
	ERR_NO_MATCHED_DATA_SIZE = 12,
	ERR_WRONG_ASCII_FORMAT = 13,
	ERR_ROBOT_MOVING_STATE = 14,
	ERR_ROBOT_PROGRAM_RUNNING = 15,
	ERR_ROBOT_MOVE_FAILED = 16,
	ERR_NO_DEFAULT_PROGRAM = 17,
	ERR_NO_CURRENT_PROGRAM = 18,
	ERR_CURRENT_PROGRAM_STATE = 19,
	ERR_EMG_STATE = 20,
	ERR_ROBOT_STATE = 21,
	ERR_ROBOT_PROGRAM_LOAD_FAILED = 22,
	ERR_DIRECT_VARIABLE_INVALID_ADDRESS = 23,
	ERR_DIRECT_VARIABLE_INVALID_FORMAT = 24,
	ERR_DIRECT_VARIABLE_REFNUM_LIMIT = 25,
	ERR_CONNECTION_EXCEPTION = 600,
	ERR_CONNECTION_TIMEOUT = 601,
};

enum HeaderStatusBit : uint32_t
{
	HEADER_STATUS_BIT_TASK_RUNNING		= 0x80000000,	// 0b 1000 0000 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_ROBOT_READY		= 0x40000000,	// 0b 0100 0000 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_EMG_STOPPED		= 0x20000000,	// 0b 0010 0000 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_COLLIDED			= 0x10000000,	// 0b 0001 0000 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_ERR_STATE			= 0x08000000,	// 0b 0000 1000 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_BUSY				= 0x04000000,	// 0b 0000 0100 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_MOVE_FINISHED		= 0x02000000,	// 0b 0000 0010 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_HOME				= 0x01000000,	// 0b 0000 0001 0000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_ZERO				= 0x00800000,	// 0b 0000 0000 1000 0000 0000 0000 0000 0000
	HEADER_STATUS_BIT_IN_RESETTING		= 0x00400000,	// 0b 0000 0000 0100 0000 0000 0000 0000 0000

	HEADER_STATUS_BIT_DIRECT_TEACHING	= 0x00000080,	// 0b 0000 0000 0000 0000 0000 0000 1000 0000
	HEADER_STATUS_BIT_TEACHING			= 0x00000040,	// 0b 0000 0000 0000 0000 0000 0000 0100 0000
	HEADER_STATUS_BIT_PROGRAM_RUNNING	= 0x00000020,	// 0b 0000 0000 0000 0000 0000 0000 0010 0000
	HEADER_STATUS_BIT_PROGRAM_PAUSED	= 0x00000010,	// 0b 0000 0000 0000 0000 0000 0000 0001 0000
	HEADER_STATUS_BIT_CONTY_CONNECTED	= 0x00000008,	// 0b 0000 0000 0000 0000 0000 0000 0000 1000
};

enum DirectVariableType : int
{
	DIRECT_VAR_TYPE_ERROR=-1,
	DIRECT_VAR_TYPE_BYTE=0,
	DIRECT_VAR_TYPE_WORD=1,
	DIRECT_VAR_TYPE_DWORD=2,
	DIRECT_VAR_TYPE_LWORD=3,
	DIRECT_VAR_TYPE_FLOAT=4,
	DIRECT_VAR_TYPE_DFLOAT=5,
	DIRECT_VAR_TYPE_MODBUS_REG=10,
};

} /* namespace DCP */
} /* namespace Service */
} /* namespace NRMKIndy */


#endif /* INDYDCP_H_ */
