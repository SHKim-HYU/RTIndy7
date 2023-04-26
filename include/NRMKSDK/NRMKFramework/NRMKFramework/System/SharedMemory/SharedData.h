/*
 * SharedData.h
 *
 *  Created on: 2018. 01. 15.
 *      Author: Hanter Jung
 */

#ifndef NRMKFRAMEWORK_SYSTEMSHAREDDATA_H_
#define NRMKFRAMEWORK_SYSTEMSHAREDDATA_H_

#include "SharedMemoryAddress.h"
#include "../../shmem/ShmemManager.hpp"

#include "string.h"
#include "inttypes.h"

namespace NRMKFramework
{

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

struct SystemSharedData
{
	SystemSharedData()
	: time(0.0), rtCycleTime(0), rtCycleJitter(0)
	, tasksExecPeriod(0), tasksExecPeriodMax(0), overruns(0)
	{}

	SystemSharedData(
			const double time,
			uint64_t rtCycleTime,
			uint64_t rtCycleJitter,
			uint64_t tasksExecPeriod,
			uint64_t maxTasksExecPeriod,
			uint64_t overruns)
	: time(time), rtCycleTime(rtCycleTime), rtCycleJitter(rtCycleJitter)
	, tasksExecPeriod(tasksExecPeriod), tasksExecPeriodMax(maxTasksExecPeriod), overruns(overruns)
	{
	}

	SystemSharedData(SystemSharedData const & data)
	{
		time				= data.time;
		rtCycleTime			= data.rtCycleTime;
		rtCycleJitter		= data.rtCycleJitter;
		tasksExecPeriod		= data.tasksExecPeriod;
		tasksExecPeriodMax	= data.tasksExecPeriodMax;
		overruns			= data.overruns;
	}

	SystemSharedData & operator=(const SystemSharedData & data)
	{
		time				= data.time;
		rtCycleTime			= data.rtCycleTime;
		rtCycleJitter		= data.rtCycleJitter;
		tasksExecPeriod		= data.tasksExecPeriod;
		tasksExecPeriodMax	= data.tasksExecPeriodMax;
		overruns			= data.overruns;
		return (*this);
	}

	void update(
			const double time,
			uint64_t rtCycleTime,
			uint64_t rtCycleJitter,
			uint64_t taskExecPeriod,
			uint64_t tasksExecPeriodMax,
			uint64_t overruns)
	{
		this->time					= time;
		this->rtCycleTime			= rtCycleTime;
		this->rtCycleJitter			= rtCycleJitter;
		this->tasksExecPeriod		= taskExecPeriod;
		this->tasksExecPeriodMax	= tasksExecPeriodMax;
		this->overruns				= overruns;
	}

	double time;
	uint64_t rtCycleTime;
	uint64_t rtCycleJitter;
	uint64_t tasksExecPeriod;
	uint64_t tasksExecPeriodMax;
	uint64_t overruns;
};


struct SystemInfoSharedData
{
	SystemInfoSharedData()
	: rtTimerRate(0)
	, rtQueuesRate{0, 0, 0, 0, 0, 0, 0, 0}
	, frameworkVersion{'\0'}
	{}

	int rtTimerRate;
	int rtQueuesRate[8];
	char frameworkVersion[32];
};


#pragma pack(pop)   /* restore original alignment from stack */




#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 4 byte boundary */

struct TaskInfoSharedData
{
	enum TaskState {
		TASK_STATE_NONE = 0,
		TASK_STATE_LOADED = 1,
		TASK_STATE_CONFIGURED = 2,
		TASK_STATE_RUNNING = 3,
		TASK_STATE_STOPPED = 4,
		TASK_STATE_DELETING = 5,
	};

	TaskInfoSharedData()
	: taskName{'\0'}, taskVersion{'\0'}, minFrameworkVersion{'\0'}, target{'\0'}
	, idx(-1), assignedQueue(-1), state(TASK_STATE_NONE)
	{
	}

	TaskInfoSharedData(int idx, int assignedQueue, const char * taskName,
			const char * taskVersion, const char * minFrameworkVersion,
			const char * target="\0")
	: idx (idx)
	, assignedQueue (assignedQueue)
	, state(TASK_STATE_NONE)
	{
		strncpy(this->taskName, taskName, 63);
		strncpy(this->taskVersion, taskVersion, 31);
		strncpy(this->minFrameworkVersion, minFrameworkVersion, 31);
		strncpy(this->target, target, 63);
	}

	void set(int idx, int assignedQueue, const char * taskName,
			const char * taskVersion, const char * minFrameworkVersion,
			const char * target="\0")
	{
		this->idx = idx;
		this->assignedQueue = assignedQueue;
		strncpy(this->taskName, taskName, 63);
		strncpy(this->taskVersion, taskVersion, 31);
		strncpy(this->minFrameworkVersion, minFrameworkVersion, 31);
		strncpy(this->target, target, 63);
	}

	void reset()
	{
		idx = -1;
		assignedQueue = -1;
		state = TASK_STATE_NONE;
		taskName[0] = '\0';
		taskVersion[0] = '\0';
		minFrameworkVersion[0] = '\0';
		target[0] = '\0';
	}

	//The size of this struct = 256byte
	char taskName[64];				//64byte
	char taskVersion[32];			//32
	char minFrameworkVersion[32];	//32
	char target[64];				//64
	int idx;						//4byte
	int assignedQueue;				//4
	int state;						//4
	char reserved[52];		//256-64-32-32-64-12=52
};

#pragma pack(pop)   /* restore original alignment from stack */



inline void getSystemData(NRMKFramework::ShmemManager & sysShm, SystemSharedData & sysData)
{
	sysShm.readMemory(NRMK_SHM_SYSTEM_ADDR_STRUCT_DATA, sizeof(SystemSharedData), &sysData);
}

inline SystemSharedData getSystemData(NRMKFramework::ShmemManager & sysShm)
{
	SystemSharedData sysData;
	getSystemData(sysShm, sysData);
	return sysData;
}

inline void getPtrSystemData(NRMKFramework::ShmemManager & sysShm, SystemSharedData ** ptrSysData)
{
	*ptrSysData = sysShm.getPtrObjByAddr<SystemSharedData>(NRMK_SHM_SYSTEM_ADDR_STRUCT_DATA);
}


inline void getSystemInfoData(NRMKFramework::ShmemManager & sysShm, SystemInfoSharedData & sysInfo)
{
	sysShm.readMemory(NRMK_SHM_SYSTEM_INFO_ADDR_STRUCT_DATA, sizeof(SystemInfoSharedData), &sysInfo);
}

inline SystemInfoSharedData getSystemInfoData(NRMKFramework::ShmemManager & sysShm)
{
	SystemInfoSharedData sysData;
	getSystemInfoData(sysShm, sysData);
	return sysData;
}

inline void getPtrSystemInfoData(NRMKFramework::ShmemManager & sysShm, SystemInfoSharedData ** ptrSysInfo)
{
	*ptrSysInfo = sysShm.getPtrObjByAddr<SystemInfoSharedData>(NRMK_SHM_SYSTEM_INFO_ADDR_STRUCT_DATA);
}


inline void getTaskInfoData(NRMKFramework::ShmemManager & sysShm, TaskInfoSharedData & taskInfo, int idx)
{
	if (idx >= 0 && idx < NRMK_SHM_SYSTEM_TASK_INFO_STRUCT_MAX_NUM)
		sysShm.readMemory(NRMK_SHM_SYSTEM_TASK_INFO_STRUCTS_START_ADDR+(idx*NRMK_SHM_SYSTEM_TASK_INFO_STRUCT_SIZE),
				sizeof(TaskInfoSharedData), &taskInfo);
}

inline TaskInfoSharedData getTaskInfoData(NRMKFramework::ShmemManager & sysShm, int idx)
{
	TaskInfoSharedData taskInfo;
	getTaskInfoData(sysShm, taskInfo, idx);
	return taskInfo;
}

inline void getPtrTaskInfoData(NRMKFramework::ShmemManager & sysShm, TaskInfoSharedData ** ptrTaskInfo)
{
	*ptrTaskInfo = sysShm.getPtrObjByAddr<TaskInfoSharedData>(NRMK_SHM_SYSTEM_TASK_INFO_STRUCTS_START_ADDR);
}




} /* namespace NRMKFramework */

#endif /* NRMKFRAMEWORK_SYSTEMSHAREDDATA_H_ */
