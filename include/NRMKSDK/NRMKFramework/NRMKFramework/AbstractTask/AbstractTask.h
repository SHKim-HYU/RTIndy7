/*
 * AbstractTask.h
 *
 *  Created on: Jun 04, 2019
 *      Author: ThachDo
 */

#ifndef NRMKFRAMEWORK_ABSTRACTTASK_H_
#define NRMKFRAMEWORK_ABSTRACTTASK_H_

#include <Poco/ClassLibrary.h>
#include <json/value.h>
#include <NRMKFramework/FrameworkDefine.h>
#include <NRMKFramework/NRMKHelper/Version.h>
#include <NRMKFramework/shmem/shmem.h>

#include <native/heap.h>
#include <string>
#include <vector>
#include <NRMKFramework/Components/AbstractComponent.h>

namespace NRMKFramework
{

class AbstractTask
{
public:
	enum {
		INTERFACE_VERSION = 2
	};

public:
	AbstractTask(std::string name, const NRMKHelper::Version & version=NRMKHelper::Version(),
			const NRMKHelper::Version & minFrameworkVersion=NRMK_FRAMEWORK_VERSION,
			const std::string & target="");
	virtual ~AbstractTask();

	std::string getName() { return _name; }
	NRMKHelper::Version getVersion() { return _version; }
	NRMKHelper::Version getMinFrameworkVersion() { return _minFrameworkVersion; }
	std::string getTarget() { return _target; }
	int getRTRate();

	bool isWorking() { return _working; }
	bool acquireSharedDataPtr(std::string name, unsigned int size, void** dataptr);
	void * acquireSharedDataPtr(std::string name, unsigned int size);
	shmem_t * acquireSharedData(std::string name, unsigned int size);

	virtual void start();
	virtual void stop();

	virtual void setParams(Json::Value params) {}
	virtual bool configure() = 0;
	virtual void execute() = 0;
	virtual void cleanup() = 0;

public:
	void _setRTRate(int rtRate);

protected:
    bool verifyComponent(NRMKLeanFramework::AbstractComponent const * const component);

private:
    std::string _getMACAddress();

private:
	const std::string _name;
	const NRMKHelper::Version _version;
	const NRMKHelper::Version _minFrameworkVersion;	//the minimum required version of framework (TaskManager)
	const std::string _target;	//optional for informing target clearly

	bool _working;

	std::vector<shmem_t *> _shmemList;
	std::vector<void *> _allocMemList;

	int _rtRate;
};

}

#endif /* NRMKFRAMEWORK_ABSTRACTTASK_H_ */
