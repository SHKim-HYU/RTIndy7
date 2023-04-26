/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "Poco/Event.h"
#include "Poco/Mutex.h"

namespace NRMKHelper
{

// Added Event object
// FIXME @20131016 Make this linux-compatible
class NRMKEvent
{
public:
// 	NRMKEvent(char* lpName)
// 	{
// 	}
// 
// 	~NRMKEvent()
// 	{
// 	}

	void set() 
	{
		try {
			_event.set();
		}
		catch (Poco::Exception ex) {
			//if (!SetEvent(_event))
			//printf("SetEvent failed (%d)\n", ex.displayText());
			printf("SetEvent failed\n");
		}
	}

	void reset()
	{
		try {
			_event.reset();
		}
		catch (Poco::Exception ex) {
			//if (!SetEvent(_event))
			//printf("ResetEvent failed (%d)\n", ex.displayText());
			printf("ResetEvent failed \n");
		}
	}

	int wait()
	{
		try {
			_event.wait(1000);
			return 1;
		}
		catch (Poco::Exception ex) {
			//if (!SetEvent(_event))
			//printf("Wait error (%d)\n", ex.displayText());
			printf("Wait error \n");
			return 0;
		}
	}

private:
	//HANDLE		_event;
	Poco::Event _event;
};


// Added Event object
// FIXME @20131016 Make this linux-compatible
class NRMKMutex
{
public:
// 	NRMKMutex(char* lpName)
// 	{
// 	}
// 
// 	~NRMKMutex()
// 	{
// 	}

	void lock() 
	{
		try {
			_mutex.lock();
		}
		catch (Poco::Exception ex) {
			printf("LockMutex failed\n");
		}
	}

	void unlock()
	{
		try {
			_mutex.unlock();
		}
		catch (Poco::Exception ex) {
			printf("UnlockMutex failed \n");
		}
	}

private:
	Poco::Mutex _mutex;
};

} // namespace NRMKHelper
