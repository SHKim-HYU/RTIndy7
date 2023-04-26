/*
 * ShmemManager.hpp
 *
 *  Created on: 2018. 1. 11.
 *      Author: Hanter Jung
 */

#ifndef NRMKFRAMEWORK_SHMEMMANAGER_HPP_
#define NRMKFRAMEWORK_SHMEMMANAGER_HPP_

#include <stdio.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include <stdexcept>

#include <sys/stat.h>
#include <semaphore.h> //POSIX semaphore

#include "shmem.h"

namespace NRMKFramework
{

class ShmemManager
{
public:
	/*ShmemManager(void * shmemPtr, uint32_t size)
	: _shmemPtr((unsigned char *)shmemPtr)
	, _size(size)
	{
		if (shmemPtr == NULL) throw std::runtime_error("Invalid Address");
	}*/

	ShmemManager(shmem_t * shm)
	: _shm(shm)
	, _shmemPtr((unsigned char *) shmem_ptr(shm))
	, _name(shm->name)
	, _size((uint32_t)shm->size)
	, _managedByOutside(true)
	{
		if (_shmemPtr == NULL)
		{
			throw std::runtime_error("Invalid Address");
			return;
		}

		initSemaphore();
	}

	ShmemManager(std::string shmName, unsigned int size)
	: _name(shmName)
	, _size((uint32_t)size)
	, _managedByOutside(false)
	{
		_shm = shmem_create_or_open(shmName.c_str(), size);
		if (!_shm)
		{
			printf("ShmemManager: Binding \"%s\" failed!\n", shmName.c_str());
			throw std::runtime_error("Binding shared memory failed");
			return;
		}
		else
			printf("ShmemManager: Binding \"%s\" success!\n", shmName.c_str());

		_shmemPtr = (unsigned char *)shmem_ptr(_shm);
		if (_shmemPtr == NULL)
		{
			printf("ShmemManager: Invalid Address for \"%s\"!\n", shmName.c_str());
			throw std::runtime_error("Invalid Address");
			return;
		}

		initSemaphore();
	}

	~ShmemManager()
	{
		sem_close(_sem);

		if (!_managedByOutside)
		{
			// FIXME why don't use close?
			// shmem_close(_shm);
		}
	}

	inline bool readMemory(uint32_t address, uint32_t size, void * ptrData)
	{
		if (address + size > _size) return false;
		memcpy(ptrData, _shmemPtr+address, size);
		return true;
	}

	inline bool writeMemory(uint32_t address, uint32_t size, void const * ptrData)
	{
		if (address + size > _size) return false;
		memcpy(_shmemPtr+address, ptrData, size);
		return true;
	}

	inline bool readMemoryWait(uint32_t address, uint32_t size, void * ptrData, bool tryWait=false)
	{
		if (address + size > _size) return false;
		//printf("ShmemManager : Try Lock Semaphore.\n");
		if (tryWait)
		{
			int ret = sem_trywait(_sem);
			if (ret != 0) return false;
		} else sem_wait(_sem);
		//printf("ShmemManager : Semaphore Locked.\n");
		memcpy(ptrData, _shmemPtr+address, size);
		//printf("ShmemManager : Unlock Semaphore.\n");
		sem_post(_sem);
		//printf("ShmemManager : Semaphore Unocked.\n");
		return true;
	}

	inline bool writeMemoryWait(uint32_t address, uint32_t size, void * ptrData, bool tryWait=false)
	{
		if (address + size > _size) return false;
		if (tryWait)
		{
			int ret = sem_trywait(_sem);
			if (ret != 0) return false;
		} else sem_wait(_sem);
		memcpy(_shmemPtr+address, ptrData, size);
		sem_post(_sem);
		return true;
	}

	inline void wait()
	{
		// printf("ShmManager: try Wait... [%d]\n", getSemVal());
		sem_wait(_sem);
		// printf("ShmManager: Wait! [%d]\n", getSemVal());
	}

	inline bool tryWait()
	{
		int ret = sem_trywait(_sem);
		if (ret == 0) return true;
		else return false;
	}

	inline void post()
	{
		// printf("ShmManager: try Post... [%d]\n", getSemVal());
		sem_post(_sem);
		// printf("ShmManager: Post! [%d]\n", getSemVal());
	}

	inline int getSemVal()
	{
		int sval;
		if (sem_getvalue(_sem, &sval) == 0)
			return sval;
		else
			return -1;
	}

	inline void * getPtr(uint32_t address)
	{
		if (address >= _size) return NULL;
		return (void*)(_shmemPtr+address);
	}

	template <typename T>
	inline T * getPtrObjByAddr(uint32_t address)
	{
		if (address + sizeof(T) > _size) return NULL;
		T * ptrObj = (T *)(_shmemPtr+address);
		return ptrObj;
	}

	template <typename T>
	inline T const * getPtrObjByAddr (uint32_t address) const
	{
		if (address + sizeof(T) > _size) return NULL;
		T * ptrObj = (T *)(_shmemPtr+address);
		return ptrObj;
	}

	template <typename T>
	inline T & getRefObjByAddr(uint32_t address)
	{
		if (address + sizeof(T) > _size) throw std::runtime_error("Invalid Address");
		T * ptrObj = (T *)(_shmemPtr+address);
		return *ptrObj;
	}

	template <typename T>
	inline T const & getRefObjByAddr (uint32_t address) const
	{
		if (address + sizeof(T) > _size) throw std::runtime_error("Invalid Address");
		const T * ptrObj = (const T *)(_shmemPtr+address);
		return *ptrObj;
	}

	inline void resetMemory()
	{
		memset(_shmemPtr, 0, _size);
	}

	std::string getName() { return _name; }
	uint32_t getSize() { return _size; }

	shmem_t * getMemoryInfo() { return _shm; }
	unsigned char * getMemoryPtr() { return _shmemPtr; }

private:
	inline void initSemaphore()
	{
		std::string semName;
		semName.append("/").append(_name).append("_sem");
//		_sem = sem_open(semName.c_str(), O_CREAT, 0666/*0666*/, 0/*1*/); // initialize semaphores for shared processes
		_sem = sem_open(semName.c_str(), O_RDWR|O_CREAT, 0666 , 1/*0*/); // initialize semaphores for shared processes

		if (_sem == SEM_FAILED)
		{
			printf("ShmemManager : Semaphore creation is failed. errno=%d\n", errno);
			throw std::runtime_error("Semaphore cannot be created");
			return;
		}

		sem_unlink(semName.c_str());	// unlink prevents the semaphore existing forever
//		sem_post(_sem);

		/*int sval;
		if (sem_getvalue(_sem, &sval) == 0)
		{
			printf("ShmemManager : Created Semaphore(\%s) with val=%d\n", semName.c_str(), sval);
		}
		else
		{
			printf("ShmemManager : Created Failed. errno=%d\n", errno);
		}*/
	}

private:
	shmem_t * _shm;
	unsigned char * _shmemPtr;
	const std::string _name;
	const uint32_t _size;
	const bool _managedByOutside;
	sem_t * _sem;
};

} /* NRMKFramework */

#endif /* NRMKFRAMEWORK_SHMEMMANAGER_HPP_ */
