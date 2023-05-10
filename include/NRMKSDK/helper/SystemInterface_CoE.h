/*
 * SystemInterface_CoE.h
 *
 *  Created on: Mar 9, 2015
 *      Author: Jaechul Kim
 */

#ifndef SYSTEMINTERFACE_COE_H_
#define SYSTEMINTERFACE_COE_H_

#include "SystemInterfaceBase.h"

namespace NRMKHelper
{
	typedef uint64_t 		UINT64;
	typedef int64_t 		INT64;
	typedef unsigned int 	UINT32;
	typedef int32_t 		INT32;
	typedef int16_t 		INT16;
	typedef uint16_t 		UINT16;
	typedef uint8_t 		UINT8;
	typedef int8_t 			INT8;

	class SystemInterface_COE : public SystemInterfaceBase
	{
	public:
		SystemInterface_COE() : _system_time_base(0LL)
		{

		}

	protected:
		template<typename T>
		inline int _readSDO(UINT16 position, UINT16 index, UINT8 subIndex, T * target)
		{
			size_t result_size;
			UINT32 abort_code;

			int error_code = ecrt_master_sdo_upload(_master, position, index, subIndex, (UINT8 *) target, sizeof(T), &result_size, &abort_code);

			if (error_code < 0)
				printf("Error in reading (or uploading) SDO (of index 0x%x:0x%x) with abort code = %d\n", index, subIndex, abort_code);

			return error_code;
		}
		template<typename T>
		inline int _writeSDO(UINT16 position, UINT16 index, UINT8 subIndex, T * target)
		{
			UINT32 abort_code;

			int error_code = ecrt_master_sdo_download(_master, position, index, subIndex, (UINT8 *) target, sizeof(T), &abort_code);

			if (error_code < 0)
				printf("Error in writing (or downloading) SDO (of index 0x%x:0x%x) with abort code = %d\n", index, subIndex, abort_code);

			return error_code;
		}

		inline int _computeCtrlWrd(int Idx, UINT16 StatWrd, UINT16 & ControlWrd)
		{
			if (!(StatWrd & (1<<STATUSWORD_OPERATION_ENABLE_BIT)))
			{
				if (!(StatWrd & (1<<STATUSWORD_SWITCHED_ON_BIT))) {
					if (!(StatWrd & (1<<STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
						if ((StatWrd & (1<<STATUSWORD_FAULT_BIT))) {
							ControlWrd = 0x80; //fault reset
							return 0;
						}
						else
						{
							ControlWrd = 0x06; //shutdown
							return 0;
						}
					}
					else
					{
						ControlWrd = 0x07; //switch on
						return 1;
					}
				}
				else
				{
					ControlWrd = 0x0F; //switch on
					return 1;
				}
			}
			else
			{
				ControlWrd = 0x0F; //switch on
				return 1;
			}

			ControlWrd = 0;
			return 0;
		}

		uint64_t _getSysTimeDC(void)
		{
			RTIME time = rt_timer_read();

			if (0 > time) {
				//rt_printf("system_time_base: %lld, time: %llu\n", system_time_base, time);
				return time;
			}
			else {
				return time - 0;
			}
		}

	protected:
		/* EtherCAT Master */
		ec_master_t	*_master;
		UINT32 	_cycle_ns;
		INT64	_system_time_base;
	};
}


#endif /* SYSTEMINTERFACE_COE_H_ */
