#include "tcp_run.h"
unsigned long periodPoco = 0;
void rt_tcp_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, 40*CYCLE_NS); // 100Hz

	rt_printf("Start Poco\n");
	while (1)
	{
		beginCycle = rt_timer_read();
		if(!system_ready)
		{
			
		}
		else
		{
			
		}
		endCycle = rt_timer_read();
		periodPoco = (unsigned long) endCycle - beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
