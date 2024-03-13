#include "ft_run.h"
#include "FTread.h"

void rt_ft_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, 1*CYCLE_NS); // 1ms

	rt_printf("Start FT\n");

    int channel =0;
    int canId = 1;
    FTread ft = FTread(channel,canId);
    //ft.setCutOffFreq(105);
	ft.setCutOffFreq(10);
	unsigned long step	;
	SE3 T_ES;
			T_ES<<  1,0,0,0,
					0,1,0,0,
					0,0,1,0,
					0,0,0,1;			
	Matrix6d AdT_ES = lr::Ad(T_ES).transpose();
	while (1)
	{	
		beginCycle = rt_timer_read();
		if(system_ready)
		{	
            ft.readData();
		     
            info.act.F_ext = AdT_ES*ft.filtered_FT;
            //rt_printf("%.3f\n",info.act.F[0]);

		}   
		else
		{
            ft.readData();
            info.act.F_ext = AdT_ES*ft.filtered_FT;
            //rt_printf("%.3f\n",info.act.F[0]);

		}
		endCycle = rt_timer_read();
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
