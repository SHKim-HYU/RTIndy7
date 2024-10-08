#include "../global_vars.h"
// Safety task
void rt_safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NRMK_DRIVE_NUM]={0,};

	rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NRMK_DRIVE_NUM;i++)
			{
				if(Axis[i].isLimitReached())
				{
					ecat_drive[i].setServoOff();
					rt_printf("Servo Off!!\n");
					// break;
				}
			}
			if(info.nom.q.array().isNaN().any())
			{
				rt_printf("nan occured\n");
				for(int i=0; i<NRMK_DRIVE_NUM; i++)
					ecat_drive[i].setServoOff();
			}

		}
	}
}