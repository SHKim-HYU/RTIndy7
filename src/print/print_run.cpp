#include "../global_vars.h"
void rt_print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
		
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*1);

	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;

			rt_printf("Time=%0.3lfs, cycle_dt=%lius, worst_cycle=%lius, overrun=%d\n", gt, periodCycle/1000, worstLoop/1000, overruns);
			// /*
            rt_printf("Arm Data\n");
			for(int j=0; j<ROBOT_DOF; ++j){
				rt_printf("ID: %d", j);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf, NomAcc :%lf\n",info.nom.q(j), info.nom.q_dot(j), info.nom.q_ddot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j), info.act.tau_ext(j));
			}
			// rt_printf("V: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.x_dot(0),info.act.x_dot(1),info.act.x_dot(2),info.act.x_dot(3),info.act.x_dot(4),info.act.x_dot(5),info.act.x_dot(6),info.act.x_dot(7),info.act.x_dot(8));
			// rt_printf("dV: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.x_ddot(0),info.act.x_ddot(1),info.act.x_ddot(2),info.act.x_ddot(3),info.act.x_ddot(4),info.act.x_ddot(5),info.act.x_ddot(6),info.act.x_ddot(7),info.act.x_ddot(8));
			rt_printf("resFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			rt_printf("tau_ext: %lf, %lf, %lf\n", info.act.tau_ext(0), info.act.tau_ext(1), info.act.tau_ext(2));			
			rt_printf("Tdes: \t%lf, %lf, %lf, %lf\n", info.des.T(0,0), info.des.T(0,1), info.des.T(0,2), info.des.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(1,0), info.des.T(1,1), info.des.T(1,2), info.des.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(2,0), info.des.T(2,1), info.des.T(2,2), info.des.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(3,0), info.des.T(3,1), info.des.T(3,2), info.des.T(3,3));
			rt_printf("T: \t%lf, %lf, %lf, %lf\n", info.nom.T(0,0), info.nom.T(0,1), info.nom.T(0,2), info.nom.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(1,0), info.nom.T(1,1), info.nom.T(1,2), info.nom.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(2,0), info.nom.T(2,1), info.nom.T(2,2), info.nom.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(3,0), info.nom.T(3,1), info.nom.T(3,2), info.nom.T(3,3));
			rt_printf("T: \t%lf, %lf, %lf, %lf\n", info.act.T(0,0), info.act.T(0,1), info.nom.T(0,2), info.nom.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(1,0), info.act.T(1,1), info.act.T(1,2), info.act.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(2,0), info.act.T(2,1), info.act.T(2,2), info.act.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(3,0), info.act.T(3,1), info.act.T(3,2), info.act.T(3,3));
			rt_printf("\n");
			rt_printf("\n");
			
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
				rt_printf(".");
				rt_printf("\n");
			}
		}
	}

}
