#include "print_run.h"


void rt_print_run(void* param){

    RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
    
	rt_task_set_periodic(NULL, TM_NOW, CYCLE_NS*1000);

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
			if (!nrmk_master.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Offline\n");
			if (!nrmk_master.getRxDomainStatus())
				rt_printf("RxDomain: Offline\n");
			if (!nrmk_master.getTxDomainStatus())
				rt_printf("TxDomain: Offline\n");
			// for(int i=0;i<NUM_AXIS;i++){
			// 	if (!nrmk_master.getAxisEcatStatus(i+NUM_IO_MODULE,slaveState[i]))
			// 	rt_printf("idx: %u, slaveState: %u\n",i+NUM_IO_MODULE,slaveState[i]);	
			// }
			for(int i=1;i<=NUM_AXIS;i++){
				if (!nrmk_master.getAxisEcatStatus(i,slaveState[i-1]))
				rt_printf("idx: %u, slaveState: %u",i,slaveState[i-1]);	
			}
			
			rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
			rt_printf("compute_dt= %lius, worst_dt= %lius, buffer_dt=%lius, ethercat_dt= %lius\n", periodCompute/1000, worstCompute/1000, periodBuffer/1000, periodEcat/1000);
			#ifdef __BULLET__
			rt_printf("Bullet_dt=%lius\n",periodBullet/1000);
			#endif
			#ifdef __CASADI__
			rt_printf("IndySim_dt=%lius\n",periodIndysim/1000);
			#endif
			for(int j=0; j<NUM_AXIS; ++j){
				rt_printf("ID: %d", j);
			// 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf \n",info.nom.q(j), info.nom.q_dot(j));
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
				// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
				rt_printf("\t TarTor: %f, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j), info.act.tau_ext(j));
			}

			rt_printf("ReadFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			rt_printf("ReadFT_CB: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F_CB(0),info.act.F_CB(1),info.act.F_CB(2),info.act.F_CB(3),info.act.F_CB(4),info.act.F_CB(5));
			rt_printf("overload: %u, error: %u\n", FTOverloadStatus[NUM_IO_MODULE+NUM_AXIS], FTErrorFlag[NUM_IO_MODULE+NUM_AXIS]);

			rt_printf("\n");

	}
}
}