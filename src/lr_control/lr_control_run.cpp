#include "lr_control_run.h"

#include "../ecat/ecat_run.h"


void rt_lr_control_run(void* param){
    RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	// nrmk_master.syncEcatMaster();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, CYCLE_NS);

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6d::Zero();
	info.des.F_CB = Vector6d::Zero();

	JVec eint = JVec::Zero();
	JVec e = JVec::Zero();

	int ft_init_cnt = 0;

	while (run)
	{
		beginCycle = rt_timer_read();
		periodEcat = 0;
		periodBuffer = 0;
		beginRead = rt_timer_read();
		nrmk_master.processTxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginRead;
		// Read data in EtherCAT Buffer
		beginReadbuf = rt_timer_read();
		readEcatData();	
		periodBuffer += (unsigned long) rt_timer_read() - beginReadbuf;
		beginCompute = rt_timer_read();
		if(system_ready){

		    JVec torques = lr::GravityForces(info.act.q,lr_control.g,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
            info.des.tau = torques;
		}
		else
		{
		    JVec torques = lr::GravityForces(info.act.q,lr_control.g,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
            info.des.tau = torques;
			
		}
		periodCompute  = (unsigned long) rt_timer_read() - beginCompute;
		// Write data in EtherCAT Buffer
		beginWritebuf = rt_timer_read();
		writeEcatData();
		periodBuffer += (unsigned long) rt_timer_read() - beginWritebuf;
		beginWrite = rt_timer_read();
		nrmk_master.processRxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginWrite;

		endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		
		if (nrmk_master.isSystemReady())
		{	
			if(ft_init_cnt==0)
			{
				// Set bias
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Set Filter 100Hz
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Start
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else
				system_ready=1;	//all drives have been done

			gt+= period;
			
			if (periodEcat > worstEcat)	worstEcat = periodEcat;
			if (periodBuffer > worstBuffer)	worstBuffer = periodBuffer;
			if (periodCompute > worstCompute) worstCompute = periodCompute;
			if (periodCycle > CYCLE_NS) overruns++;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
	}
}