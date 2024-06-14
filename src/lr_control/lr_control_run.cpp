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

	double dt = 0.001;

	JVec Hinf_K,gamma;
	Hinf_K<<50,50,30,10,10,10;
	gamma << 1000,1000,800,600,600,600;
	LR_Trajectory lr_traj;

	std::vector<JVec> way_points;
	std::vector<double> delays;
	int is_traj_set_=0;


	JVec q_home;
	q_home<<0,0,-1.5708,0,-1.5708,0;
	
	JVec torques=JVec::Zero();
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

		    //JVec torques = lr::GravityForces(info.act.q,lr_control.g,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			
			//JVec torques = lr_control.GravityForces(info.act.q);		

			if(is_traj_set_<1){
				way_points.push_back(info.act.q);
				way_points.push_back(JVec::Zero());
				way_points.push_back(q_home);
				way_points.push_back(JVec::Zero());
				way_points.push_back(q_home);
				delays.push_back(5);
				delays.push_back(5);
				delays.push_back(5);
				delays.push_back(5);
				is_traj_set_=1;
				//std::cout<<"111:"<<is_traj_set_<<std::endl;
				// lr_traj.WayPointJointTrajectory(way_points,delays , gt, info.des.q, info.des.q_dot, info.des.q_ddot, 20);

				 torques = lr_control.GravityForces(info.act.q);
			}else{
				 lr_traj.WayPointJointTrajectory(way_points,delays , gt, info.des.q, info.des.q_dot, info.des.q_ddot, 20);
				//std::cout<<"222:"<<info.des.q.transpose()<<std::endl;
				// torques = lr_control.GravityForces(info.act.q);
				  torques = lr_control.HinfControl(info.act.q, info.act.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot, eint,  dt,  Hinf_K,  gamma);
			}
			
			
            info.des.tau = torques;
		}
		else
		{
		    //JVec torques = lr::GravityForces(info.act.q,lr_control.g,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			JVec torques = lr_control.GravityForces(info.act.q);
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