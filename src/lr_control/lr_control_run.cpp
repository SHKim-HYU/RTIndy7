#include "lr_control_run.h"
#include "waypoint.h"

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
	std::vector<JVec> joint_way_points_l,joint_way_points_r;
	std::vector<double> joint_delays;
	bool is_set_traj = 0;
	JVec q_home_l,q_home_r;
	
	q_home_l<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;
	q_home_r<<-0.14558084, -0.813371 ,-1.3075236,-0.614656,-1.03065,0.0693011;
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
		Vector3d g_;
		g_<<0 ,8.487,-4.9;		
		
		if(system_ready){
			if(is_set_traj==0){
				info.des_l.q = info.act_l.q;
				info.des_l.q_dot=JVec::Zero();
				info.des_l.q_ddot=JVec::Zero();
				joint_way_points_l.push_back(info.act_l.q);
				joint_way_points_l.push_back(info.act_l.q);
				joint_way_points_l.push_back(JVec::Zero());
				joint_way_points_l.push_back(q_home_l);

				info.des_r.q = info.act_r.q;
				info.des_r.q_dot=JVec::Zero();
				info.des_r.q_ddot=JVec::Zero();
				joint_way_points_r.push_back(info.act_r.q);
				joint_way_points_r.push_back(info.act_r.q);
				joint_way_points_r.push_back(JVec::Zero());
				joint_way_points_r.push_back(q_home_r);

				joint_delays.push_back(5.0);
				joint_delays.push_back(5.0);
				joint_delays.push_back(5.0);
				gt = 0;
				is_set_traj=1;
			}
			lr_traj.WayPointJointTrajectory(joint_way_points_l, joint_delays, gt, info.des_l.q, info.des_l.q_dot, info.des_l.q_ddot);
			lr_traj.WayPointJointTrajectory(joint_way_points_r, joint_delays, gt, info.des_r.q, info.des_r.q_dot, info.des_r.q_ddot);
			


		    JVec torques_l = lr::GravityForces(info.act_l.q,g_,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			JVec torques_r = lr::GravityForces(info.act_r.q,g_,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			lr_control.g = g_;
            JVec torq_l = lr_control.HinfControl(info.act_l.q,info.act_l.q_dot,info.des_l.q,info.des_l.q_dot,info.des_l.q_ddot,JVec::Zero());
			JVec torq_r = lr_control.HinfControl(info.act_r.q,info.act_r.q_dot,info.des_r.q,info.des_r.q_dot,info.des_r.q_ddot,JVec::Zero());
            info.des_l.tau = torq_l;
			info.des_r.tau = torq_r;
		}
		else
		{
			std::cout<<"isReady()?" << nrmk_master.isSystemReady()<<std::endl;
			lr_control.g = g_;
		    JVec torques_l = lr::GravityForces(info.act_l.q,g_,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			JVec torques_r = lr::GravityForces(info.act_r.q,g_,lr_control.Mlist,lr_control.Glist,lr_control.Slist,1);
			JVec torq_l = lr_control.HinfControl(info.act_l.q,info.act_l.q_dot,JVec::Zero(),JVec::Zero(),JVec::Zero(),JVec::Zero());
            info.des_l.tau = torques_l;
			info.des_r.tau = torques_r;
			
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