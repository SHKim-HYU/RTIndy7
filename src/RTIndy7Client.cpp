/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "RTIndy7Client.h"


JointInfo info;
JointInfo sim;

// MR_Indy7 mr_indy7;


// Xenomai RT tasks
RT_TASK RTIndy7_task;
#ifdef __BULLET__
RT_TASK bullet_task;
#endif
#ifdef __POCO__
RT_TASK poco_task;
#endif
#ifdef __CASADI__
RT_TASK indysim_task;
CS_Indy7 cs_indy7;
CS_Indy7 cs_nom_indy7;
CS_Indy7 cs_sim_indy7;
#endif
RT_TASK safety_task;
RT_TASK print_task;

// #ifdef __BULLET__
// b3RobotSimulatorClientAPI* sim;
// #endif

//For Trajectory management
//Task

//////////////////////////////////////////////////////////////////

void signal_handler(int signum);

void saveLogData(){}

int initAxes()
{
	for (int i = 0; i < NUM_AXIS; i++)
	{	
		Axis[i].setGearRatio(gearRatio[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ENC_CORE);
		Axis[i].setTauADC(TauADC[i]);
		Axis[i].setTauK(TauK[i]);
		Axis[i].setZeroPos(zeroPos[i]);

		Axis[i].setDirQ(dirQ[i]);
		Axis[i].setDirTau(dirTau[i]);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
	}
	
	return 1;
}
/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
	    	info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
#ifdef __RP__
	    	info.q_target(6)=0.0;
#endif
	    	traj_time = 10.0;
	    	// motion++;
	        break;
	    case 2:
	    	// info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	// info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
			info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
#ifdef __RP__
	    	info.q_target(6)=0.0;
#endif
	    	// info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	// info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	traj_time = 10.0;
	    	motion++;
	    	// motion=1;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
#ifdef __RP__
	    	info.q_target(6)=0.0;
#endif
	    	// info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	// info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 10.0;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
#ifdef __RP__
	    	info.q_target(6)=0.0;
#endif
	    	traj_time = 10.0;
	    	motion=1;
	    	break;
	    default:
	    	info.q_target(0)=info.act.q(0); 	info.q_target(1)=info.act.q(1); 	info.q_target(2)=info.act.q(2);
	    	info.q_target(3)=info.act.q(3); 	info.q_target(4)=info.act.q(4); 	info.q_target(5)=info.act.q(5);
#ifdef __RP__
	    	info.q_target(6)=0.0;
#endif
	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<NUM_AXIS;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();

		info.des.q(i)=Axis[i].getDesPosInRad();
		info.des.q_dot(i)=Axis[i].getDesVelInRad();
		info.des.q_ddot(i)=Axis[i].getDesAccInRad();
	}
}


int compute()
{
	// Update Robot
	cs_indy7.updateRobot(info.act.q , info.act.q_dot);
	// Update nominal
	cs_nom_indy7.updateRobot(info.nom.q , info.nom.q_dot);

	SE3 T_ee = cs_indy7.computeFK(info.act.q);
	info.act.x << T_ee(0,3), T_ee(1,3), T_ee(2,3), 0, 0, 0;

	// rt_printf("T_ee:\n");
	// rt_printf("%lf, %lf, %lf, %lf\n", T_ee(0,0), T_ee(0,1), T_ee(0,2), T_ee(0,3));
	// rt_printf("%lf, %lf, %lf, %lf\n", T_ee(1,0), T_ee(1,1), T_ee(1,2), T_ee(1,3));
	// rt_printf("%lf, %lf, %lf, %lf\n", T_ee(2,0), T_ee(2,1), T_ee(2,2), T_ee(2,3));
	// rt_printf("%lf, %lf, %lf, %lf\n", T_ee(3,0), T_ee(3,1), T_ee(3,2), T_ee(3,3));

	Jacobian J_b = cs_indy7.getJ_b();
	info.act.x_dot = J_b*info.act.q_dot;
	info.act.tau_ext = J_b.transpose()*info.act.F;
	
	// rt_printf("tau_ext:\n");
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf\n\n", info.act.tau_ext(0), info.act.tau_ext(1), info.act.tau_ext(2), info.act.tau_ext(3), info.act.tau_ext(4), info.act.tau_ext(5));

	info.act.tau_fric = cs_indy7.FrictionEstimation(info.act.q_dot);
	// info.act.tau_fric = JVec::Zero();
	
	return 0;
}

void controller()
{
	//nominal controller
	// info.nom.tau = cs_nom_indy7.ComputedTorqueControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot);
	info.nom.tau = cs_nom_indy7.ComputedTorqueControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot) + 1* info.act.tau_ext;
	// info.nom.tau = cs_nom_indy7.HinfControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot);
		
	info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);
	
	// // Bound compensation
	// Comp_alpha = 1500;
	// Comp_bound << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;
	// Comp_bound = Comp_alpha*Comp_bound;
	
	// for(int i=0; i<NUM_AXIS; i++)
	// {
	// 	if(abs(info.act.tau_aux(i))<=Comp_bound(i))
	// 	{
	// 		Comp_(i) = info.act.tau_aux(i);
	// 		Est_(i) = 0.0;
	// 	}
	// 	else if(abs(info.act.tau_aux(i))>Comp_bound(i))
	// 	{
	// 		Comp_(i) = Comp_bound(i)*sign(info.act.tau_aux(i));
	// 		Est_(i) = info.act.tau_aux(i)-Comp_bound(i)*sign(info.act.tau_aux(i));
	// 	}
	// }
	// info.des.tau = info.nom.tau - Comp_ + info.act.tau_fric;
	// info.nom.tau += Est_;

	// NRIC result
	// info.des.tau = info.nom.tau - info.act.tau_aux + info.act.tau_fric;
	info.des.tau = info.nom.tau - info.act.tau_aux;

	// Hinf ctrl
	// info.des.tau = cs_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot) - 1* info.act.tau_ext;
	// info.des.tau = cs_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot);
	// info.des.tau = cs_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot) + info.act.tau_fric;

	// IDC
	// info.des.tau = cs_indy7.ComputedTorqueControl(info.act.q, info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot);
	
	// Gravity
	// info.des.tau = cs_indy7.computeG( info.act.q ) - 1* info.act.tau_ext; // calcTorque
	// info.des.tau = cs_indy7.computeG( info.act.q ); // calcTorque
	// info.des.G = cs_indy7.computeG( info.act.q ); // calcTorque

	// Update nominal
	cs_nom_indy7.computeRK45(info.nom.q, info.nom.q_dot, info.nom.tau, info.nom.q, info.nom.q_dot);
}

void readEcatData(){
	// Drive
	nrmk_master.readBuffer(0x60410, StatusWord);
	nrmk_master.readBuffer(0x60640, ActualPos);
	nrmk_master.readBuffer(0x606c0, ActualVel);
	nrmk_master.readBuffer(0x60770, ActualTor);
	nrmk_master.readBuffer(0x60610, ModeOfOperationDisplay);

#ifdef __CB__
	// IO Module
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x61001, StatusCode);
	nrmk_master.readBuffer(0x61002, DI5V);
	nrmk_master.readBuffer(0x61003, DI1);
	nrmk_master.readBuffer(0x61004, DI2);
	nrmk_master.readBuffer(0x61005, AI1);
	nrmk_master.readBuffer(0x61006, AI2);
	nrmk_master.readBuffer(0x61007, FTRawFxCB);
	nrmk_master.readBuffer(0x61008, FTRawFyCB);
	nrmk_master.readBuffer(0x61009, FTRawFzCB);
	nrmk_master.readBuffer(0x610010, FTRawTxCB);
	nrmk_master.readBuffer(0x610011, FTRawTyCB);
	nrmk_master.readBuffer(0x610012, FTRawTzCB);
	nrmk_master.readBuffer(0x610013, FTOverloadStatusCB);
	nrmk_master.readBuffer(0x610014, FTErrorFlagCB);
	nrmk_master.readBuffer(0x610015, RS485RxCnt);
	nrmk_master.readBuffer(0x610016, RS485RxD0);
	nrmk_master.readBuffer(0x610017, RS485RxD1);
	nrmk_master.readBuffer(0x610018, RS485RxD2);
	nrmk_master.readBuffer(0x610019, RS485RxD3);
	nrmk_master.readBuffer(0x610020, RS485RxD4);
	nrmk_master.readBuffer(0x610021, RS485RxD5);
	nrmk_master.readBuffer(0x610022, RS485RxD6);
	nrmk_master.readBuffer(0x610023, RS485RxD7);
	nrmk_master.readBuffer(0x610024, RS485RxD8);
	nrmk_master.readBuffer(0x610025, RS485RxD9);
#endif

	// Tool
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x60001, IStatus);
	nrmk_master.readBuffer(0x60002, IButton);
	nrmk_master.readBuffer(0x60003, FTRawFx);
	nrmk_master.readBuffer(0x60004, FTRawFy);
	nrmk_master.readBuffer(0x60005, FTRawFz);
	nrmk_master.readBuffer(0x60006, FTRawTx);
	nrmk_master.readBuffer(0x60007, FTRawTy);
	nrmk_master.readBuffer(0x60008, FTRawTz);
	nrmk_master.readBuffer(0x60009, FTOverloadStatus);
	nrmk_master.readBuffer(0x600010, FTErrorFlag);	
	
	for(int i=0; i<NUM_AXIS;i++)
	{

		Axis[i].setCurrentPosInCnt(ActualPos[i+NUM_IO_MODULE]);
		Axis[i].setCurrentVelInCnt(ActualVel[i+NUM_IO_MODULE]);
		Axis[i].setCurrentTorInCnt(ActualTor[i+NUM_IO_MODULE]);
		
		Axis[i].setCurrentTime(gt);

		info.act.q(i) = Axis[i].getCurrPosInRad();
		info.act.q_dot(i) = Axis[i].getCurrVelInRad();
		info.act.tau(i) = Axis[i].getCurrTorInNm();

		if(!system_ready)
		{
			Axis[i].setTarPosInRad(info.act.q(i));
			Axis[i].setDesPosInRad(info.act.q(i));
			info.nom.q = info.act.q;
			info.nom.q_dot = info.act.q_dot;
			info.nom.tau = info.act.tau;
		}

	}
	
	// Update RFT data
	info.act.F(0) = (double)FTRawFx[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(1) = (double)FTRawFy[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(2) = (double)FTRawFz[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(3) = (double)FTRawTx[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(4) = (double)FTRawTy[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(5) = (double)FTRawTz[NUM_IO_MODULE+NUM_AXIS] / torque_divider;

#ifdef __CB__
	info.act.F_CB(0) = (double)FTRawFxCB[0] / force_divider;
	info.act.F_CB(1) = (double)FTRawFyCB[0] / force_divider;
	info.act.F_CB(2) = (double)FTRawFzCB[0] / force_divider;
	info.act.F_CB(3) = (double)FTRawTxCB[0] / torque_divider;
	info.act.F_CB(4) = (double)FTRawTyCB[0] / torque_divider;
	info.act.F_CB(5) = (double)FTRawTzCB[0] / torque_divider;
#endif

	// info.act.F<<(double)FTRawFx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFz[NUM_IO_MODULE+NUM_AXIS]
	//           <<(double)FTRawTx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTz[NUM_IO_MODULE+NUM_AXIS];
}

void writeEcatData(){
	for(int i=0;i<NUM_AXIS;i++){
		Axis[i].setDesTorInNm(info.des.tau(i));
		TargetTor[i+NUM_IO_MODULE]=Axis[i].getDesTorInCnt();
	}
	// TO DO: write data to actuators in EtherCAT system interface
	nrmk_master.writeBuffer(0x60710, TargetTor);
	// nrmk_master.writeBuffer(0x60600, ModeOfOperation);
}

// RTIndy7_task
void RTIndy7_run(void *arg)
{
	RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;

		/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6d::Zero();
	info.des.F_CB = Vector6d::Zero();
	info.act.tau_aux = JVec::Zero();

	int ft_init_cnt = 0;

	// Real
	NRIC_Kp << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0;
	NRIC_Ki << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
	NRIC_K_gamma << 850.0, 850.0, 850.0, 550.0, 550.0, 550.0;
	// NRIC_Kp << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
	// NRIC_Ki << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
	// NRIC_K_gamma << 285.0, 285.0, 285.0, 155.0, 155.0, 155.0;
	cs_indy7.setNRICgain(NRIC_Kp, NRIC_Ki, NRIC_K_gamma);

	// nominal
	Kp_n << 70.0, 70.0, 40.0, 25.0, 15.0, 15.0;
	Kd_n << 7.0, 7.0, 4.0, 2.5, 1.5, 1.5;
	Ki_n = JVec::Zero();
	// Kp_n << 0.001, 0.001, 0.001, 0.00025, 0.00025, 0.00001;
	// Kd_n << 0.0001, 0.0001, 0.0001, 0.000025, 0.000025, 0.000001;
	// Ki_n = JVec::Zero();

	Hinf_Ki_n << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    Hinf_Kp_n << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0;
	Hinf_Kv_n = JVec::Zero();
    Hinf_K_gamma_n << 1.0+1.0/invL2sqr_1, 1.0+1.0/invL2sqr_2, 1.0+1.0/invL2sqr_3, 1.0+1.0/invL2sqr_4, 1.0+1.0/invL2sqr_5, 1.0+1.0/invL2sqr_6;


	cs_nom_indy7.setPIDgain(Kp_n, Kd_n, Ki_n);
	cs_nom_indy7.setHinfgain(Hinf_Kp_n, Hinf_Kv_n, Hinf_Ki_n, Hinf_K_gamma_n);

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

			// Trajectory Generation
			trajectory_generation();
			
			//[ToDo] Add MPC Function 
			compute();	

			// Calculate controller
			controller();
		}
		else
		{
			info.des.tau = cs_indy7.computeG( info.act.q ); // calcTorque
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
				// Stop FT Sensor
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_STOP_DEVICE;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_STOP_DEVICE;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
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
			else if(ft_init_cnt==2)
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
			else if(ft_init_cnt==3)
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
			else
				system_ready=1;	//all drives have been done

			gt+= period;
			
			if (periodEcat > worstEcat)	worstEcat = periodEcat;
			if (periodBuffer > worstBuffer)	worstBuffer = periodBuffer;
			if (periodCompute > worstCompute) worstCompute = periodCompute;
			if (periodCycle > cycle_ns) overruns++;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
	}
}

#ifdef __CASADI__
// IndySim task
void indysim_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns); 

    Jacobian J_b;

	int cnt = -1;    

	Kp_s << 70.0, 70.0, 40.0, 25.0, 15.0, 15.0;
	Kd_s << 7.0, 7.0, 4.0, 2.5, 1.5, 1.5;
	Ki_s = JVec::Zero();

	Hinf_Ki_s << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    Hinf_Kp_s << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0;
	Hinf_Kv_s = JVec::Zero();
    Hinf_K_gamma_s << 1.0+1.0/invL2sqr_1, 1.0+1.0/invL2sqr_2, 1.0+1.0/invL2sqr_3, 1.0+1.0/invL2sqr_4, 1.0+1.0/invL2sqr_5, 1.0+1.0/invL2sqr_6;

#ifdef __RP__
	Hinf_Ki_s(NUM_AXIS-1) = 100.0;
	Hinf_Kp_s(NUM_AXIS-1) = 20;
	Hinf_K_gamma_s(NUM_AXIS-1) = 1.0+1.0/invL2sqr_7;
#endif

	cs_sim_indy7.setPIDgain(Kp_s, Kd_s, Ki_s);
	cs_sim_indy7.setHinfgain(Hinf_Kp_s, Hinf_Kv_s, Hinf_Ki_s, Hinf_K_gamma_s);

	// loop
	while(1)
	{
		if(system_ready)
		{
			beginCycle = rt_timer_read();

			
			if(motion>1 && cnt == -1)
			{
				cnt=0;
			}

			/////////////////  RK4   //////////////////
			if(cnt == 0)
			{
				cs_sim_indy7.computeRK45(info.act.q, JVec::Zero(), JVec::Zero(), info.sim.q, info.sim.q_dot);
				// cs_sim_indy7.computeRK45(info.act.q, info.act.q_dot, info.act.tau, info.sim.q, info.sim.q_dot);

				cnt++;
			}
			else if(cnt==1)
			{
				cs_sim_indy7.computeRK45(info.sim.q, info.sim.q_dot, info.sim.tau, info.sim.q, info.sim.q_dot);
			}
			
			// update robot & compute dynamics
			cs_sim_indy7.updateRobot(info.sim.q, info.sim.q_dot);
			
			// Calculate Joint controller
			J_b = cs_sim_indy7.getJ_b();
			info.sim.tau_ext = J_b.transpose()*info.act.F;

			// info.sim.tau = cs_sim_indy7.HinfControl(info.sim.q, info.sim.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot);
			info.sim.tau = cs_sim_indy7.ComputedTorqueControl(info.sim.q, info.sim.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot);
			
			endCycle = rt_timer_read();
			periodIndysim = (unsigned long) endCycle - beginCycle;
		
		}
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
#endif


#ifdef __BULLET__
// Bullet task
void bullet_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, 40*cycle_ns);

	//---------BULLET SETUP START------------------
	b3PhysicsClientHandle b3client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(b3client))
	{
	printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	exit(0);
	}
	b3RobotSimulatorClientAPI_InternalData b3data;
	b3data.m_physicsClientHandle = b3client;
	b3data.m_guiHelper = 0;
	b3RobotSimulatorClientAPI_NoDirect b3sim;
	b3sim.setInternalData(&b3data);

	b3sim.setTimeStep(FIXED_TIMESTEP);
	b3sim.resetSimulation();
	b3sim.setGravity( btVector3(0 , 0 , -9.8));

	// [ToDo] model path update
	int robotId = b3sim.loadURDF("/home/robot/robot_ws/RTIndy7/description/indy7.urdf");
	// int robotId = b3sim.loadURDF("quadruped/minitaur.urdf");
	b3sim.setRealTimeSimulation(false);
	Bullet_Indy7 bt3indy7(&b3sim,robotId);
	
	rt_printf("Start Bullet\n");
	while (1)
	{
		beginCycle = rt_timer_read();
		if(!system_ready)
		{
			bt3indy7.reset_q(&b3sim, info.nom.q);
		}
		else
		{
			bt3indy7.reset_q(&b3sim, info.nom.q);
			b3sim.stepSimulation();

		}
		endCycle = rt_timer_read();
		periodBullet = (unsigned long) endCycle - beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
#endif

#ifdef __POCO__
void poco_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, 40*cycle_ns); // 100Hz

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
#endif

// Safety task
void safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NUM_AXIS;i++)
			{
				if(Axis[i].isLimitReached())
				{
					for(int i=0;i<NUM_AXIS;i++)
						nrmk_master.setServoOff(i+NUM_IO_MODULE);
					rt_printf("Servo Off!!\n");
					break;
				}
			}
		}
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
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
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*1);
	
	string filename = "robot_log.csv";
	ifstream checkFile(filename);

	if (checkFile.is_open())
	{
		checkFile.close();
		remove(filename.c_str());
	}

	ofstream newFile(filename);
	if(newFile.is_open())
	{
		newFile<<"Time, q_r1, q_r2, q_r3, q_r4, q_r5, q_r6, q_r3, dq_r1, dq_r2, dq_r3, dq_r4, dq_r5, dq_r6, t_r1, t_r2, t_r3, t_r4, t_r5, t_r6, G_r1, G_r2, G_r3, G_r4, G_r5, G_r6, "
		"q_n1, q_n2, q_n3, q_n4, q_n5, q_n6, q_n3, dq_n1, dq_n2, dq_n3, dq_n4, dq_n5, dq_n6, t_n1, t_n2, t_n3, t_n4, t_n5, t_n6, "
		"qd1, qd2, qd3, qd4, qd5, qd6, qd3, dqd1, dqd2, dqd3, dqd4, dqd5, dqd6\n";
		newFile.close();
	}


	ofstream csvFile(filename, ios_base::app);


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
			{
				rt_printf("RxDomain: Offline\n");
			}
			if (!nrmk_master.getTxDomainStatus())
			{
				rt_printf("TxDomain: Offline\n");
			}
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
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf \n",info.nom.q(j), info.nom.q_dot(j));
				rt_printf("\t SimPos: %lf, SimVel: %lf \n",info.sim.q(j), info.sim.q_dot(j));
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
				// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
				rt_printf("\t TarTor: %f, ActTor: %lf, NomTor: %lf\n\t SimTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j),  info.sim.tau(j), info.act.tau_ext(j));
			}
			rt_printf("FK: x: %lf, y: %lf, z: %lf\n", info.act.x(0), info.act.x(1), info.act.x(2));
			rt_printf("ReadFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			// rt_printf("ReadFT_CB: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F_CB(0),info.act.F_CB(1),info.act.F_CB(2),info.act.F_CB(3),info.act.F_CB(4),info.act.F_CB(5));
			// rt_printf("overload: %u, error: %u\n", FTOverloadStatus[NUM_IO_MODULE+NUM_AXIS], FTErrorFlag[NUM_IO_MODULE+NUM_AXIS]);

			// rt_printf("\n");



		if(csvFile.is_open())
		{
			csvFile<<gt<<", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.act.q(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.act.q_dot(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.des.tau(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.act.F(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.nom.q(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.nom.q_dot(i) << ", ";				
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.nom.tau(i) << ", ";
			for (int i = 0; i < NUM_AXIS; ++i) csvFile<<info.des.q(i) << ", ";
			for (int i = 0; i < NUM_AXIS-1; ++i) csvFile<<info.des.q_dot(i) << ", ";
			csvFile<<info.des.q_dot(NUM_AXIS-1)<<"\n";
		}
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
	csvFile.close();
}


/****************************************************************************/
void signal_handler(int signum)
{
	FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_STOP_DEVICE;
	// FTConfigParamCB[0]=FT_STOP_DEVICE;
	nrmk_master.writeBuffer(0x70003, FTConfigParam);
	// nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
	nrmk_master.processRxDomain();


	rt_task_delete(&RTIndy7_task);
	
	rt_task_delete(&safety_task);
	rt_task_delete(&print_task);
#ifdef __CASADI__
	rt_task_delete(&indysim_task);
#endif
#ifdef __BULLET__
	rt_task_delete(&bullet_task);
#endif
#ifdef __POCO__
	rt_task_delete(&poco_task);
#endif

	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGTSTP)
		printf("╔═══════════════[SIGNAL INPUT SIGTSTP]══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
    
    nrmk_master.deinit();
    exit(1);
}


/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGTSTP, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	cpu_set_t cpuset_qt, cpuset_rt1, cpuset_rt2;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt1);  
    CPU_ZERO(&cpuset_rt2);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt1);  
    CPU_SET(5, &cpuset_rt2);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	// cycle_ns = 1000000; // nanosecond -> 1kHz
	cycle_ns = 250000; // nanosecond -> 4kHz
	// cycle_ns = 125000; // nanosecond -> 8kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	// mr_indy7=MR_Indy7();
	// mr_indy7.MRSetup();

	#ifndef __RP__
	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_sim_indy7=CS_Indy7();
	cs_sim_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	#else
	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indyrp2/indyrp2.json");
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indyrp2/indyrp2.json");
	cs_sim_indy7=CS_Indy7();
	cs_sim_indy7.CSSetup("../lib/URDF2CASADI/indyrp2/indyrp2.json");
	#endif

	MAX_TORQUES<<MAX_TORQUE_1,MAX_TORQUE_2,MAX_TORQUE_3,MAX_TORQUE_4,MAX_TORQUE_5,MAX_TORQUE_6;

	// For CST (cyclic synchronous torque) control
	if (nrmk_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
	{
		printf("System Initialization Failed\n");
	    return 0;
	}
	for (int i = 0; i < NUM_AXIS; ++i)
		ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;

	// For trajectory interpolation
	initAxes();

	for(int i=0;i<NUM_SLAVES;i++)
		nrmk_master.setServoOn(i);
	
	// RTIndy7_task: create and start
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 99, 0);
    rt_task_set_affinity(&RTIndy7_task, &cpuset_rt1);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

#ifdef __BULLET__
	// RTIndy7 simulation
	rt_task_create(&bullet_task, "bullet_task", 0, 80, 0);
	rt_task_start(&bullet_task, &bullet_run, NULL);
#endif

#ifdef __POCO__
	// RTIndy7 visualization	
	rt_task_create(&poco_task, "poco_task", 0, 90, 0);
    rt_task_set_affinity(&poco_task, &cpuset_rt2);
	rt_task_start(&poco_task, &poco_run, NULL);
#endif

#ifdef __CASADI__
	// RTIndy7 simulation
	rt_task_create(&indysim_task, "indysim_task", 0, 96, 0);
    rt_task_set_affinity(&indysim_task, &cpuset_rt2);
	rt_task_start(&indysim_task, &indysim_run, NULL);
#endif

	// RTIndy7 safety
	rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &safety_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt2);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
	pause();
	/*
	while (1)
	{
		usleep(1e5);
	}
	*/
	// Finalize
	signal_handler(0);

    return 0;
}



