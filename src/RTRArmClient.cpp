
// Automatically generated realtime application source file for STEP platforms
//
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP).
//
// Copyright (C) 2013-2015 Neuromeka <http://www.neuromeka.com>

//-system-/////////////////////////////////////////////////////////////////
#ifndef __XENO__
#define __XENO__
#endif
#include "RTRArmClient.h"
#include "MR_Indy7.h"

#define USE_DC_MODE

hyuEcat::Master ecatmaster;
hyuEcat::EcatNRMK_Indy_Tool ecat_nrmk_indy_tool[NUM_FT];
hyuEcat::EcatNRMK_Drive ecat_nrmk_drive[JOINTNUM];
JointInfo info;

MR_Indy7 mr_indy7;

// Xenomai RT tasks
RT_TASK RTRArm_task;
RT_TASK print_task;
RT_TASK plot_task;

// For RT thread management
static int run = 1;

double ECAT_ActualPos_zero[JOINTNUM] = {ZERO_POS_1, ZERO_POS_2,ZERO_POS_3, ZERO_POS_4, ZERO_POS_5, ZERO_POS_6};
double ActualPos_Rad[JOINTNUM] = {0.0,};
double ActualVel_Rad[JOINTNUM] = {0.0,};
double ActualVel_Rad_Old[JOINTNUM] = {0.0,};
double ActualAcc_Rad[JOINTNUM] = {0.0,};
double TargetPos_Rad[JOINTNUM] = {0.0,};
double TargetVel_Rad[JOINTNUM] = {0.0,};
double TargetAcc_Rad[JOINTNUM] = {0.0,};
double ECAT_calcTorq[JOINTNUM] = {0.0,};
double ECAT_TargetToq[JOINTNUM] = {0.0,};
INT32   MotorDir[JOINTNUM] = {1,1,1,1,1,1};
int   old_flag[JOINTNUM]={0,};

int limit_flag=0;

//For Trajectory management
//Task

double traq[JOINTNUM]={0.0, 0.0, 0.0, 0, 0.0, 0.0};
double traq_d[JOINTNUM]={0.0,};
double traq_dd[JOINTNUM]={0.0,};
int flag=0;
int rtsercan_fd  = -1;
void signal_handler(int signum);
void saveLogData(){}
/****************************************************************************/
void EncToRad(double wc, double dt)
{
	for(int i=0; i<JOINTNUM; i++)
	{
		if(old_flag[i]==0){
			ECAT_ActualPos_Old[i]=ECAT_ActualPos[i];
			old_flag[i]=1;
		}
		else if(old_flag[i]==1){
			//ECAT_ActualVel[i]=1.0/(1.0+dt*wc)*ECAT_ActualVel_Old[i]+dt*wc/(1.0+dt*wc)*(ECAT_ActualPos_Old[i]-ECAT_ActualPos[i])/dt;
			switch(i)
			{
				case 0:
					ActualPos_Rad[i]=-(double)ECAT_ActualPos[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121); //262144/2PI
					ActualVel_Rad[i]=-(double)ECAT_ActualVel[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121);
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];				
				case 1:
					ActualPos_Rad[i]=-(double)ECAT_ActualPos[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121); //262144/2PI
					ActualVel_Rad[i]=-(double)ECAT_ActualVel[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121);
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];
					break;
				case 2:
					ActualPos_Rad[i]=(double)ECAT_ActualPos[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121); //262144/2PI
					ActualVel_Rad[i]=(double)ECAT_ActualVel[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121);
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];
					break;
				case 3:
					ActualPos_Rad[i]=-(double)ECAT_ActualPos[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101); //262144/2PI
					ActualVel_Rad[i]=-(double)ECAT_ActualVel[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101);
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];
					break;
				case 4:
					ActualPos_Rad[i]=-(double)ECAT_ActualPos[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101); //262144/2PI
					ActualVel_Rad[i]=-(double)ECAT_ActualVel[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101);	
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];
					break;
				case 5:
					ActualPos_Rad[i]=(double)ECAT_ActualPos[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101); //262144/2PI
					ActualVel_Rad[i]=(double)ECAT_ActualVel[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101);
					ActualVel_Rad[i] =1.0/(1.0+dt*wc)*ActualVel_Rad_Old[i]+wc*dt/(1.0+dt*wc)*ActualVel_Rad[i];
					break;
			}
			//for Kinematics & Dynamics
			info.act.q(i)=ActualPos_Rad[i];
			info.act.dq(i)=ActualVel_Rad[i];
			ActualVel_Rad_Old[i]= ActualVel_Rad[i];
			//buffer for calculate velocity
			ECAT_ActualPos_Old[i]=ECAT_ActualPos[i];
			ECAT_ActualVel_Old[i]=ECAT_ActualVel[i];
		}
	}
}

void Robot_Limit()
{
	for(int i=0;i<JOINTNUM;i++)
	{
		if(abs(ActualVel_Rad[i])>4)
		{
			traq_d[i]=0;
			traq_dd[i]=0;
			limit_flag=1;
		}
	}
}
int isDriveInit(void)
{
	int elmo_count = 0;
	for(int i=0; i<JOINTNUM; ++i)
	{
		if(ecat_nrmk_drive[i].initialized())
			elmo_count++;
	}

	// for(int i=0;i<JOINTNUM;i++)
	// {
	// 	ecat_nrmk_drive[i].mode_of_operation_ = ecat_nrmk_drive[i].MODE_CYCLIC_SYNC_TORQUE;
	// }
	if(elmo_count == JOINTNUM)

		return 1;
	else
		return 0;
}

int compute()
{

	return 0;
}
void readEcatData(){
		for(int k=0; k<JOINTNUM; ++k){
			DeviceState[k] = 			ecat_nrmk_drive[k].NRMK_Drive_DeviceState();
			StatusWord[k] = 			ecat_nrmk_drive[k].status_word_;
			ModeOfOperationDisplay[k] = ecat_nrmk_drive[k].mode_of_operation_display_;
			ControlWord[k] = 			ecat_nrmk_drive[k].control_word_;
			ECAT_ActualPos[k] = 				ecat_nrmk_drive[k].position_-ECAT_ActualPos_zero[k];
			ECAT_ActualVel[k] = 				ecat_nrmk_drive[k].velocity_;
			ECAT_ActualTor[k] = 				ecat_nrmk_drive[k].torque_;
		}
		// read FT
		iStatus = ecat_nrmk_indy_tool[0].iStatus_;
		iButton = ecat_nrmk_indy_tool[0].iButton_;
		FT_Raw_Fx = ecat_nrmk_indy_tool[0].FT_Raw_Fx_;
		FT_Raw_Fy = ecat_nrmk_indy_tool[0].FT_Raw_Fy_;
		FT_Raw_Fz = ecat_nrmk_indy_tool[0].FT_Raw_Fz_;
		FT_Raw_Tx = ecat_nrmk_indy_tool[0].FT_Raw_Tx_;
		FT_Raw_Ty = ecat_nrmk_indy_tool[0].FT_Raw_Ty_;
		FT_Raw_Tz = ecat_nrmk_indy_tool[0].FT_Raw_Tz_;
		FT_OverloadStatus = ecat_nrmk_indy_tool[0].FT_OverloadStatus_;
		FT_ErrorFlag = ecat_nrmk_indy_tool[0].FT_ErrorFlag_;
}

void calcTorque_To_EcatTorque(JVec calc_torque, double* toq){
   for(int i=0; i<6; ++i) {
        if(i==0)
            toq[i] = -(calc_torque(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_1*GEAR_RATIO_121*EFFICIENCY)*100.0;
		else if(i==1)
            toq[i] = -(calc_torque(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_2*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==2)
            toq[i] = (calc_torque(i))*(double)(TORQUE_ADC_200)/(double)(TORQUE_CONST_3*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==3)
            toq[i] = -(calc_torque(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_4*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==4)
            toq[i] = -(calc_torque(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_5*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==5)
            toq[i] = (calc_torque(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_6*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else
            return;
    }    
}
void saturationEcatTorque(double *p_toq , int maxtoq, int *p_dir)
{
    double toq_tmp=0;
	for(int i=0; i<JOINTNUM; ++i)
	{
		toq_tmp = p_toq[i];
		if(toq_tmp <= -maxtoq)
		{
			p_toq[i] = p_dir[i]*-maxtoq;
		}
		else if(toq_tmp >= maxtoq)
		{
			p_toq[i] = p_dir[i]*maxtoq;
		}
		else
		{
			p_toq[i] = p_dir[i]*toq_tmp;
		}
	}
	return;
}
void saturationEint(JVec &torque, JVec MAX_TORQUES){
    for(int i =0;i<JOINTNUM;i++){
        if(abs(torque(i))> MAX_TORQUES(i)){
            if(torque(i)<0) torque(i) = MAX_TORQUES(i);
            else torque(i) = -MAX_TORQUES(i);
        }
    }
}
void writeEcatData(double* ECAT_calcTorq){
	for(int j=0; j<JOINTNUM; ++j)
	{
		ECAT_TargetToq[j] = round(ECAT_calcTorq[j]);
		ecat_nrmk_drive[j].writeTorque(ECAT_TargetToq[j]);
	}
}
// RTRArm_task
void RTRArm_run(void *arg)
{
	unsigned int runcount=0;
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;
	//ecatmaster.SyncEcatMaster(rt_timer_read());
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	info.des.q = JVec::Zero();
	info.des.dq = JVec::Zero();
	info.des.ddq = JVec::Zero();
	JVec eint = JVec::Zero();
	JVec e = JVec::Zero();
	JVec q0 = JVec::Zero();
	JVec qT = JVec::Zero();
	//qT<< 0.1,0.1,-1.5708,0.1,-1.5708,0.1;
	qT<< 1.0,-1.0,-1.5707,0.5,-1.0,1.0;
	double Tf = 5;
	int method =5;
	while (run)
	{
		runcount++;
		if (!run){
			break;
		}
		previous = rt_timer_read();
		// [ToDo] Here is an error for PDO mapping

		ecatmaster.TxUpdate();
#if defined(USE_DC_MODE)
		ecatmaster.SyncEcatMaster(rt_timer_read());
#endif
		if(system_ready)
		{
			readEcatData();
			EncToRad(wc,dt);
			if(traj_flag ==0){
				q0 = info.act.q;
				traj_flag =1;
			}
			if(traj_flag == 1){
				if (double_gt <=Tf){
					JointTrajectory(q0, qT, Tf, double_gt , method , info.des.q, info.des.dq, info.des.ddq) ;
				}else if(double_gt > Tf){
					q0 = info.act.q;
					// qT = JVec::Zero();
					traj_flag =2;
				}
				
			}
			if(traj_flag ==2){
				JointTrajectory(q0, qT, Tf, double_gt-Tf , method , info.des.q, info.des.dq, info.des.ddq) ;
			}
			
			compute();
			Robot_Limit();
			JVec clacTorq = mr_indy7.Gravity( info.act.q ); // calcTorque
			//JVec clacTorq = mr_indy7.ComputedTorqueControl( info.act.q , info.act.dq, info.des.q, info.des.dq); // calcTorque
			e = info.des.q-info.act.q;
			// [ToDo] Add anti-windup function
			eint = eint+e*dt;
			// JVec clacTorq = mr_indy7.HinfControl( info.act.q , info.act.dq, info.des.q, info.des.dq,info.des.ddq,eint);
			// mr_indy7.saturationMaxTorque(clacTorq,MAX_TORQUES);

			calcTorque_To_EcatTorque(clacTorq, ECAT_calcTorq);
			saturationEcatTorque(ECAT_calcTorq, 1000, MotorDir);
			writeEcatData(ECAT_calcTorq);
		}
		ecatmaster.RxUpdate();

		if (system_ready)
		{
			//saveLogData();
		}
		else
		{
			double_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
		}
		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();
		double_gt += ((double)(long)(p3 - p1))*1e-9;
		ethercat_time = (long) now - previous;

		if ( isDriveInit() == 1 && (runcount > WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)))
		{
			system_ready=1;	//all drives have been done
			gt+= period;
			if (worst_time<ethercat_time)
				worst_time=ethercat_time;
			if(ethercat_time > (long)cycle_ns)
				++fault_count;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
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
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e8);

	
	while (1)
	{
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
			// rt_printf("Time=%0.3lfs \tFlag : %d\n", double_gt,flag);
			// rt_printf("ethercat_dt= %lius, worst_dt= %lins, fault=%d\n", ethercat_time/1000, worst_time, fault_count);

			// for(int j=0; j<JOINTNUM; ++j){
			// 	rt_printf("ID: %d", j+NUM_FT);
			// 	rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
			// 	// rt_printf("\t ecat_ActPos : %d",ecat_nrmk_drive[j].position_);
			// 	//rt_printf("\t ecat_ActPosZero : %f",ECAT_ActualPos_zero[j]);
			// 	rt_printf("\t ActPos: %lf, ActVel :%lf \n",ActualPos_Rad[j], ActualVel_Rad[j]);
			// 	rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.dq[j],info.des.ddq[j]);
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.dq[j]-info.act.dq[j]);
			// 	rt_printf("\t TarTor: %f, ",				ECAT_calcTorq[j]);
			// 	rt_printf("\t ActTor: %d,\n",			ECAT_ActualTor[j]);
			// }
			// rt_printf("\n");
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

		rt_task_wait_period(NULL); //wait for next cycle
	}
}


void plot_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100 ms)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7);	// period = 10 (msec)


	while (1)
	{

		rt_task_wait_period(NULL);
	}
}

/****************************************************************************/
void signal_handler(int signum)
{
	rt_task_delete(&plot_task);
	rt_task_delete(&RTRArm_task);
	rt_task_delete(&print_task);
	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
    ecatmaster.deactivate();
    exit(1);
}


/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);

	int portNum=1; //COM as default
	char portName[100];
	unsigned int baud = 9600;
	switch (portNum)
	{
		case 0: strcpy(portName, RS485PORT); break;
		case 1: strcpy(portName, COM1); break;
		case 2: strcpy(portName, COM2); break;
		default: strcpy(portName, COM1); break;
	}
	if ((portNum>0) && (baud>RS232_BAUD_LIMIT))
		baud=RS232_BAUD_LIMIT;
	if (baud<1200)
		baud=1200;
	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns = 1000000; // nanosecond -> 1kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	mr_indy7=MR_Indy7();
	mr_indy7.MRSetup();
	MAX_TORQUES<<MAX_TORQUE_1,MAX_TORQUE_2,MAX_TORQUE_3,MAX_TORQUE_4,MAX_TORQUE_5,MAX_TORQUE_6;


	for(int j=0; j<JOINTNUM; ++j)

	{
		ecatmaster.addSlaveNRMK_Drive(0, j+1, &ecat_nrmk_drive[j]);
		ecat_nrmk_drive[j].mode_of_operation_ = ecat_nrmk_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
	for(int i=0; i<NUM_FT; ++i)
		ecatmaster.addSlaveNRMK_Indy_Tool(0, i+JOINTNUM+1, &ecat_nrmk_indy_tool[i]);

#if defined(USE_DC_MODE)
	ecatmaster.activateWithDC(0, cycle_ns);  //a first arg DC location of MotorDriver?
	rtsercan_fd=SERCAN_open();
#else
	ecatmaster.activate();
#endif
	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
	

	// RTRArm_task: create and start
	printf("Now running rt task ...\n");
	rt_printf(" sercan_dev_open = %d\n", rtsercan_fd);

	rt_task_create(&RTRArm_task, "RTRArm_task", 0, 99, 0);
	rt_task_start(&RTRArm_task, &RTRArm_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	
	// plotting: data socket comm
	//rt_task_create(&plot_task, "plotting", 0, 80, 0);
	//rt_task_start(&plot_task, &plot_run, NULL);

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



