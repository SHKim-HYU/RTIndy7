#include "../global_vars.h"

const int 	 zeroPos_arm[NRMK_DRIVE_NUM] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6,
                                  ZERO_POS_7,ZERO_POS_8,ZERO_POS_9,ZERO_POS_10,ZERO_POS_11,ZERO_POS_12};
const int 	 gearRatio_arm[NRMK_DRIVE_NUM] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC_arm[NRMK_DRIVE_NUM] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK_arm[NRMK_DRIVE_NUM] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1};
const int 	 dirTau_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1};
const double qdotLimit[NRMK_DRIVE_NUM] = {2*PI, 2*PI, 2*PI, 2*PI, 2*PI, 3*PI,2*PI, 2*PI, 2*PI, 2*PI, 2*PI, 3*PI};

double force_divider =50.0;
double torque_divider = 2000.0;
double ft_offset[6] = {0.0,};

bool isSlaveInit()
{
    // Indy Drive Servo on
    for(int i=0; i<NRMK_DRIVE_NUM; i++)
    {
        if(!ecat_drive[i].isSystemReady())
            return false;
    }


    return true;
}

int initAxes()
{
    // Arm Drive Init Axes
	for (int i = 0; i < NRMK_DRIVE_NUM; i++)
	{	
		Axis[i].setGearRatio(gearRatio_arm[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ENC_CORE);
		Axis[i].setTauADC(TauADC_arm[i]);
		Axis[i].setTauK(TauK_arm[i]);
		Axis[i].setZeroPos(zeroPos_arm[i]);
		Axis[i].setVelLimits(qdotLimit[i], -qdotLimit[i]);
		Axis[i].setDirQ(dirQ_arm[i]);
		Axis[i].setDirTau(dirTau_arm[i]);
		Axis[i].setConversionConstants();
		Axis[i].setTrajPeriod(period);
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
	}

	return 1;
}

void readData()
{
	ecat_master.TxUpdate();
    
    for(int i=0; i<NRMK_DRIVE_NUM;i++)
	{
		Axis[i].setCurrentPosInCnt(ecat_drive[i].position_);
		Axis[i].setCurrentVelInCnt(ecat_drive[i].velocity_);
		Axis[i].setCurrentTorInCnt(ecat_drive[i].torque_);
		
		Axis[i].setCurrentTime(gt);

		info.act.q(i) = Axis[i].getCurrPosInRad();
		info.act.q_dot(i) = Axis[i].getCurrVelInRad();
		info.act.tau(i) = Axis[i].getCurrTorInNm();
		
		if(!system_ready)
		{
			Axis[i].setTarPosInRad(info.act.q(i));
			Axis[i].setDesPosInRad(info.act.q(i));
			info.nom.q(i) = info.act.q(i);
			info.nom.q_dot(i) = info.act.q_dot(i);
			info.nom.tau(i) = info.act.tau(i);
		}

	}

	for(int i=0; i<NRMK_TOOL_NUM; i++)
	{
		// Update RFT data
		if (ecat_tool[i].FT_Raw_F[0] ==0 && ecat_tool[i].FT_Raw_F[1] ==0 && ecat_tool[i].FT_Raw_F[2] ==0)
		{
			F_tmp = Twist::Zero();
		}
		else{
			for(int j=0; j<3; j++)
			{
				F_tmp(j) = (double)ecat_tool[i].FT_Raw_F[j] / force_divider - ft_offset[j];
				F_tmp(j+3) = (double)ecat_tool[i].FT_Raw_T[j] / torque_divider - ft_offset[j+3];
			}
		}
	}
}

void writeData()
{
    for(int i=0;i<NRMK_DRIVE_NUM;i++){

        Axis[i].setDesTorInNm(info.des.tau(i));
            
        INT16 temp = Axis[i].getDesTorInCnt();

        ecat_drive[i].writeTorque(temp);

        ecat_master.RxUpdate();
	}
    ecat_master.SyncEcatMaster(rt_timer_read());
}

void rt_motor_run(void *arg)
{
    RTIME beginCycle, endCycle;
	RTIME beginCyclebuf;

	beginCyclebuf = 0;
   
	memset(&info, 0, sizeof(ROBOT_INFO));

	int ft_init_cnt = 0;

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6d::Zero();
	info.des.F_CB = Vector6d::Zero();
	info.act.tau_aux = JVec::Zero();

	int DRIVE_NUM_LIST[NRMK_DRIVE_NUM] = {2,3,4,5,6,7,10,11,12,13,14,15};
	int TOOLS_NUM_LIST[NRMK_TOOL_NUM] = {1,8,9,16};
    for(int j=0; j<NRMK_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveNRMKdrive(0,DRIVE_NUM_LIST[j], &ecat_drive[j]);
		ecat_drive[j].mode_of_operation_ = ecat_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
    for(int j=0; j<NRMK_TOOL_NUM; ++j)
	{
		ecat_master.addSlaveNRMKtool(0, TOOLS_NUM_LIST[j], &ecat_tool[j]);
	}

    initAxes();


    ecat_master.activateWithDC(0, cycle_ns);
    
    for (int i=0; i<NRMK_DRIVE_NUM; i++)
        ecat_drive[i].setServoOn();
    
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        beginCycle = rt_timer_read();
        // Read Joints Data
        readData();
        if(system_ready)
        {
                    
        }
		else
		{
			
		}
        // Write Joint Data
        writeData();
        
        endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		periodLoop = (unsigned long) beginCycle - beginCyclebuf;

        if(isSlaveInit())
		{
			if(ft_init_cnt==0)
			{
				// Stop FT Sensor
				UINT32 FTConfigParam=FT_STOP_DEVICE;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Start
				UINT32 FTConfigParam=FT_START_DEVICE;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Set bias
				UINT32 FTConfigParam=FT_SET_BIAS;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==3)
			{
				// Set Filter 10Hz
				UINT32 FTConfigParam=FT_SET_FILTER_500;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else
				system_ready=true;;	//all drives have been done
		} 
            
        
		if(system_ready)
		{
			gt+= period;
			if (periodCycle > cycle_ns) overruns++;
			if (periodLoop > worstLoop) worstLoop = periodLoop;
		}

        beginCyclebuf = beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
    }
}


