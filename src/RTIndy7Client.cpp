#include "RTIndy7Client.h"

ROBOT_INFO info;

CS_Indy7 cs_indy7;
CS_Indy7 cs_nom_indy7;

RT_TASK safety_task;
RT_TASK motor_task;
RT_TASK bullet_task;
RT_TASK print_task;

using namespace std;
using namespace lr;


static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

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
	des_int = JVec::Zero();
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

/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
#ifdef __RP__		
			info.q_target(0)=0.0; info.q_target(1)=0.0; info.q_target(2)=0.0;
			info.q_target(3)=-1.5709; info.q_target(4)=0.0; info.q_target(5)=-1.5709;
			info.q_target(6)=0.0;
#else
			info.q_target(0)=0.0; info.q_target(1)=0.0; info.q_target(2)=-1.5709;
			info.q_target(3)=0.0; info.q_target(4)=-1.5709; info.q_target(5)=0.0;
#endif	
			// info.q_target(0)=0.090390; info.q_target(1)=-0.402861; info.q_target(2)=-0.613726;
			// info.q_target(3)=0.003550; info.q_target(4)=-2.134188; info.q_target(5)=0.082176;
			
			traj_time = 3;
			modeControl =1;
	    	motion++;
			motion=2;
	        break;

		// case 2:
		// 	info.q_target(0)=0.0; info.q_target(1)=0.707; info.q_target(2)=-1.5709;
		// 	info.q_target(3)=0.0; info.q_target(4)=-0.707; info.q_target(5)=0.0;
		// 	traj_time = 3;
		// 	modeControl =1;
	    // 	motion++;
		// 	// motion=1;
	    //     break;

		// case 3:
			
		// 	info.q_target(0)=0.0; info.q_target(1)=0.0; info.q_target(2)=0.0;
		// 	info.q_target(3)=0.0; info.q_target(4)=0.0; info.q_target(5)=0.0;
	    // 	traj_time = 3;
		// 	modeControl =1;
	    // 	motion=1;
	    //     break;

	    case 2:
			cs_nom_indy7.resetTaskAdmittance();
	    	info.T_init = info.nom.T;
			info.T_target << 0.0, -0, 1.0, 0.672989,
							0.0, 1.0, 0, -0.177496,
							-1.0, 0.0, 0.0, 0.47077,
							0, 0, 0, 1;
			info.T_target << 0.0, -0, 1.0, 0.5975,
							0.0, 1.0, 0, -0.19,
							-1.0, 0.0, 0.0, 0.649504,
							0, 0, 0, 1;
			info.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 							1.40484e-21, 1, 1.40483e-21, -0.1865, 
 							-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 							0, 0, 0, 1;
			traj_time = 10;	
			modeControl=3;
			motioncnt = 0;
			// motion++;
			motion=7;
			break;
		case 3:
	    	info.T_init = info.T_target;
			info.T_target << 0.0, -0, 1.0, 0.672989,
							0.0, 1.0, 0, -0.277496,
							-1.0, 0.0, 0.0, 0.57077,
							0, 0, 0, 1;
			// info.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 			// 				1.40484e-21, 1, 1.40483e-21, -0.1865, 
 			// 				-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 			// 				0, 0, 0, 1;
			traj_time = 10;	
			modeControl=3;
			motioncnt = 0;
			// motion=2;
			motion++;
			break;

		case 4: 
	    	info.T_init = info.T_target;
			info.T_target << 0.0, -0, 1.0, 0.672989,
							0.0, 1.0, 0, -0.277496,
							-1.0, 0.0, 0.0, 0.37077,
							0, 0, 0, 1;
			// info.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 			// 				1.40484e-21, 1, 1.40483e-21, -0.1865, 
 			// 				-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 			// 				0, 0, 0, 1;
			traj_time = 10;	
			modeControl=3;
			motioncnt = 0;
			motion=3;
			// motion++;
			break;
		
		case 5:
			info.T_init = info.T_target;
			info.T_target << 0.0, -0, 1.0, 0.672989,
							0.0, 1.0, 0, -0.077496,
							-1.0, 0.0, 0.0, 0.37077,
							0, 0, 0, 1;
			// info.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 			// 				1.40484e-21, 1, 1.40483e-21, -0.1865, 
 			// 				-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 			// 				0, 0, 0, 1;
			traj_time = 10;	
			modeControl=3;
			motioncnt = 0;
			// motion=2;
			motion++;
			break;

		case 6:
			info.T_init = info.T_target;
			info.T_target << 0.0, -0, 1.0, 0.672989,
							0.0, 1.0, 0, -0.077496,
							-1.0, 0.0, 0.0, 0.5077,
							0, 0, 0, 1;
			// info.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 			// 				1.40484e-21, 1, 1.40483e-21, -0.1865, 
 			// 				-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 			// 				0, 0, 0, 1;
			traj_time = 10;	
			modeControl=3;
			motioncnt = 0;
			motion=3;
			// motion++;
			break;
		case 7:

			break;

		default:
			info.q_target(0)=info.act.q(0); info.q_target(1)=info.act.q(1); info.q_target(2)=info.act.q(2);
			info.q_target(3)=info.act.q(3); info.q_target(4)=info.act.q(4); info.q_target(5)=info.act.q(5);
#ifdef __RP__
			info.q_target(6)=info.act.q(6);
#endif

	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<ROBOT_DOF;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();
		if(modeControl==1)
		{
			info.des.q(i)=Axis[i].getDesPosInRad();
			info.des.q_dot(i)=Axis[i].getDesVelInRad();
			info.des.q_ddot(i)=Axis[i].getDesAccInRad();
		}
	}

	// 3. Lie Screw Trajectory --> [ToDo] It will be replaced to liescrewtraj function
	if(!motioncnt) 
	{			
		gt_offset = gt;
		motioncnt++;
	}
	LieScrewTrajectory(info.T_init, info.T_target, info.V_init, info.V_target, info.V_init, info.V_target, traj_time, gt-gt_offset, info.des.T, info.des.x_dot, info.des.x_ddot);
}

void compute()
{
	// Update Indy7
	cs_indy7.updateRobot(info.act.q, info.act.q_dot, JVec::Zero());
	// Update nominal
	cs_nom_indy7.updateRobot(info.nom.q , info.nom.q_dot, info.nom.q_ddot);
	
	info.act.T = cs_indy7.getFK();
	info.act.R = cs_indy7.getRMat();
	info.nom.T = cs_nom_indy7.getFK();
	info.nom.R = cs_nom_indy7.getRMat();

	Jacobian J_b = cs_indy7.getJ_b();
	Jacobian dJ_b = cs_indy7.getJdot_b();
	info.act.x_dot = cs_indy7.getBodyTwist();
	info.act.x_ddot = dJ_b*info.nom.q_dot + J_b*info.nom.q_ddot;
	
	Jacobian J_b_nom = cs_nom_indy7.getJ_b();
	Jacobian dJ_b_nom = cs_nom_indy7.getJdot_b();
	info.nom.x_dot = cs_nom_indy7.getBodyTwist();
	info.nom.x_ddot = dJ_b_nom*info.nom.q_dot + J_b_nom*info.nom.q_ddot;
		
	Twist F_mometum = cs_indy7.computeF_Tool(info.act.x_dot, info.act.x_ddot);
	
	// info.act.F = F_tmp-F_mometum;
	info.act.F = cs_indy7.computeF_Threshold(F_tmp);
	info.act.tau_ext = J_b.transpose()*(info.act.F);

	info.act.tau_fric = cs_indy7.FrictionEstimation(info.act.q_dot);
	// info.act.tau_fric = JVec::Zero();

	manipulability = cs_indy7.getManipulability();
}

void control()
{	
	if(modeControl==0)
	{
		// info.des.tau = cs_indy7.computeG(info.act.q);
		JVec G_damp = {0.5, 0.5, 0.5, 1.0, 1.0, 0.05};
		info.nom.tau = cs_nom_indy7.computeG(info.nom.q) - G_damp.cwiseProduct(info.nom.q_dot) + info.act.tau_ext;
		// [Simulation]
		cs_nom_indy7.computeRK45(info.nom.q, info.nom.q_dot, info.nom.tau, info.nom.q, info.nom.q_dot, info.nom.q_ddot);
			
		// [NRIC]
		info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);
		// info.des.tau = info.nom.tau - info.act.tau_aux + info.act.tau_ext;
		info.des.tau = info.nom.tau - info.act.tau_aux;
	}
	else if(modeControl==1)
	{
		// [Joint Space Nominal Controller]
		// info.nom.tau = cs_nom_indy7.ComputedTorqueControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot);
    	// info.nom.tau = cs_nom_indy7.ComputedTorqueControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot, info.act.tau_ext);
		info.nom.tau = cs_nom_indy7.PassivityInverseDynamicControl(info.nom.q, info.nom.q_dot, info.des.q, info.des.q_dot, info.des.q_ddot);

		// [Simulation]
		cs_nom_indy7.computeRK45(info.nom.q, info.nom.q_dot, info.nom.tau, info.nom.q, info.nom.q_dot, info.nom.q_ddot);
			
		// [NRIC]
		info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);
		// info.des.tau = info.nom.tau - info.act.tau_aux + info.act.tau_ext;
		info.des.tau = info.nom.tau - info.act.tau_aux;
	}
	else if(modeControl==2)
	{
		// [Task Space Nominal Controller]
		if(gt>13)
		{
			info.des.F << 0, 0, 0, 0, 0, 0;
			// info.act.F << 0, 0, 0, 0, 0, 0;
			// // Set bias
			// UINT32 FTConfigParam=FT_SET_BIAS;
			// ecat_tool[0].writeFTconfig(FTConfigParam);			
			// ecat_master.RxUpdate();
		}
		
		// info.nom.tau = cs_nom_indy7.TaskInverseDynamicsControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot);
		// info.nom.tau = cs_nom_indy7.TaskJTbasedInverseDynamicsControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot);
		// info.nom.tau = cs_nom_indy7.TaskPassivityInverseDynamicsControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot);
		// info.nom.tau = cs_nom_indy7.TaskImpedanceControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot, Twist::Zero(), info.act.F);
		info.nom.tau = cs_nom_indy7.TaskImpedanceControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot, info.des.F, info.act.F);
		// info.nom.tau = cs_nom_indy7.TaskJTbasedImpedanceControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot, Twist::Zero(), info.act.F);
		// info.nom.tau = cs_nom_indy7.TaskJTbasedImpedanceControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot, info.des.F, info.act.F);
		// info.nom.tau = cs_nom_indy7.TaskPassivityImpedanceControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot, Twist::Zero(), info.act.F);
		// info.nom.tau = cs_nom_indy7.TaskComplianceImpedanceControl(info.des.T, info.des.x_dot, info.des.x_ddot, Twist::Zero(), info.act.F);
		cs_nom_indy7.computeMK45(info.des.T, info.des.x_dot, info.des.x_ddot, info.sim.T, info.sim.x_dot, info.sim.x_ddot, info.des.F, info.act.F);
		// info.sim.T = info.nom.T;
		// [Simulation]
		cs_nom_indy7.computeRK45(info.nom.q, info.nom.q_dot, info.nom.tau, info.nom.q, info.nom.q_dot, info.nom.q_ddot);
			
		// [NRIC]
		info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);
		// info.des.tau = info.nom.tau - info.act.tau_aux + info.act.tau_ext;
		info.des.tau = info.nom.tau - info.act.tau_aux;
		// info.des.tau = info.act.tau_aux;
	}
	else if(modeControl==3)
	{
		// [Task Space Nominal Controller]
		if(gt>13)
		{
			info.des.F << 0, 0, 0, 0, 0, 0;
			
			// // Set bias
			// UINT32 FTConfigParam=FT_SET_BIAS;
			// ecat_tool[0].writeFTconfig(FTConfigParam);			
			// ecat_master.RxUpdate();
		}

		// cs_nom_indy7.TaskAdmittance(info.des.T, info.des.x_dot, info.des.x_ddot, info.sim.T, info.sim.x_dot, info.sim.x_ddot, info.des.F, info.act.F);
		// cs_nom_indy7.computeMK45(info.des.T, info.des.x_dot, info.des.x_ddot, info.sim.T, info.sim.x_dot, info.sim.x_ddot, info.des.F, info.act.F);
		cs_nom_indy7.computeDecoupledMK45(info.des.T, info.des.x_dot, info.des.x_ddot, info.sim.T, info.sim.x_dot, info.sim.x_ddot, info.des.F, info.act.F);
		info.nom.q_ddot = cs_nom_indy7.TaskStablePD(info.sim.T, info.sim.x_dot, info.sim.x_ddot);

		// info.nom.q_ddot = cs_nom_indy7.TaskStablePD(info.des.T, info.des.x_dot, info.des.x_ddot);
		// info.nom.q_ddot = cs_nom_indy7.TaskStablePDImpedance(info.des.T, info.des.x_dot, info.des.x_ddot, info.des.F, info.act.F);
		// info.sim.T = info.nom.T;

		info.nom.q_dot += info.nom.q_ddot*period;
		info.nom.q += info.nom.q_dot*period;

		// [NRIC]
		info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);
		// info.des.tau = info.nom.tau - info.act.tau_aux + info.act.tau_ext;
		info.des.tau = cs_nom_indy7.getG() - info.act.tau_aux;
		// info.des.tau = info.act.tau_aux;
	}
	else if(modeControl==4)
	{
		// [Task Space Nominal Controller]
		info.nom.tau = cs_nom_indy7.TaskInverseDynamicsControl(info.nom.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot)+ info.act.tau_ext - tau_bd;
		
		// [Simulation]
		cs_nom_indy7.computeRK45(info.nom.q, info.nom.q_dot, info.nom.tau, info.nom.q, info.nom.q_dot, info.nom.q_ddot);
			
		// [NRIC]
		info.des.tau = cs_indy7.TaskInverseDynamicsControl(info.act.q_dot, info.des.T, info.des.x_dot, info.des.x_ddot);
		info.act.tau_aux = cs_indy7.NRIC(info.act.q, info.act.q_dot, info.nom.q, info.nom.q_dot);

		alpha = cs_indy7.computeAlpha(info.nom.q_dot-info.act.q_dot, info.act.tau_aux, info.act.tau_ext);
    	tau_bd = alpha*info.act.tau_aux;
    	
		info.des.tau += (1-alpha)*info.act.tau_aux;
		
	}

	// [Gravity Compensator]
	// info.des.tau = cs_indy7.computeG(info.act.q);
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

void motor_run(void *arg)
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
	tau_bd = JVec::Zero();
	A_ = Matrix6d::Zero(); D_ = Matrix6d::Zero(); K_ = Matrix6d::Zero();

	// Real
#ifdef __RP__
	NRIC_Kp << 20.0, 25.0, 10.0, 10.0, 3.0, 3.0, 1.5;
	NRIC_Ki << 5.0, 5.5, 2.5, 2.5, 0.8, 0.8, 0.6;
	// NRIC_Kp << 100.0, 100.0, 100.0, 100, 100, 100;
	// NRIC_Ki << 20.0, 20.0, 20.0, 20.0, 20., 20.0;
	// NRIC_K << 80.0, 80.0, 50.0, 15.0, 15.0, 15.0;
	// NRIC_gamma << 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
	NRIC_gamma << 80.0, 80.0, 50.0, 50.0, 15.0, 15.0, 15.0;
	NRIC_K << 550.0, 600.0, 450.0, 450.0, 250.0, 250.0, 175.0;
#else
	NRIC_Kp << 20.0, 25.0, 10.0, 3.0, 3.0, 1.5;
	NRIC_Ki << 5.0, 5.5, 2.5, 0.8, 0.8, 0.6;
	// NRIC_Kp << 100.0, 100.0, 100.0, 100, 100, 100;
	// NRIC_Ki << 20.0, 20.0, 20.0, 20.0, 20., 20.0;
	// NRIC_K << 80.0, 80.0, 50.0, 15.0, 15.0, 15.0;
	// NRIC_gamma << 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
	NRIC_gamma << 80.0, 80.0, 50.0, 15.0, 15.0, 15.0;
	NRIC_K << 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
#endif
	// NRIC_Kp << 10.0, 10.0, 10.0, 10, 20, 20;
	// NRIC_Ki << 8.0, 8.0, 8.0, 8.0, 8.0, 8.0;
	// NRIC_gamma << 80.0, 80.0, 50.0, 15.0, 15.0, 15.0;
	// NRIC_K << 400.0, 400.0, 400.0, 400.0, 600.0, 600.0;
	cs_indy7.setNRICgain(NRIC_Kp, NRIC_Ki, NRIC_K,  NRIC_gamma);
	
	// nominal
	// for IDC
#ifdef __RP__
	Kp_n << 50.0, 50.0, 30.0, 30.0, 20.0, 20.0, 20.0;
	Kd_n << 5.0, 5.0, 3.0, 3.0, 2.0, 2.0, 2.0;
	K_n << 1,1,0.5,0.5,0.3,0.2,0.2;
#else
	Kp_n << 50.0, 50.0, 30.0, 20.0, 20.0, 20.0;
	Kd_n << 5.0, 5.0, 3.0, 2.0, 2.0, 2.0;
	K_n << 1,1,0.5,0.3,0.2,0.2;
#endif
	cs_nom_indy7.setPIDgain(Kp_n, Kd_n, K_n);

	// Task
	// for IDC
	// Task_Kp << 8000, 8000, 8000, 1000, 1000, 1000;
	// Task_Kv << 1000, 1000, 1000, 500, 500, 500;
	// for pIDC
	// Task_Kp << 200000, 200000, 200000, 200000, 200000, 200000;
	// Task_Kv << 1000, 1000, 1000, 1000, 1000, 1000;
	// for SPD
	Task_Kp << 1, 1, 1, 0.033, 0.033, 0.033;
	// Task_Kp << 1, 1, 1, 1, 1, 1;
	Task_Kp = Task_Kp * 1e6;
	Task_Kv = Task_Kp*period*30.0;
	// Task_Kp << 4000, 100, 4000, 4000, 4000, 4000;
	// Task_Kv << 400, 20, 400, 400, 400, 400;
#ifdef __RP__	
	Task_K << 1,1,0.5,0.3,0.2,0.2;

	cs_nom_indy7.setTaskgain(Task_Kp, Task_Kv, Task_K);

	// for Impedance model
	// A_.diagonal() << 400, 400, 400, 10, 10, 10;
	A_.diagonal() << 4, 4, 4, 0.4, 0.4, 0.04;
    // K_.diagonal() << 3000,3000, 3000, 300, 300, 0.1;
	K_.diagonal() << 10,10, 30000, 3000, 3000, 3000;
	for(int i=0;i<6;i++)
		D_(i,i) = 2*1*sqrt(A_(i,i)*K_(i,i));
	
	// D_(0,0) = 0.1; 
	// D_(1,1) = 0.1; 
	// D_(2,2) = 0.1; 
	// D_(3,3) = 0.01; 
	// D_(4,4) = 0.2; 
	// D_(5,5) = 0.2; 
	cs_nom_indy7.setTaskImpedancegain(A_,D_,K_);
	

    for(int j=0; j<NRMK_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveNRMKdrive(0, j+OFFSET_NUM, &ecat_drive[j]);
		ecat_drive[j].mode_of_operation_ = ecat_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
    for(int j=0; j<NRMK_TOOL_NUM; ++j)
	{
		ecat_master.addSlaveNRMKtool(0, j+OFFSET_NUM+NRMK_DRIVE_NUM, &ecat_tool[j]);
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
            // Trajectory Generation
            trajectory_generation();
            
            // Compute KDL
            compute();	

            
            // Controller
            control();
                    
        }
		else
		{
			info.des.tau = cs_indy7.computeG(info.act.q);
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

// Safety task
void safety_run(void *arg)
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

void print_run(void *arg)
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
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
	// /*
	// Joint data log
	string filename1 = "joint_log.csv";
	ifstream checkFile1(filename1);

	if (checkFile1.is_open())
	{
		checkFile1.close();
		remove(filename1.c_str());
	}
	ofstream newFile1(filename1);
	if(newFile1.is_open())
	{
		newFile1<<"Time, q_r1, q_r2, q_r3, q_r4, q_r5, q_r6, dq_r1, dq_r2, dq_r3, dq_r4, dq_r5, dq_r6, q_n1, q_n2, q_n3, q_n4, q_n5, q_n6, dq_n1, dq_n2, dq_n3, dq_n4, dq_n5, dq_n6\n";
		newFile1.close();
	}
	ofstream csvFile1(filename1, ios_base::app);

	// Task data log
	string filename2 = "task_log.csv";
	ifstream checkFile2(filename2);

	if (checkFile2.is_open())
	{
		checkFile2.close();
		remove(filename2.c_str());
	}

	ofstream newFile2(filename2);
	if(newFile2.is_open())
	{
		newFile2<<"Time, xr, yr, zr, q0r, q1r, q2r, q3r, xn, yn, zn, q0n, q1n, q2n, q3n, xd, yd, zd, q0d, q1d, q2d, q3d\n";
		newFile2.close();
	}
	ofstream csvFile2(filename2, ios_base::app);

	// FT data log
	string filename3 = "ft_log.csv";
	ifstream checkFile3(filename3);

	if (checkFile3.is_open())
	{
		checkFile3.close();
		remove(filename3.c_str());
	}

	ofstream newFile3(filename3);
	if(newFile3.is_open())
	{
		newFile3<<"Time, Fx, Fy, Fz, Tx, Ty, Yz, Fx_tmp, Fy_tmp, Fz_tmp, Tx_tmp, Ty_tmp, Tz_tmp\n";
		newFile3.close();
	}
	ofstream csvFile3(filename3, ios_base::app);

	// Torque data log
	string filename4 = "torque_log.csv";
	ifstream checkFile4(filename4);

	if (checkFile4.is_open())
	{
		checkFile4.close();
		remove(filename4.c_str());
	}

	ofstream newFile4(filename4);
	if(newFile4.is_open())
	{
		newFile4<<"Time, tau_r1, tau_r2, tau_r3, tau_r4, tau_r5, tau_r6, tau_n1, tau_n2, tau_n3, tau_n4, tau_n5, tau_n6,\n";
		newFile4.close();
	}
	ofstream csvFile4(filename4, ios_base::app);
	// */

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
			rt_printf("modeControl: %d\n",modeControl);
			// /*
            rt_printf("Arm Data\n");
			for(int j=0; j<ROBOT_DOF; ++j){
				rt_printf("ID: %d", j);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf, NomAcc :%lf\n",info.nom.q(j), info.nom.q_dot(j), info.nom.q_ddot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j), info.act.tau_ext(j));
			}
			rt_printf("readFT: %lf, %lf, %lf, %lf, %lf, %lf\n", F_tmp(0),F_tmp(1),F_tmp(2),F_tmp(3),F_tmp(4),F_tmp(5));
			rt_printf("resFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			rt_printf("manipulability: %lf\n",manipulability);
			rt_printf("Tdes: \t%lf, %lf, %lf, %lf\n", info.des.T(0,0), info.des.T(0,1), info.des.T(0,2), info.des.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(1,0), info.des.T(1,1), info.des.T(1,2), info.des.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(2,0), info.des.T(2,1), info.des.T(2,2), info.des.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.des.T(3,0), info.des.T(3,1), info.des.T(3,2), info.des.T(3,3));
			rt_printf("Tadm: \t%lf, %lf, %lf, %lf\n", info.sim.T(0,0), info.sim.T(0,1), info.sim.T(0,2), info.sim.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.sim.T(1,0), info.sim.T(1,1), info.sim.T(1,2), info.sim.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.sim.T(2,0), info.sim.T(2,1), info.sim.T(2,2), info.sim.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.sim.T(3,0), info.sim.T(3,1), info.sim.T(3,2), info.sim.T(3,3));
			rt_printf("Tnom: \t%lf, %lf, %lf, %lf\n", info.nom.T(0,0), info.nom.T(0,1), info.nom.T(0,2), info.nom.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(1,0), info.nom.T(1,1), info.nom.T(1,2), info.nom.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(2,0), info.nom.T(2,1), info.nom.T(2,2), info.nom.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.nom.T(3,0), info.nom.T(3,1), info.nom.T(3,2), info.nom.T(3,3));
			rt_printf("Tact: \t%lf, %lf, %lf, %lf\n", info.act.T(0,0), info.act.T(0,1), info.act.T(0,2), info.act.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(1,0), info.act.T(1,1), info.act.T(1,2), info.act.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(2,0), info.act.T(2,1), info.act.T(2,2), info.act.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info.act.T(3,0), info.act.T(3,1), info.act.T(3,2), info.act.T(3,3));
			rt_printf("\n");
			rt_printf("\n");
			
			// /*
			if(csvFile1.is_open())
			{
				csvFile1<<gt<<", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile1<<info.act.q(i) << ", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile1<<info.act.q_dot(i) << ", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile1<<info.nom.q(i) << ", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile1<<info.nom.q_dot(i) << ", ";
				csvFile1<<"\n";
			}
			if(csvFile2.is_open())
			{
				Quaterniond quat_r(info.act.R);
				Quaterniond quat_n(info.nom.R);
				Quaterniond quat_d(info.des.T.block<3,3>(0,0));
				
				csvFile2<<gt<<", ";
				for (int i = 0; i < 3; ++i) csvFile2<<info.act.T(i,3) << ", "; // xr,yr,zr
				for (int i = 0; i < 4; ++i) csvFile2<<quat_r.vec()(i) << ", ";
				for (int i = 0; i < 3; ++i) csvFile2<<info.nom.T(i,3) << ", "; // xn, yn, zn
				for (int i = 0; i < 4; ++i) csvFile2<<quat_n.vec()(i) << ", ";
				for (int i = 0; i < 3; ++i) csvFile2<<info.des.T(i,3) << ", "; // xd,yd,zd
				for (int i = 0; i < 4; ++i) csvFile2<<quat_n.vec()(i) << ", ";
				csvFile2<<"\n";
			}
			if(csvFile3.is_open())
			{
				csvFile3<<gt<<", ";
				for (int i = 0; i < 6; ++i) csvFile3<<info.act.F(i) << ", "; // Fx_tmp, Fy_tmp, Fz_tmp
				for (int i = 0; i < 6; ++i) csvFile3<<F_tmp(i) << ", "; // Fx_tmp, Fy_tmp, Fz_tmp
				csvFile3<<"\n";
			}
			if(csvFile4.is_open())
			{
				csvFile4<<gt<<", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile4<<info.act.tau(i) << ", ";
				for (int i = 0; i < ROBOT_DOF; ++i) csvFile4<<info.nom.tau(i) << ", ";
				csvFile4<<"\n";
			}
			// */
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
	// /*
	csvFile1.close();
	csvFile2.close();
	csvFile3.close();
	csvFile4.close();
	// */
}

// Bullet task
void bullet_run(void *arg)
{
	struct sockaddr_ipc addr_nom, addr_act, addr_cube;
	uint addr_nom_len = sizeof(addr_nom);
	uint addr_act_len = sizeof(addr_act);
	uint addr_cube_len = sizeof(addr_cube);
	int socket_nom, socket_act, socket_cube;
	int ret_nom, ret_act, ret_cube;
	struct timespec ts;
	size_t poolsz;
	size_t BUFLEN = sizeof(packet::JointState);
	size_t BUFLEN_CUBE = sizeof(packet::Pose);

	struct packet::JointState *bullet_nom_msg = (packet::JointState *)malloc(BUFLEN);
	struct packet::JointState *bullet_act_msg = (packet::JointState *)malloc(BUFLEN);
	struct packet::Pose *bullet_cube_msg = (packet::Pose *)malloc(BUFLEN_CUBE);

    rt_task_set_periodic(NULL, TM_NOW, 100*cycle_ns); // 100ms

	socket_nom = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_nom < 0) {
		perror("socket_nom");
		exit(EXIT_FAILURE);
	}
	socket_act = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_act < 0) {
		perror("socket_act");
		exit(EXIT_FAILURE);
	}
	socket_cube = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_cube < 0) {
		perror("socket_cube");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	if( __cobalt_setsockopt(socket_nom, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_nom, 0, sizeof(addr_nom));
	addr_nom.sipc_family = AF_RTIPC;
	addr_nom.sipc_port = XDDP_PORT_SIM;

	if( __cobalt_setsockopt(socket_act, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_act, 0, sizeof(addr_act));
	addr_act.sipc_family = AF_RTIPC;
	addr_act.sipc_port = XDDP_PORT_ACT;

	if( __cobalt_setsockopt(socket_cube, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_cube, 0, sizeof(addr_cube));
	addr_cube.sipc_family = AF_RTIPC;
	addr_cube.sipc_port = XDDP_PORT_ODOM;

	if(__cobalt_bind(socket_nom, (struct sockaddr *)&addr_nom, sizeof(addr_nom)) == -1)
		fail("bind");
	if(__cobalt_bind(socket_act, (struct sockaddr *)&addr_act, sizeof(addr_act)) == -1)
		fail("bind");
	if(__cobalt_bind(socket_cube, (struct sockaddr *)&addr_cube, sizeof(addr_cube)) == -1)
		fail("bind");

    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle
		if(system_ready)
		{
			Quaterniond quaternion(info.sim.T.block<3,3>(0,0));		
			for(int i=0; i<ROBOT_DOF; i++)
			{
				bullet_nom_msg->position[i] = info.nom.q(i);
				bullet_act_msg->position[i] = info.act.q(i);
			}

			bullet_cube_msg->orientation.x = quaternion.x();
			bullet_cube_msg->orientation.y = quaternion.y();
			bullet_cube_msg->orientation.z = quaternion.z();
			bullet_cube_msg->orientation.w = quaternion.w();
			bullet_cube_msg->position.x = info.sim.T(0,3);
			bullet_cube_msg->position.y = info.sim.T(1,3);
			bullet_cube_msg->position.z = info.sim.T(2,3);

			ret_nom = __cobalt_sendto(socket_nom, bullet_nom_msg, BUFLEN, 0, (struct sockaddr *) &addr_nom, addr_nom_len);
			ret_act = __cobalt_sendto(socket_act, bullet_act_msg, BUFLEN, 0, (struct sockaddr *) &addr_act, addr_act_len);
			ret_cube = __cobalt_sendto(socket_cube, bullet_cube_msg, BUFLEN_CUBE, 0, (struct sockaddr *) &addr_cube, addr_cube_len);
		}
	}
	close(socket_nom);
	close(socket_act);
	close(socket_cube);

	return;
}

void signal_handler(int signum)
{
    rt_task_delete(&motor_task);
	rt_task_delete(&bullet_task);
    rt_task_delete(&print_task);

    ecat_master.deactivate();

    printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGTSTP)
		printf("╔═══════════════[SIGNAL INPUT SIGTSTP]══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	

    exit(1);
}

int main(int argc, char *argv[])
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

    CPU_SET(5, &cpuset_qt);  
    CPU_SET(6, &cpuset_rt1);  
    CPU_SET(7, &cpuset_rt2);  

#ifdef __RP__
	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indyrp2/indyrp2.json", period);
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indyrp2/indyrp2.json", period);
#else
	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
#endif
    rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &safety_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

	rt_task_create(&bullet_task, "bullet_task", 0, 60, 0);
    // rt_task_set_affinity(&bullet_task, &cpuset_rt1);
    rt_task_start(&bullet_task, &bullet_run, NULL);

    rt_task_create(&print_task, "print_task", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt1);
    // rt_task_start(&print_task, &print_run, NULL);

    // Must pause here
    pause();

    // Finalize
    signal_handler(0);

    return 0;
}

