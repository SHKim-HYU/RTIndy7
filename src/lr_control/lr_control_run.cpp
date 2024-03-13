#include "lr_control_run.h"
#include "waypoint.h"
#include <LR/include/liegroup_robotics.h>
#include "../ecat/ecat_run.h"

using namespace lr;
ScrewList LeftSlist,RightSlist,RightBlist,LeftBlist;
SE3 LeftM,RightM,RelM;
SE3 Tbr,Tbl;
RelScrewList RelSlist,RelBlist;

//D11936I07004
void real_parameter_settings(){
	//Kinematic Calibration results
	LeftSlist<< 0.0,          0.30038698,  0.74851057 ,-0.00136847  ,1.09807653 ,-0.18264502,
				-0.0009083,   0.00010428,  0.00088469, -0.00067349,  0.00127813, -0.00437526,
				-0.00000043, -0.00092475, -0.00095012, -0.00000132, -0.00117119, -0.00046257,
				0.         , 0.00034208,  0.00117984,  0.00043815,  0.00115955, -0.00243131,
				-0.00047124 ,-0.99999859, -0.99999796, -0.00284616, -0.99999076, -0.00422828,
				0.99999989 ,-0.00164235, -0.0016424 ,  0.99999585, -0.00413995,  0.99998811;
	RightSlist<< 0.        ,  0.29810954,  0.74870726, -0.0055249,   1.09928541, -0.18706666,
				 0.00397988, -0.00005562,  0.00147425,  0.00110991, -0.00132848, -0.01569872,
				 0.00001137,  0.00389446,  0.00336214, -0.00001857,  0.00456952, -0.00350901,
				-0.        , -0.00015708,  0.0019792 , -0.00338485, -0.00120828, -0.01862742,
				-0.0028571 , -0.99999744, -0.99999549, -0.00011394, -0.99999927, -0.00151796,
				 0.99999592, -0.00225845, -0.00226099,  0.99999426, -0.00005171,  0.99982534;		
	RelSlist<<   0,   -0.2270 ,   0.1831 ,  -0.5761   ,-1.0242 ,   0.1876 ,  -1.3995  , -1.0647  , -1.5153  , -1.3918 ,  -1.8690,   -1.2111,
         0 ,  -0.0003 ,  -0.0001 ,  -0.0007  , -0.0005 ,   0.0003  ,  0.0012 ,  -0.0028 ,   0.0018  , -0.0056 ,  -0.0057 ,  -0.0352,
         0,    0.0003 ,  -0.0005 ,   0.0014 ,   0.0027 ,  -0.0005 ,   0.0009 ,  0.0013  ,  0.0014  ,  0.0033  ,  0.0040  ,  0.0188,
         0 ,   0.0013  ,  0.0029 ,   0.0013 ,   0.0005 ,   0.0024 ,  -0.0011 ,   0.0023 ,   0.0002 ,   0.0023 ,   0.0034 ,   0.0175,
         0 ,  -1.0000 ,   0.0014 ,  -1.0000 ,  -1.0000 ,   0.0038 ,  -0.8695 ,  -0.4943 ,  -0.4943 ,  -0.8682 ,  -0.4962 ,  -0.8687,
    1.0000 ,   0.0001 ,   1.0000 ,   0.0026 ,   0.0026 ,   1.0000 ,  -0.4938 ,   0.8692 ,   0.8692 , -0.4962  ,  0.8681 ,  -0.4949;	
	LeftM<< 0.99999704  ,0.00010386 ,-0.00243131  ,0.00115543,
			-0.00011414 , 0.99999105 ,-0.00422828 ,-0.18824687,
			0.00243085 , 0.00422854 , 0.99998811  ,1.32432479,
			0.        ,  0.        ,  0.         , 1.        ;				 		

	RightM<< 0.9998197,   0.00368465 ,-0.01862742, -0.0090968, 
	-0.00371357 , 0.99999195 ,-0.00151796, -0.18912017,
	0.01862168,  0.00158686 , 0.99982534,  1.33104441,
	0.        ,  0.         , 0.        ,  1.        ;
	RelM<< -0.9998   ,-0.0058,    0.0175,    0.0030,
	-0.0181    ,0.4949 ,  -0.8687,   -1.2242,
	-0.0036   ,-0.8689 ,  -0.4949 ,  -2.0915,
			0   ,      0 ,        0 ,   1.0000;

	LeftBlist = Ad(LeftM)*LeftSlist;
	RightBlist = Ad(RightM)*RightSlist;
	RelBlist = Ad(RelM)*RelSlist;

Tbr <<    1.0000     ,    0     ,    0    ,     0,
         0  ,  0.5000   , 0.8660  ,  0.1563,
         0  , -0.8660 ,   0.5000  ,  0.3772,
         0  ,       0  ,       0  ,  1.0000;

Tbl <<   -1.0000     ,    0    ,     0  ,       0,
         0  , -0.5000  , -0.8660  , -0.1563,
         0   ,-0.8660  ,  0.5000 ,   0.3772,
         0  ,       0  ,       0  ,  1.0000;

}
void rt_lr_control_run(void *param)
{
	RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	// nrmk_master.syncEcatMaster();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, CYCLE_NS);

	real_parameter_settings();




    int ft_init_cnt = 0;
	JVec tau_l = JVec::Zero();
	JVec tau_r = JVec::Zero();
	JVec eint_l = JVec::Zero();
	JVec eint_r = JVec::Zero();
	double dt= 0.001;
	JVec Hinf_K =JVec::Zero();
	JVec gamma = JVec::Zero();
	JVec gamma_r = JVec::Zero();
 	Hinf_K << 300.0, 300.0, 80.0, 80.0, 70.0, 70.0;
 	gamma << invL2sqr_1, invL2sqr_2, invL2sqr_3, invL2sqr_4, invL2sqr_5, invL2sqr_6;
	gamma_r<< 800,600,500,500,500,600;
	std::vector<JVec> joint_way_points_l,joint_way_points_r;
	std::vector<double> joint_delays;

 	std::vector<SE3> task_way_points;
 	std::vector<double> task_delays;


	JVec q0 =JVec::Zero();
	JVec q_home_l,q_home_r;
	q_home_l<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;
	q_home_r<<-0.14558084, -0.813371 ,-1.3075236,-0.614656,-1.03065,0.0693011;
	JVec q_motion1_l,q_motion1_r;

    q_motion1_l<<1.37353,   0.537386,    1.26897 ,  -0.35138,    1.54321 ,-0.0690937;
	q_motion1_r<<-1.37353,   -0.537386,    -1.26897 ,  0.35138,    -1.54321 ,0.0690937;
 	info.des.q_r = q0;
 	info.des.q_dot_r = JVec::Zero();
 	info.des.q_ddot_r = JVec::Zero();

	info.des.q_l = q0;
 	info.des.q_dot_l = JVec::Zero();
 	info.des.q_ddot_l = JVec::Zero();

	LR_Trajectory lr_traj = LR_Trajectory();
	bool is_set_traj = 0;
	int print_cnt  = 0;
	Vector6d lambda_int = Vector6d::Zero();

	Matrix6d Task_Kp = Matrix6d::Identity() * 6000.0;
	Task_Kp(3,3)=Task_Kp(3,3)/5.0;
	Task_Kp(4,4)=Task_Kp(4,4)/5.0;
	Task_Kp(5,5)=Task_Kp(5,5)/5.0;
	Matrix6d Task_Kv = Matrix6d::Identity() * 200.0;
	Task_Kv(3,3)=Task_Kv(3,3)/5.0;
	Task_Kv(4,4)=Task_Kv(4,4)/5.0;
	Task_Kv(5,5)=Task_Kv(5,5)/5.0;
	JVec q_dot_filtered = JVec::Zero();	
	JVec K_ir;

	JVec e_int_nr_l=JVec::Zero();
	JVec e_int_dn_l=JVec::Zero();
	JVec e_int_nr_r=JVec::Zero();
	JVec e_int_dn_r=JVec::Zero();
	while (run)
	{
		rt_ecat_setup_start( beginCycle, endCycle, beginRead,  beginReadbuf,  beginWrite, beginWritebuf, beginCompute,  ft_init_cnt);
		if (system_ready)
		{
			if(is_set_traj==0){
				if(use_job==0){
					//std::cout<<"System Ready Use Job"<<std::endl;
					info.des.q_l = info.act.q_l;
					info.des.q_dot_l=JVec::Zero();
					info.des.q_ddot_l=JVec::Zero();

					info.des.q_r = info.act.q_r;
					info.des.q_dot_r=JVec::Zero();
					info.des.q_ddot_r=JVec::Zero();

					joint_way_points_l.push_back(info.act.q_l);
					joint_way_points_l.push_back(info.act.q_l);
					joint_way_points_l.push_back(q_home_l);
					joint_way_points_l.push_back(JVec::Zero());
					joint_way_points_l.push_back(q_motion1_l);
					joint_way_points_l.push_back(q_motion1_l);
					joint_way_points_l.push_back(q_home_l);
					
					joint_way_points_r.push_back(info.act.q_r);
					joint_way_points_r.push_back(info.act.q_r);
					joint_way_points_r.push_back(q_home_r);
					joint_way_points_r.push_back(JVec::Zero());
					joint_way_points_r.push_back(q_motion1_r);
					joint_way_points_r.push_back(q_motion1_r);
					joint_way_points_r.push_back(q_home_r);

					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);					
					joint_delays.push_back(5.0);		
					joint_delays.push_back(5.0);		


					//JointWayPointGenerator(joint_way_points_l, joint_delays, info.act.q_l);
					//JointWayPointGenerator(joint_way_points_r, joint_delays, info.act.q_r);


					//SE3 X0 =  lr::FKinBody(lr_control.M,lr_control.Blist,info.act.q);
					//TaskWayPointGenerator(task_way_points, task_delays, X0);
					//tau_l = lr_control.HinfControl(info.act.q_l, info.act.q_dot_l, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l, eint_l, dt, Hinf_K, gamma);			
					//tau_r = lr_control.HinfControl(info.act.q_r, info.act.q_dot_r, info.des.q_r, info.des.q_dot_l, info.des.q_ddot_l, eint_r, dt, Hinf_K, gamma);			

					tau_l = lr_control.GravityForces(info.act.q_l);
					tau_r = lr_control.GravityForces(info.act.q_r);

					eint_l=eint_l*0;		
					eint_r=eint_r*0;		
					gt=0;		
					is_set_traj = 1;
				}
				if(use_job==1){
					info.des.q_l = info.act.q_l;
					info.des.q_dot_l=JVec::Zero();
					info.des.q_ddot_l=JVec::Zero();

					info.des.q_r = info.act.q_r;
					info.des.q_dot_r=JVec::Zero();
					info.des.q_ddot_r=JVec::Zero();

					info.nom.q_l = info.act.q_l;
					info.nom.q_dot_l=info.act.q_dot_l;
					info.nom.q_ddot_l=JVec::Zero();

					info.nom.q_r = info.act.q_r;
					info.nom.q_dot_r=info.act.q_dot_r;
					info.nom.q_ddot_r=JVec::Zero();

					joint_way_points_l.push_back(info.act.q_l);
					joint_way_points_l.push_back(info.act.q_l);
					joint_way_points_l.push_back(q_home_l);
					joint_way_points_l.push_back(JVec::Zero());
					joint_way_points_l.push_back(q_motion1_l);
					joint_way_points_l.push_back(q_motion1_l);
					joint_way_points_l.push_back(q_home_l);
					
					joint_way_points_r.push_back(info.act.q_r);
					joint_way_points_r.push_back(info.act.q_r);
					joint_way_points_r.push_back(q_home_r);
					joint_way_points_r.push_back(JVec::Zero());
					joint_way_points_r.push_back(q_motion1_r);
					joint_way_points_r.push_back(q_motion1_r);
					joint_way_points_r.push_back(q_home_r);


					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);		
					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);
					joint_delays.push_back(5.0);

					
					//JointWayPointGenerator(joint_way_points_l, joint_delays, info.act.q_l);
					//JointWayPointGenerator(joint_way_points_r, joint_delays, info.act.q_r);


					//SE3 X0 =  lr::FKinBody(lr_control.M,lr_control.Blist,info.act.q);
					//TaskWayPointGenerator(task_way_points, task_delays, X0);
					tau_l = lr_control.HinfControl(info.act.q_l, info.act.q_dot_l, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l, eint_l, dt, Hinf_K, gamma);			
					tau_r = lr_control.HinfControl(info.act.q_r, info.act.q_dot_r, info.des.q_r, info.des.q_dot_l, info.des.q_ddot_l, eint_r, dt, Hinf_K, gamma_r);			

					//tau_l = lr_control.NRICControl(info.act.q_l, info.act.q_dot_l, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l,info.nom.q_l, info.nom.q_dot_l,  e_int_nr_l,  e_int_dn_l, dt,  Hinf_K,  gamma);
					//tau_r = lr_control.NRICControl(info.act.q_r, info.act.q_dot_r, info.des.q_r, info.des.q_dot_r, info.des.q_ddot_r,info.nom.q_r, info.nom.q_dot_r,  e_int_nr_r,  e_int_dn_r, dt,  Hinf_K,  gamma);

					//tau_l = lr_control.GravityForces(info.act.q_l);
					//tau_r = lr_control.GravityForces(info.act.q_r);

					eint_l=eint_l*0;		
					eint_r=eint_r*0;
					gt=0;		
					is_set_traj = 1;
				}
			}						
			else{
				if(use_control==1){
					tau_l = lr_control.GravityForces(info.act.q_l);
					tau_r = lr_control.GravityForces(info.act.q_r);
				}
				if(use_grav_comp){
					// JVec q_dot = info.act.q_dot;		
					// Jacobian Jb = JacobianBody(lr_control.Blist,info.act.q);
					// JVec tau_ext = Jb.transpose()*info.act.F_ext;
    				// K_ir << 7.0, 7.0,5.0, 5.0, 3.0, 3.0, 3.0;					
					// JVec u = JVec::Zero();					
					// //torques = lr_control.GravityForces(info.act.q)-tau_ext + K_ir.cwiseProduct(u + tau_ext) ;

					//std::cout<<"q_l:"<<info.act.q_l.transpose()<<std::endl;
					//std::cout<<"q_l:"<<info.act.q_r.transpose()<<std::endl;
					tau_l = lr_control.GravityForces(info.act.q_l);
					tau_r = lr_control.GravityForces(info.act.q_r);
				}
				if(use_job){
	
					lr_traj.WayPointJointTrajectory(joint_way_points_l, joint_delays, gt, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l,30);
					lr_traj.WayPointJointTrajectory(joint_way_points_r, joint_delays, gt, info.des.q_r, info.des.q_dot_r, info.des.q_ddot_r,30);

					//tau_l = lr_control.HinfControl(info.act.q_l, info.act.q_dot_l, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l, eint_l, dt, Hinf_K, gamma);			
					//tau_r = lr_control.HinfControl(info.act.q_r, info.act.q_dot_r, info.des.q_r, info.des.q_dot_l, info.des.q_ddot_l, eint_r, dt, Hinf_K, gamma);			
					tau_l = lr_control.NRICControl(info.act.q_l, info.act.q_dot_l, info.des.q_l, info.des.q_dot_l, info.des.q_ddot_l,info.nom.q_l, info.nom.q_dot_l,  e_int_nr_l,  e_int_dn_l, dt,  Hinf_K,  gamma,0.015);
					tau_r = lr_control.NRICControl(info.act.q_r, info.act.q_dot_r, info.des.q_r, info.des.q_dot_r, info.des.q_ddot_r,info.nom.q_r, info.nom.q_dot_r,  e_int_nr_r,  e_int_dn_r, dt,  Hinf_K,  gamma_r,0.015);

				}


			}
		}
		else
		{
			//std::cout<<"System Not Ready"<<std::endl;
			tau_l = lr_control.GravityForces(info.act.q_l);
			tau_r = lr_control.GravityForces(info.act.q_r);
		}
		info.des.tau_l = tau_l;
		info.des.tau_r = tau_r;
		rt_ecat_setup_end( beginCycle, endCycle, beginRead,  beginReadbuf,  beginWrite, beginWritebuf, beginCompute,  ft_init_cnt);
		rt_task_wait_period(NULL); // wait for next cycle
	}
}
