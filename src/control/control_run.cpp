#include "../global_vars.h"

void compute()
{
	// Update Indy7
	cs_indy7.updateRobot(info.act.q, info.act.q_dot);
	// Update nominal
	cs_nom_indy7.updateRobot(info.nom.q , info.nom.q_dot);
	
	info.act.T = cs_indy7.getFK();
	info.act.R = cs_indy7.getRMat();
	info.nom.T = cs_nom_indy7.getFK();
	info.nom.R = cs_nom_indy7.getRMat();

	Jacobian J_b = cs_indy7.getJ_b();
	Jacobian dJ_b = cs_indy7.getJdot_b();
	info.act.x_dot = cs_indy7.getBodyTwist();
	info.act.x_ddot = dJ_b*info.nom.q_dot + J_b*info.nom.q_ddot;
		
	Twist F_mometum = cs_indy7.computeF_Tool(info.act.x_dot, info.act.x_ddot);
	
	// info.act.F = F_tmp-F_mometum;
	// info.act.F = cs_indy7.computeF_Threshold(F_tmp);
	info.act.tau_ext = J_b.transpose()*(info.act.F);

	info.act.tau_fric = cs_indy7.FrictionEstimation(info.act.q_dot);
	// info.act.tau_fric = JVec::Zero();

	manipulability = cs_indy7.getManipulability();
}

void control()
{	
	info.des.tau = cs_indy7.computeG(info.act.q)-info.act.tau_ext;
}


