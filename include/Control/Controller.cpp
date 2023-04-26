/*
 * Controller.cpp
 *
 *  Created on: 2019. 5. 15.
 *      Author: Administrator
 */

#include "Controller.h"

namespace HYUControl {

Controller::Controller():m_Jnum(6)
{
	m_KpBase = KpBase;
	m_KdBase = KdBase;
	m_KiBase = KiBase;
	Kp.setZero();
	Kd.setZero();
	Ki.setZero();

	e.setZero();
	e_dev.setZero();
	e_int.setZero();

}

Controller::Controller(int JointNum)
{

    m_Jnum = JointNum;
    m_KpBase = KpBase;
    m_KdBase = KdBase;
    m_KiBase = KiBase;
    Kp.setZero();
    Kd.setZero();
    Ki.setZero();
    KpTask.resize(6);
    KdTask.resize(6);
    KiTask.resize(6);

    e.setZero();
    e_dev.setZero();
    e_int.setZero();
    e_old.setZero();
    eTask.resize(6);
    edotTask.resize(6);
    xd_buff.setZero();



}
Controller::Controller(robot *pManipulator, int JointNum)
{
    this->pManipulator=pManipulator;

    m_Jnum = JointNum;
    m_KpBase = KpBase;
    m_KdBase = KdBase;
    m_KiBase = KiBase;
    Kp.setZero();
    Kd.setZero();
    Ki.setZero();
    KpTask.resize(m_Jnum);
    KdTask.resize(m_Jnum);
    KiTask.resize(m_Jnum);

    e.setZero();
    e_dev.setZero();
    e_int.setZero();
    e_old.setZero();
    eTask.resize(m_Jnum);
    edotTask.resize(m_Jnum);

    dq.resize(m_Jnum);
    dqdot.resize(m_Jnum);
    dqddot.resize(m_Jnum);
    dq.setZero();
    dqdot.setZero();
    dqddot.setZero();
    xd_buff.setZero();

    G.resize(ROBOT_DOF,1);
    M.resize(ROBOT_DOF,ROBOT_DOF);
    C.resize(ROBOT_DOF,ROBOT_DOF);

	f_a << 1.081, 2.992, 0.6053, 0.9424, 189.1,			1;//J5: 0.4528;
	f_b << 2895, 376, 21.01, 482.4, 55.05,				1;//J5: 681.6;
	f_c << 0.5047, 0.01382, 1.628, 1.1155, 54.62,		1;//J5: 0.2781;
	f_d << 1.09, 0.7048, 1.325, 1.514, 0.9182,			1;//J5: 0.2215;
	f_e << 1.97, 1.242, 1844, 3.528, 18.12,				1;//J5: 0.7493;
	f_f << 0.3709, 1.114, 0.0, 0.0005186, 1.056,		1;//J5: 0.3944;

	init_time_vsd=0;
    t_flag=0;
}

void Controller::SetPIDGain(float _Kp, float _Kd, float _Ki, int _JointNum){
    Kp(_JointNum) = _Kp;
    Kd(_JointNum) = _Kd;
    Ki(_JointNum) = _Ki;
    return;
}

void Controller::PDController_gravity(float *q, float *q_dot, float *dq, float *dq_dot, float *toq, Jointf &g_mat)
{
	//Real
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946, 1;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691, 1;
	//Kp={0.0,};
	//Kd={0.0,};
	//Kp << 5,		3,		3,		3,		3;
	//Kd << 0.05,		0.03,	0.03,	0.03,	0.03;


//		int i = 1; //Joint Number
	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q[i];
		e_dev(i) = dq_dot[i] - q_dot[i];

/*		if(i==0)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
		else if(i==2)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
		else if(i==3)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
		else if(i==4)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
*/
		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;

		//For Simulation
/*		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
*/
			else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
			return;
	}

}

void Controller::PD_Gravity(float * q, float * q_dot, float *dq_, float *dq_dot, float * toq)
{
    //Kp << 40,        40,	    30,      35,      15;
	//Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946;
    //Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
    //Kd << 0.23,	    0.23,   0.18,    0.16,   0.12;
    //Kd << 0.03375482 * Kp(0), 0.034499895*Kp(1), 0.0345007*Kp(2), 0.015999985*Kp(3), 0.03599971*Kp(4);

	Kp << 28.7931,		32.5279,	18.764,		23.2564,		14.1946,	14.1946;//5.1946;
    Kd << 0.00575482 * Kp(0), 0.005499895*Kp(1), 0.0045007*Kp(2), 0.004599985*Kp(3), 0.00459971*Kp(4), 0.00459971*Kp(5);

    G.resize(ROBOT_DOF,1);
    G=pManipulator->pDyn->G_Matrix();

    for(int i=0; i<m_Jnum; ++i) {

        e(i) = dq_[i] - q[i];
        e_dev(i) = dq_dot[i] - q_dot[i];

            //For Simulation
/*		if(i==0)
			toq[i] = G(i);
		else if(i==1)
			toq[i] = G(i);
		else if(i==2)
			toq[i] = G(i);
		else if(i==3)
			toq[i] = G(i);
		else if(i==4)
			toq[i] = G(i);*/
        //For Simulation
        if(i==0)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*GEAR_RATIO_121)*1000*1.66;
        	//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*GEAR_RATIO_121)*1000*1.66;
        	//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
        else if(i==2)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*GEAR_RATIO_121)*1000*1.66;
        	//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
        else if(i==3)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*GEAR_RATIO_101)*1000*1.66;
        	//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
        else if(i==4)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*GEAR_RATIO_101)*1000*1.66;
        	//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
        else if(i==5)
        	toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*GEAR_RATIO_101)*1000*1.66;
			//toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i))/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
        else
            return;
    }

}

void Controller::Gravity(float * q, float * q_dot, float * toq)
{
    G.resize(ROBOT_DOF,1);
    G=pManipulator->pDyn->G_Matrix();

    for(int i=0; i<m_Jnum; ++i) {

        //For Simulation
		if(i==0)
			toq[i] = G(i)/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
			toq[i] = G(i)/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
		else if(i==2)
			toq[i] = G(i)/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
		else if(i==3)
			toq[i] = G(i)/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
		else if(i==4)
			toq[i] = G(i)/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
		else if(i==5)
			toq[i] = G(i)/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
		else
            return;
    }
}

    void Controller::Inverse_Dynamics_Control(float *q_, float *q_dot, float *dq_, float *dq_dot, float *dq_ddot, float * toq)
    {
        //Kp << 118.7931,		119.5279,	59.764,		160.2564,		39.1946;
        //Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
        //Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;
        //Kp << 11.7931,		11.5279,	5.764,		16.2564,		3.1946;
        //Kd << 2*sqrt(Kp(0)), 2*sqrt(Kp(1)), 2*sqrt(Kp(2)), 2*sqrt(Kp(3)), 2*sqrt(Kp(4));
        Kp << 150,        200,	    200,      200,      500,	100;
        Ki << 100,        100,      100,     100,   400,	400;
        //Kd << 0.03375482 *2* Kp(0), 2*0.034499895*Kp(1), 2*0.0345007*Kp(2), 2*0.015999985*Kp(3), 2*0.03599971*Kp(4);
        Kd << 2*sqrt(Kp(0)), 2*sqrt(Kp(1)), 2*sqrt(Kp(2)), 2*sqrt(Kp(3)), 2*sqrt(Kp(4)), 2*sqrt(Kp(5));

        G.resize(ROBOT_DOF,1);
        G=pManipulator->pDyn->G_Matrix();
        M.resize(ROBOT_DOF,ROBOT_DOF);
        M=pManipulator->pDyn->M_Matrix();
        C.resize(ROBOT_DOF,ROBOT_DOF);
        C=pManipulator->pDyn->C_Matrix();

        q = Map<VectorXf>(q_, this->m_Jnum);
        qdot = Map<VectorXf>(q_dot, this->m_Jnum);

        dq = Map<VectorXf>(dq_, this->m_Jnum);
        dqdot = Map<VectorXf>(dq_dot, this->m_Jnum);
        dqddot = Map<VectorXf>(dq_ddot, this->m_Jnum);
        //dq=dq_;
        //dqdot=dq_dot;
        //dqddot=dq_ddot;

        Jointf u, dq_dd;

        e = dq - q;
        e_dev= dqdot - qdot;
        e_int = e_old+e*0.001;

        e_old=e_int;


        u0=dqddot + Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e);// +Ki.cwiseProduct(e_int);
        u= M * u0 + C * qdot + G;


        for(int i=0; i<m_Jnum; ++i)
        {
            if(i==0)
                toq[i] = u(i)/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
            else if(i==1)
                toq[i] = u(i)/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
            else if(i==2)
                toq[i] = u(i)/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
            else if(i==3)
                toq[i] = u(i)/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
            else if(i==4)
                toq[i] = u(i)/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
            else if(i==5)
				toq[i] = u(i)/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
            else
            	return;
        }
    }
    void Controller::ComputedTorque(float *q_, float *q_dot, float *dq_, float *dq_dot, float *dq_ddot, float * toq)
    {
        Kp << 20,        23,	    10,      15,      5,	5;
        Kd << 0.03375482 * Kp(0), 0.034499895*Kp(1), 0.0345007*Kp(2), 0.015999985*Kp(3), 0.03599971*Kp(4), 0.03599971*Kp(5);

        G=pManipulator->pDyn->G_Matrix();
        M=pManipulator->pDyn->M_Matrix();
        //C=pManipulator->pDyn->C_Matrix();

        q = Map<VectorXf>(q_, this->m_Jnum);
        qdot = Map<VectorXf>(q_dot, this->m_Jnum);

        dq = Map<VectorXf>(dq_, this->m_Jnum);
        dqdot = Map<VectorXf>(dq_dot, this->m_Jnum);
        dqddot = Map<VectorXf>(dq_ddot, this->m_Jnum);
        //dq=dq_;
        //dqdot=dq_dot;
        //dqddot=dq_ddot;


        Jointf u, uff, dq_dd;

        e = dq - q;
        e_dev= dqdot - qdot;

        uff = M * dqddot + C * dqdot + G;
        u0=Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e);
        u= uff +u0;


        for(int i=0; i<m_Jnum; ++i)
        {
            if(i==0)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
            else if(i==1)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
            else if(i==2)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
            else if(i==3)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
            else if(i==4)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
            else if(i==5)
                toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
            else
            	return;
        }
    }

    void Controller::VSD(float *_q, float *_qdot, Vector3f &xd, float *toq, float gt, int flag)
    {
        float tmp_M=0;
        float alpha=8.0, kt=0;


        mvZeta0_=0.1; mvZeta1_=1.0; mvK_=250; mvKsi_=0.1;
        mvC0_.resize(this->m_Jnum);
        M.resize(this->m_Jnum,this->m_Jnum);
        q.resize(this->m_Jnum);
        q=Map<VectorXf>(_q,this->m_Jnum);
        qdot.resize(this->m_Jnum);
        qdot=Map<VectorXf>(_qdot,this->m_Jnum);
        dq.resize(this->m_Jnum);
        dq<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        eTask.resize(this->m_Jnum);
        eTask=q-dq;

        x=pManipulator->pKin->ForwardKinematics();
        l_Jaco=pManipulator->pKin->LinearJacobian();
        x_dot=l_Jaco*qdot;
        M=pManipulator->pDyn->M_Matrix();
        G=pManipulator->pDyn->G_Matrix();
        /////////////////////////// desired point가 다른 경우 시간 초기화 코드
        if(xd_buff(0)!=xd(0) || xd_buff(1)!=xd(1) || xd_buff(2)!=xd(2) || flag==0)
        	init_time_vsd=gt;


        for(int i=0; i<m_Jnum ; i++)
        {
            for(int j=0; j<m_Jnum ; j++)
            {
                tmp_M+=M(i,j);
            }
            mvC0_(i)=mvZeta0_*sqrt(mvK_)*sqrt(abs(tmp_M));
            tmp_M=0;
        }

        kt=mvK_*(1.0-(1.0+alpha*(gt-init_time_vsd)+powf(alpha,2)*powf((gt-init_time_vsd),2)/2.0)*expf(-alpha*(gt-init_time_vsd)));
        //u0=-mvC0_.cwiseProduct(qdot)-l_Jaco.transpose()*(mvK_*(x-xd)+mvZeta1_*sqrt(mvK_)*x_dot)+G;
        u0=-mvC0_.cwiseProduct(qdot)-mvKsi_*mvC0_.cwiseProduct(eTask)-l_Jaco.transpose()*(kt*(x-xd)+mvZeta1_*sqrt(mvK_)*x_dot)+G;
        //u0=2/3.141592*atan()
        for(int i=0; i<m_Jnum; ++i)
        {

            if(i==0){
                toq[i] = u0(i)/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
                //toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
            }
            else if(i==1){

                toq[i] = u0(i)/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
            	//toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
            }
            else if(i==2){
                toq[i] = u0(i)/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
                //toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
            }
            else if(i==3){
                toq[i] = u0(i)/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
                //toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
            }
            else if(i==4){
                toq[i] = u0(i)/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
                //toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
            }
            else if(i==5){
            	toq[i] = u0(i)/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
            	//toq[i] = (u0(i)+FrictionCompensation(qdot(i),i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
            }
            else
                return;
        }
        xd_buff=xd;

    }

void Controller::PDController(Jointf &q, Jointf &q_dot, float *dq, float *dq_dot, float *toq)
{
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946,	19.1946;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691,	0.691;

	int i = 1;
//	for(int i=0; i<m_Jnum; ++i)
//	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);

	//	e_int.at(i) = e.at(i)+e_old.at(i);
	//	e_old.at(i)=e.at(i);
		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		else if(i==5)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000;
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
			return;
	/*	if(i==0)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		else if(i==1)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		else if(i==2)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		else if(i==3)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		else if(i==4)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}
			return;*/
//	}


}
/*
void Controller::Impedance(float *_q_dot, Matrix<float, 6, 1> & _x,Matrix<float, 6, 1> & _x_dot, Matrix<float, 6, 1> & _dx, Matrix<float, 6, 1> & _dx_dot, Matrix<float, 6, 1> & _dx_ddot, float * toq)
{
    M=pManipulator->pDyn->M_Matrix();
    C=pManipulator->pDyn->C_Matrix();
    G=pManipulator->pDyn->G_Matrix();
    qdot = Map<VectorXf>(_q_dot, this->m_Jnum);

    _a_jaco.resize(6,ROBOT_DOF);
    _a_jaco=pManipulator->pKin->AnalyticJacobian();
    _pinv_jaco.resize(ROBOT_DOF,6);
    _pinv_jaco=pManipulator->pKin->DPI_l(_a_jaco);
    _jaco_dot.resize(6,ROBOT_DOF);
    _jaco_dot=pManipulator->pKin->Jacobian_dot();

    Matrix<float,6,6> Md,Kd,Bd;
    Md = _pinv_jaco.transpose()*M*_pinv_jaco;
    Kd=100*Kd.setIdentity();
    Bd=0.7*Bd.setIdentity();

    ax.resize(6,1);
    ax=_dx_ddot-Md.inverse()*(Bd*(_x_dot-_dx_dot)+Kd*(_x-_dx));

    u0=M*_pinv_jaco*(ax-_jaco_dot*qdot)+C*qdot+G;
    for(int i=0; i<m_Jnum; ++i) {

        if (i == 0) {

            toq[i] = u0(i);
        } else if (i == 1) {

            toq[i] = u0(i);
        } else if (i == 2) {

            toq[i] = u0(i);
        } else if (i == 3) {

            toq[i] = u0(i);
        } else if (i == 4) {

            toq[i] = u0(i);
        } else

            return;
    }
}*/
    /*
void Controller::Impedance(Jointf &q, Jointf &q_dot, Jointf &q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Jointf &g_mat)
{
	Damp << 0.5, 0.5, 0.5, 0.5, 0.5;
	Stiff << 8.0,8.0,8.0,8.0,8.0;

	Jointf u;

	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);

	}

	//	int i = 0; //Joint Number
	u = m_mat*q_ddot +g_mat + Damp.cwiseProduct(e_dev) + Stiff.cwiseProduct(e);

	for(int i=0; i<m_Jnum; ++i)
	{

		if(i==0){

			toq[i] = u(i)/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){

			toq[i] = u(i)/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){

			toq[i] = u(i)/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){

			toq[i] = u(i)/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){

			toq[i] = u(i)/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else

			return;
	}
}
void Controller::Impedance(Jointf &q, Jointf &q_dot, Jointf &q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Jointf &g_mat, Matrixf &c_mat)
{
//	Damp << 3.0, 3.0, 1.4, 2.8, 1.4;
//	Stiff << 40.0,40.0,20.0,40.0,20.0;
	Damp << 1.0, 1.0, 1.0, 1.0, 1.0;
	Stiff << 30.0,30.0,30.0,30.0,30.0;

	Jointf u, dq_ddot_vec;

	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);
		dq_ddot_vec(i) = dq_ddot[i];
	}

	//	int i = 0; //Joint Number
	u = m_mat * dq_ddot_vec + c_mat* q_dot + g_mat + Damp.cwiseProduct(e_dev) + Stiff.cwiseProduct(e);

	for(int i=0; i<m_Jnum; ++i)
	{

		if(i==0){

			toq[i] = u(i)/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){

			toq[i] = u(i)/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){

			toq[i] = u(i)/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){

			toq[i] = u(i)/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){

			toq[i] = u(i)/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else

			return;
	}
}*/

/*
void Controller::Impedance(float *q, float *q_dot, float *q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrix6f &m_mat, Matrix6f &c_mat, Jointf &g_mat)
{
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;
	Damp={0.5, 0.5, 0.5, 0.5, 0.5};
	Stiff={8.0,8.0,8.0,8.0,8.0};

	//	int i = 0; //Joint Number
	for(int i=0; i<m_Jnum; ++i)
	{

		e.at(i) = dq[i] - q[i];
		e_dev.at(i) = dq_dot[i] - q_dot[i];
		//Jointf g = Liedynamics::G_Matrix();

		if(i==0){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
/*			return;
	}
}*/


void Controller::TorqueOutput(float *p_toq , int maxtoq, int *p_dir)
{
    float toq_tmp=0;
	for(int i=0; i<m_Jnum; ++i)
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

void Controller::CLIKController(float * _q, float * _qdot, float * _dq, float * _dqdot, const Eigen::VectorXf * _dx, const Eigen::VectorXf * _dxdot, const Eigen::VectorXf & _dqdotNull, float * p_Toq, float & _dt)
{
    G=pManipulator->pDyn->G_Matrix();
    q = Map<VectorXf>(_q, this->m_Jnum);
    qdot = Map<VectorXf>(_qdot, this->m_Jnum);

    LinearJacobian=pManipulator->pKin->LinearJacobian();

    eTask.setZero();
    edotTmp.setZero();
    edotTask.setZero();

}

void Controller::CLIKController_2nd(float *_q, float *_qdot, Matrix<float,6,1>& dq, Matrix<float,6,1>& dq_dot, Matrix<float,6,1>& dq_ddot)
{

}

void Controller::FrictionIdentify(float *_q, float *_qdot, float *dq_, float *dq_dot, float *dq_ddot, float *toq, float _gt)
{
	Kp << 15.7931,		18.5279,	3.764,		3.2564,		2.1946,		2.1946;
    Kd << 0.00575482 * Kp(0), 0.005499895*Kp(1), 0.0045007*Kp(2), 0.004599985*Kp(3), 0.00459971*Kp(4), 0.00459971*Kp(5);
    //Kp << 118.7931,		119.5279,	59.764,		100.2564,		19.1946;
    //Kd << 0.01375482 * Kp(0), 0.014499895*Kp(1), 0.0145007*Kp(2), 0.005999985*Kp(3), 0.01599971*Kp(4);
    G.resize(ROBOT_DOF,1);
    G=pManipulator->pDyn->G_Matrix();
	Jointf u;
	float T, w, Mag;
	int joint;

	dq<<0.0 , 0.0, 0.0, 0.0, 0.0, 0.0;
 	if(t_flag==0)
	{
		init_time=_gt;
		t_flag=1;
	}
	else
	{

		//joint=4;
		for(int joint=0; joint<m_Jnum; ++joint)
		{
			switch(joint)
			{
			case 0:
				T=20.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time));
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			case 1:
				T=20.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time))-30*M_PI/180;
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			case 2:
				T=20.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time));
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			case 3:
				T=20.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time))+50*M_PI/180;
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			case 4:
				T=10.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time));
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			case 5:
				T=10.0; w=2.0*M_PI/T; Mag=50;
				dq[joint]=Mag*M_PI/180*sin(w*(_gt-init_time));
				dqdot[joint]=Mag*M_PI/180*w*cos(w*(_gt-init_time));
				//u[joint]=Kp[joint]*e[joint]+Kd[joint]*e_dev[joint];
				break;
			default:
				break;

			}

		}
		for(int i=0; i<m_Jnum; ++i)
		{

			e(i) = dq[i] - _q[i];
			e_dev(i) = dqdot[i] - _qdot[i];

		}
		u=Kp.cwiseProduct(e)+Kd.cwiseProduct(e_dev)+G;
		//u=G;
		//+FrictionCompensation(dqdot[i],i)
		for(int i=0;i<m_Jnum;i++)
		{
			if(i==0){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
			}
			else if(i==1){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
			}
			else if(i==2){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
			}
			else if(i==3){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
			}
			else if(i==4){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
			}
			else if(i==5){
				toq[i] = (u(i)+FrictionCompensation(dqdot[i],i))/(float)(TORQUE_CONST_6*MAX_CURRENT_6*HARMONIC_100)*1000*1.66;
				//toq[i] = (u(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
			}
			else
				return;
		}

		Map<VectorXf>(dq_,this->m_Jnum)=dq;
		Map<VectorXf>(dq_dot,this->m_Jnum)=dqdot;
	}
}

float Controller::FrictionCompensation(float _qdot, int _JointNum)
{
	float res;
	res=f_a[_JointNum]*(tanh(f_b[_JointNum]*_qdot)-tanh(f_c[_JointNum]*_qdot))+f_d[_JointNum]*tanh(f_e[_JointNum]*_qdot)+f_f[_JointNum]*_qdot;
	return res;
}

Jointf Controller::return_u0(void)
{
	return u0;
}

} /* namespace HYUCtrl */
