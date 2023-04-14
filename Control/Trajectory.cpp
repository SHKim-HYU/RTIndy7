/*
 * Trajectory.cpp
 *
 *  Created on: Nov 20, 2018
 *      Author: spec
 */

#include "Trajectory.h"

namespace hyuCtrl {



Trajectory::Trajectory():dq(0), dq_dot(0), dq_ddot(0) {

}

Trajectory::~Trajectory() {

}

void Trajectory::SetPolynomial5th(int NumJoint, float startPos, float FinalPos, float InitTime, float Duration)
{
	TrajDuration[NumJoint] = Duration;
	TrajInitTime[NumJoint] = InitTime;
	Coefficient[NumJoint].setZero();
	m_cof << 1, 0, 	0, 		0, 				0, 					0,
			 0, 1, 	0, 		0, 				0, 					0,
			 0, 0, 	2, 		0, 				0, 					0,
			 1, powf(TrajDuration[NumJoint],1), 	powf(TrajDuration[NumJoint],2), 	powf(TrajDuration[NumJoint],3), 	powf(TrajDuration[NumJoint],4), 	powf(TrajDuration[NumJoint],5),
			 0, 1, 									2*powf(TrajDuration[NumJoint],1), 	3*powf(TrajDuration[NumJoint],2), 	4*powf(TrajDuration[NumJoint],3), 	5*powf(TrajDuration[NumJoint],4),
			 0, 0, 									2, 									6*powf(TrajDuration[NumJoint],1), 	12*powf(TrajDuration[NumJoint],2),	20*powf(TrajDuration[NumJoint],3);

	StateVec[NumJoint] << startPos,
						0,
						0,
						FinalPos,
						0,
						0;

	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
	m_isReady[NumJoint] = 1;

}

void Trajectory::SetPolynomial5th_t(int NumJoint, state *act, float FinalPos, float InitTime, float Duration,float *q_)
{
	TrajDuration[NumJoint] = Duration;
	TrajInitTime[NumJoint] = InitTime;
	m_cof << 1, 0, 	0, 		0, 				0, 					0,
			 0, 1, 	0, 		0, 				0, 					0,
			 0, 0, 	2, 		0, 				0, 					0,
			 1, powf(TrajDuration[NumJoint],1), 	powf(TrajDuration[NumJoint],2), 	powf(TrajDuration[NumJoint],3), 	powf(TrajDuration[NumJoint],4), 	powf(TrajDuration[NumJoint],5),
			 0, 1, 									2*powf(TrajDuration[NumJoint],1), 	3*powf(TrajDuration[NumJoint],2), 	4*powf(TrajDuration[NumJoint],3), 	5*powf(TrajDuration[NumJoint],4),
			 0, 0, 									2, 									6*powf(TrajDuration[NumJoint],1), 	12*powf(TrajDuration[NumJoint],2),	20*powf(TrajDuration[NumJoint],3);

	StateVec[NumJoint] << act->x[NumJoint],
			//act->j_q_d(NumJoint),
			//act->j_q_dd(NumJoint),
						0,
						0,
						FinalPos,
						0,
						0;
	q_[0]=StateVec[NumJoint](0);
	q_[1]=StateVec[NumJoint](1);
	q_[2]=StateVec[NumJoint](2);
	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
	m_isReady[NumJoint] = 1;

}

void Trajectory::SetPolynomial5th_j(int NumJoint, state *act, float FinalPos, float InitTime, float Duration,float *q_, int traj_changed)
{
	TrajDuration[NumJoint] = Duration;
	TrajInitTime[NumJoint] = InitTime;
	m_cof << 1, 0, 	0, 		0, 				0, 					0,
			 0, 1, 	0, 		0, 				0, 					0,
			 0, 0, 	2, 		0, 				0, 					0,
			 1, powf(TrajDuration[NumJoint],1), 	powf(TrajDuration[NumJoint],2), 	powf(TrajDuration[NumJoint],3), 	powf(TrajDuration[NumJoint],4), 	powf(TrajDuration[NumJoint],5),
			 0, 1, 									2*powf(TrajDuration[NumJoint],1), 	3*powf(TrajDuration[NumJoint],2), 	4*powf(TrajDuration[NumJoint],3), 	5*powf(TrajDuration[NumJoint],4),
			 0, 0, 									2, 									6*powf(TrajDuration[NumJoint],1), 	12*powf(TrajDuration[NumJoint],2),	20*powf(TrajDuration[NumJoint],3);
	if(traj_changed)
	{
		StateVec[NumJoint] << act->dq[NumJoint],
							act->dq_dot[NumJoint],
							act->dq_ddot[NumJoint],
							//0,
							//0,
							FinalPos,
							0,
							0;
	}
	else
	{
		StateVec[NumJoint] << act->q[NumJoint],
							act->q_dot[NumJoint],
							act->q_ddot[NumJoint],
							//0,
							//0,
							FinalPos,
							0,
							0;
	}
	q_[0]=StateVec[NumJoint](0);
	q_[1]=StateVec[NumJoint](1);
	q_[2]=StateVec[NumJoint](2);
	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
	m_isReady[NumJoint] = 1;

}

float Trajectory::Polynomial5th(int NumJoint, float CurrentTime, int *Flag)
{
	if((CurrentTime - TrajInitTime[NumJoint]) >= (TrajDuration[NumJoint]))
	{
		//des->q(NumJoint) = StateVec[NumJoint](3);
		//des->q_dot(NumJoint) = StateVec[NumJoint](4);
		//des->q_ddot(NumJoint) = StateVec[NumJoint](5);


		m_isReady[NumJoint] = 0;
		*Flag = 0;
		return StateVec[NumJoint](3);
	}

	if(m_isReady[NumJoint])
	{
		dq=0;
		dq_dot = 0;
		dq_ddot = 0;

		TrajTime[NumJoint] = CurrentTime - TrajInitTime[NumJoint];
		for(int i=0; i<6; ++i)
		{
			dq += powf(TrajTime[NumJoint], i)*Coefficient[NumJoint](i);
			if(i>=1)
				dq_dot += (i)*powf(TrajTime[NumJoint], i-1)*Coefficient[NumJoint](i);
			if(i>=2)
				dq_ddot += i*(i-1)*powf(TrajTime[NumJoint], i-2)*Coefficient[NumJoint](i);
		}
		return dq;

	}
	else
	{
		return 0;
	}

}
float Trajectory::Polynomial5th(int NumJoint, float CurrentTime, int *Flag, float *q_)
{
	if((CurrentTime - TrajInitTime[NumJoint]) >= (TrajDuration[NumJoint]))
	{
		//des->q(NumJoint) = StateVec[NumJoint](3);
		//des->q_dot(NumJoint) = StateVec[NumJoint](4);
		//des->q_ddot(NumJoint) = StateVec[NumJoint](5);
		q_[0]=StateVec[NumJoint](3);
		q_[1]=StateVec[NumJoint](4);
		q_[2]=StateVec[NumJoint](5);

		m_isReady[NumJoint] = 0;
		*Flag = 0;
		return StateVec[NumJoint](3);
	}

	if(m_isReady[NumJoint])
	{
		dq=0;
		dq_dot = 0;
		dq_ddot = 0;

		TrajTime[NumJoint] = CurrentTime - TrajInitTime[NumJoint];
		for(int i=0; i<6; ++i)
		{
			dq += powf(TrajTime[NumJoint], i)*Coefficient[NumJoint](i);
			if(i>=1)
				dq_dot += (i)*powf(TrajTime[NumJoint], i-1)*Coefficient[NumJoint](i);
			if(i>=2)
				dq_ddot += i*(i-1)*powf(TrajTime[NumJoint], i-2)*Coefficient[NumJoint](i);
		}

		q_[0]=dq;
		q_[1]=dq_dot;
		q_[2]=dq_ddot;
		return dq;

	}
	else
	{
		return 0;
	}

}


void Trajectory::SetPolynomial5th(int NumJoint, float startPos, float FinalPos, float InitTime, float Duration, float *q_)
{
    TrajDuration[NumJoint] = Duration;
    TrajInitTime[NumJoint] = InitTime;
    Coefficient[NumJoint].setZero();
    m_cof << 1.0, 0.0, 	0.0, 		0.0, 				0.0, 					0.0,
            0.0, 1.0, 	0.0, 		0.0, 				0.0, 					0.0,
            0.0, 0.0, 	2.0, 		0.0, 				0.0, 					0.0,
            1.0, powf(TrajDuration[NumJoint],1.0), 	powf(TrajDuration[NumJoint],2.0), 	powf(TrajDuration[NumJoint],3.0), 	powf(TrajDuration[NumJoint],4.0), 	powf(TrajDuration[NumJoint],5.0),
            0.0, 1.0, 									2.0*powf(TrajDuration[NumJoint],1.0), 	3.0*powf(TrajDuration[NumJoint],2.0), 	4.0*powf(TrajDuration[NumJoint],3.0), 	5.0*powf(TrajDuration[NumJoint],4.0),
            0.0, 0.0, 									2.0, 									6.0*powf(TrajDuration[NumJoint],1.0), 	12.0*powf(TrajDuration[NumJoint],2.0),	20.0*powf(TrajDuration[NumJoint],3.0);

    StateVec[NumJoint] << startPos,
            //act->j_q_d(NumJoint),
            //act->j_q_dd(NumJoint),
            0.0,
            0.0,
            FinalPos,
            0.0,
            0.0;
    q_[0]=StateVec[NumJoint](0);
    q_[1]=StateVec[NumJoint](1);
    q_[2]=StateVec[NumJoint](2);
    Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
    m_isReady[NumJoint] = 1;

}




} /* namespace HYUDA */


