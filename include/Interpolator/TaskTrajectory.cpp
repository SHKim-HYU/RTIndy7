/*
 * TaskTrajectory.cpp
 *
 *  Created on: May 29, 2024
 *      Author: Sunhong Kim
 */

#include "TaskTrajectory.h"


Trajectory::Trajectory():dq(0), dq_dot(0), dq_ddot(0) {

}

Trajectory::~Trajectory() {

}

void Trajectory::SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration)
{
	TrajDuration[NumJoint] = Duration;
	TrajInitTime[NumJoint] = InitTime;
	Coefficient[NumJoint].setZero();
	m_cof << 1, 0, 	0, 		0, 				0, 					0,
			 0, 1, 	0, 		0, 				0, 					0,
			 0, 0, 	2, 		0, 				0, 					0,
			 1, pow(TrajDuration[NumJoint],1), 	pow(TrajDuration[NumJoint],2), 	pow(TrajDuration[NumJoint],3), 	pow(TrajDuration[NumJoint],4), 	pow(TrajDuration[NumJoint],5),
			 0, 1, 									2*pow(TrajDuration[NumJoint],1), 	3*pow(TrajDuration[NumJoint],2), 	4*pow(TrajDuration[NumJoint],3), 	5*pow(TrajDuration[NumJoint],4),
			 0, 0, 									2, 									6*pow(TrajDuration[NumJoint],1), 	12*pow(TrajDuration[NumJoint],2),	20*pow(TrajDuration[NumJoint],3);

	StateVec[NumJoint] << startPos,
						0,
						0,
						FinalPos,
						0,
						0;

	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
	m_isReady[NumJoint] = 1;

}

// void Trajectory::SetPolynomial5th_t(int NumJoint, state *act, double FinalPos, double InitTime, double Duration,double *q_)
// {
// 	TrajDuration[NumJoint] = Duration;
// 	TrajInitTime[NumJoint] = InitTime;
// 	m_cof << 1, 0, 	0, 		0, 				0, 					0,
// 			 0, 1, 	0, 		0, 				0, 					0,
// 			 0, 0, 	2, 		0, 				0, 					0,
// 			 1, pow(TrajDuration[NumJoint],1), 	pow(TrajDuration[NumJoint],2), 	pow(TrajDuration[NumJoint],3), 	pow(TrajDuration[NumJoint],4), 	pow(TrajDuration[NumJoint],5),
// 			 0, 1, 									2*pow(TrajDuration[NumJoint],1), 	3*pow(TrajDuration[NumJoint],2), 	4*pow(TrajDuration[NumJoint],3), 	5*pow(TrajDuration[NumJoint],4),
// 			 0, 0, 									2, 									6*pow(TrajDuration[NumJoint],1), 	12*pow(TrajDuration[NumJoint],2),	20*pow(TrajDuration[NumJoint],3);

// 	StateVec[NumJoint] << act->x[NumJoint],
// 			//act->j_q_d(NumJoint),
// 			//act->j_q_dd(NumJoint),
// 						0,
// 						0,
// 						FinalPos,
// 						0,
// 						0;
// 	q_[0]=StateVec[NumJoint](0);
// 	q_[1]=StateVec[NumJoint](1);
// 	q_[2]=StateVec[NumJoint](2);
// 	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
// 	m_isReady[NumJoint] = 1;

// }

// void Trajectory::SetPolynomial5th_j(int NumJoint, state *act, double FinalPos, double InitTime, double Duration,double *q_)
// {
// 	TrajDuration[NumJoint] = Duration;
// 	TrajInitTime[NumJoint] = InitTime;
// 	m_cof << 1, 0, 	0, 		0, 				0, 					0,
// 			 0, 1, 	0, 		0, 				0, 					0,
// 			 0, 0, 	2, 		0, 				0, 					0,
// 			 1, pow(TrajDuration[NumJoint],1), 	pow(TrajDuration[NumJoint],2), 	pow(TrajDuration[NumJoint],3), 	pow(TrajDuration[NumJoint],4), 	pow(TrajDuration[NumJoint],5),
// 			 0, 1, 									2*pow(TrajDuration[NumJoint],1), 	3*pow(TrajDuration[NumJoint],2), 	4*pow(TrajDuration[NumJoint],3), 	5*pow(TrajDuration[NumJoint],4),
// 			 0, 0, 									2, 									6*pow(TrajDuration[NumJoint],1), 	12*pow(TrajDuration[NumJoint],2),	20*pow(TrajDuration[NumJoint],3);

// 	StateVec[NumJoint] << act->q[NumJoint],
// 						act->q_dot[NumJoint],
// 						act->q_ddot[NumJoint],
// 						//0,
// 						//0,
// 						FinalPos,
// 						0,
// 						0;

// 	q_[0]=StateVec[NumJoint](0);
// 	q_[1]=StateVec[NumJoint](1);
// 	q_[2]=StateVec[NumJoint](2);
// 	Coefficient[NumJoint] = m_cof.inverse()*StateVec[NumJoint];
// 	m_isReady[NumJoint] = 1;

// }

double Trajectory::Polynomial5th(int NumJoint, double CurrentTime, int *Flag)
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
			dq += pow(TrajTime[NumJoint], i)*Coefficient[NumJoint](i);
			if(i>=1)
				dq_dot += (i)*pow(TrajTime[NumJoint], i-1)*Coefficient[NumJoint](i);
			if(i>=2)
				dq_ddot += i*(i-1)*pow(TrajTime[NumJoint], i-2)*Coefficient[NumJoint](i);
		}
		return dq;

	}
	else
	{
		return 0;
	}

}
double Trajectory::Polynomial5th(int NumJoint, double CurrentTime, int *Flag, double *q_)
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
			dq += pow(TrajTime[NumJoint], i)*Coefficient[NumJoint](i);
			if(i>=1)
				dq_dot += (i)*pow(TrajTime[NumJoint], i-1)*Coefficient[NumJoint](i);
			if(i>=2)
				dq_ddot += i*(i-1)*pow(TrajTime[NumJoint], i-2)*Coefficient[NumJoint](i);
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


void Trajectory::SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration, double *q_)
{
    TrajDuration[NumJoint] = Duration;
    TrajInitTime[NumJoint] = InitTime;
    Coefficient[NumJoint].setZero();
    m_cof << 1.0, 0.0, 	0.0, 		0.0, 				0.0, 					0.0,
            0.0, 1.0, 	0.0, 		0.0, 				0.0, 					0.0,
            0.0, 0.0, 	2.0, 		0.0, 				0.0, 					0.0,
            1.0, pow(TrajDuration[NumJoint],1.0), 	pow(TrajDuration[NumJoint],2.0), 	pow(TrajDuration[NumJoint],3.0), 	pow(TrajDuration[NumJoint],4.0), 	pow(TrajDuration[NumJoint],5.0),
            0.0, 1.0, 									2.0*pow(TrajDuration[NumJoint],1.0), 	3.0*pow(TrajDuration[NumJoint],2.0), 	4.0*pow(TrajDuration[NumJoint],3.0), 	5.0*pow(TrajDuration[NumJoint],4.0),
            0.0, 0.0, 									2.0, 									6.0*pow(TrajDuration[NumJoint],1.0), 	12.0*pow(TrajDuration[NumJoint],2.0),	20.0*pow(TrajDuration[NumJoint],3.0);

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


