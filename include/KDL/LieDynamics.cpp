#include "LieDynamics.h"


// how to use
// 1. update_info
// 2. Prepare_Dynamics
// 3. M_Matrix, C_Matrix, G_Matrix

namespace HYUMotionDynamics{

Liedynamics::Liedynamics():isFirstRun(0)
{
	i.setIdentity();
}
Liedynamics::Liedynamics(HYUMotionKinematics::PoEKinematics &_PoEKin, HYUMotionKinematics::PoEKinematics &_PoECoM):isFirstRun(0)
{
    i.setIdentity();
    pLink = &_PoEKin;
    pCoM = &_PoECoM;
}


Liedynamics::~Liedynamics()
{
}

void Liedynamics::UpdateDynamicInfo(Matrix3f inertia, float mass, int link_num)
{
	GIner[link_num] = GeneralizedInertia(inertia, mass);
}



Matrix6f Liedynamics::GeneralizedInertia(Matrix3f inertia, float mass) //MR P.288
{
	Matrix6f G_iner;
	G_iner.setZero();
	G_iner.block<3, 3>(0, 0) = inertia;
	G_iner.block<3, 3>(3, 3) = mass*i;
	return G_iner;
}


Matrix6n6nf Liedynamics::Inertia_Link(void)
{
	Matrix6n6nf iner_link = Matrix6n6nf::Zero();
	for (int i = 0; i < ROBOT_DOF; ++i)
	{
		iner_link.block<6, 6>(6 * i, 6 * i) = GIner[i+1];
	}
	return iner_link;
}

Matrix6n6nf Liedynamics::Gamma_Link(void)
{
	Matrix6n6nf res = Matrix6n6nf::Zero();
	for (int i = 1; i < ROBOT_DOF; ++i)
	{
		res.block<6, 6>(6 * i, 6 * (i - 1)) = LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pCoM->GetTMat(i,i+1)));
	}
	return res;
}

Matrix6n6nf Liedynamics::L_link(void)
{
	Matrix6n6nf res = Matrix6n6nf::Identity();

	for (int i = 1; i < ROBOT_DOF; ++i)
	{
		for(int j=i+1; j<= ROBOT_DOF; ++j)
		{
			res.block<6, 6>(6*(j-1) , 6*(i-1)) = LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pCoM->GetTMat(i,j)));
		}
	}
	//res=(res-Gamma_mat).inverse();

	return res;
}

Matrix6nnf Liedynamics::A_Link(void)
{
	Matrix6nnf res = Matrix6nnf::Zero();

	for (int i = 0; i < ROBOT_DOF; ++i)
	{
		res.block<6, 1>(6 * i,  i) = pCoM-> A[i+1];
	}
	return res;
}

Matrix6n6nf Liedynamics::ad_Aqdot_Link(float qdot[])
{
	Matrix6n6nf res = Matrix6n6nf::Zero();
	for (int i = 0; i < ROBOT_DOF; ++i)
	{
		res.block<6, 6>(6 * i, 6 * i) = LieOperator::adjointMatrix(pCoM->A[i+1]*qdot[i]);
	}
	return res;
}

Matrix6n6nf Liedynamics::ad_V_Link(float qdot[])
{
	Matrix6n6nf res;
	res.setZero();
//	Matrix<float,ROBOT_DOF,1> q_dot;

	q_d = Map<Jointd>(qdot,ROBOT_DOF);

	V = L_mat*A_mat*q_d;

	for (int i = 0; i < ROBOT_DOF; ++i)
	{
		res.block<6, 6>(6 * i, 6 * i) = LieOperator::adjointMatrix(V.segment(6 * i, 6));
	}

	return res;
}

Vector6nf Liedynamics::Vdot_base(int axis)
{
	Vector6nf res = Vector6nf::Zero();
	Vector6f res1 = Vector6f::Zero(); //twist of base


	switch (axis)
	{
	case _RotX:
		res1 << 0, 0, 0, gravity, 0, 0;
		res.head(6) = LieOperator::AdjointMatrix(inverse_SE3(pCoM->GetTMat(0,1)))*res1;   //MR P.90
		break;
	case _RotY:
		res1 << 0, 0, 0, 0, gravity, 0;
		res.head(6) = LieOperator::AdjointMatrix(inverse_SE3(pCoM->GetTMat(0,1)))*res1;
		break;
	case _RotZ:
		res1 << 0, 0, 0, 0, 0, gravity;
		res.head(6) = LieOperator::AdjointMatrix(inverse_SE3(pCoM->GetTMat(0,1)))*res1;
		break;
	default:
		break;
	}	
	return res;
}

void Liedynamics::Prepare_Dynamics(float q[], float qdot[])
{
	if(isFirstRun == 0)
	{
		Iner_mat = Inertia_Link();
		A_mat = A_Link();
		isFirstRun = 1;
	}

    pCoM->HTransMatrix(q);
	Gamma_mat = Gamma_Link();
	L_mat = L_link();
	ad_Aqd = ad_Aqdot_Link(qdot);
	ad_V = ad_V_Link(qdot);
	LA_mat = L_mat*A_mat;
}

Matrixf Liedynamics::M_Matrix(void)
{
    M_verify=LA_mat.transpose()*Iner_mat*LA_mat;
	return M_verify;
}

Matrixf Liedynamics::C_Matrix(void)
{
	C_verify = -LA_mat.transpose()*(Iner_mat*L_mat*ad_Aqd*Gamma_mat + ad_V.transpose() * Iner_mat)*LA_mat;
	return C_verify;
}
Matrixf Liedynamics::C_out(void)
{
    return C_verify;
}
Jointd Liedynamics::G_Matrix(void)
{
	G_verify = LA_mat.transpose()*Iner_mat*L_mat*Vdot_base(_RotZ);
	return G_verify;
}
void Liedynamics::Mdot_Matrix( MatrixXf &_Mdot )
{
    _Mdot.resize(ROBOT_DOF, ROBOT_DOF);
    _Mdot.setZero();
    _Mdot.noalias() += -LA_mat.transpose()*Gamma_mat.transpose()*ad_Aqd.transpose()*L_mat.transpose()*Iner_mat*LA_mat;
    _Mdot.noalias() += -LA_mat.transpose()*Iner_mat*L_mat*ad_Aqd*Gamma_mat*LA_mat;
    return;
}
}//end namespace
