/*! 
 *  @file PropertyDefinition.h
 *  @brief header for property definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */


#pragma once

#include <Eigen/Dense>

#define CONTROL_FREQ 1000
#define ROBOT_DOF 6
#define OFFSET_NUM 5
#define NRMK_DRIVE_NUM 6
#define NRMK_TOOL_NUM 1

// [General Parameters]
// 1. Motor
#define TORQUE_CONST_500 0.0884 		
#define TORQUE_CONST_200 0.087		
#define TORQUE_CONST_100 0.058		

#define TORQUE_ADC_500 48 // Torque ADC for Core 500 [NRMK]
#define TORQUE_ADC_200 96 // Torque ADC for Core 200 [NRMK]
#define TORQUE_ADC_100 96 // Torque ADC for Core 100 [NRMK]

// Indy7
// SN: DD11I7E0D001 [Indy7]
#define ZERO_POS_1 20902
#define ZERO_POS_2 28201
#define ZERO_POS_3 292649
#define ZERO_POS_4 61158
#define ZERO_POS_5 2180
#define ZERO_POS_6 31587

#define F_c 20.123, 12.287, 4.5622, 3.1492, 3.4757, 3.4986
#define F_v1 111.32, 70.081, 25.337, 13.131, 8.5082, 9.9523
#define F_v2 0.5193, 0.4824, 0.9098, 1.0961, 0.73829, 1.1475 

#define MAX_CURRENT_1 2.55
#define MAX_CURRENT_2 2.55
#define MAX_CURRENT_3 2.83
#define MAX_CURRENT_4 2.83
#define MAX_CURRENT_5 2.83
#define MAX_CURRENT_6 2.83

#define MAX_TORQUE_1 431.97
#define MAX_TORQUE_2 431.97
#define MAX_TORQUE_3 197.23
#define MAX_TORQUE_4 79.79
#define MAX_TORQUE_5 79.79
#define MAX_TORQUE_6 79.79

#define invL2sqr_1 1000
#define invL2sqr_2 1000
#define invL2sqr_3 800
#define invL2sqr_4 600
#define invL2sqr_5 600
#define invL2sqr_6 600


// 2. Electrical
#define ENC_2048 2048
#define ENC_1024 1024
#define ENC_1000 1000
#define ENC_512 512
#define ABS_ENC_19 524288

#define ENC_CORE_500 65536
#define ENC_CORE_200 65536
#define ENC_CORE_100 65536
#define ENC_CORE 65536

// 3. Mechanical
#define HARMONIC_120 120
#define HARMONIC_100 100
#define HARMONIC_50 50

#define GEAR_RATIO_121 121
#define GEAR_RATIO_101 101
#define GEAR_RATIO_50 50
#define GEAR_RATIO_18 18

#define WHEEL_RADIUS 0.076 // [m]
#define BASE_l 0.41 // [m]
#define BASE_w 0.31 // [m]

#define EFFICIENCY 85.0 // Gear efficiency

// Tool Information [Tip]
#define Ixx 0.00012118
#define Iyy 0.00012118
#define Izz 0.00000861
#define X_com 0.0
#define Y_com 0.0
#define Z_com 0.0355073
#define mass_tool 0.06
// Tool Information [FT Sensor]
#define mass_FT 0.02

// FT Sensor Commands
#define FT_START_DEVICE 	0x0000000B
#define FT_STOP_DEVICE 		0x0000000C

#define FT_SET_FILTER_500 	0x00010108
#define FT_SET_FILTER_300 	0x00020108
#define FT_SET_FILTER_200 	0x00030108
#define FT_SET_FILTER_150 	0x00040108
#define FT_SET_FILTER_100 	0x00050108
#define FT_SET_FILTER_50 	0x00060108
#define FT_SET_FILTER_40 	0x00070108
#define FT_SET_FILTER_30 	0x00080108
#define FT_SET_FILTER_20 	0x00090108
#define FT_SET_FILTER_10 	0x000A0108

#define FT_SET_BIAS 		0x00000111
#define FT_UNSET_BIAS 		0x00000011

// 4. Type Definition
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 4, 4> se3;
typedef Eigen::Matrix<double, 3, 3> so3;
typedef Eigen::Matrix<double, 6, 1> Twist;
typedef Eigen::Matrix<double, 6, 1> Vector6d;   
typedef Eigen::Matrix<float, 6, 1> Vector6f;  
typedef Eigen::Matrix<double, 3, 1> Vector3d;   
typedef Eigen::Matrix<double, 4, 1> Vector4d;  
typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
typedef Eigen::Matrix<double, 3, 3> Matrix3d;  

typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, 1> JVec;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> JMat;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> ScrewList;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> Jacobian;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM,6 > pinvJacobian;
typedef Eigen::Matrix<double, 6*NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> DerivativeJacobianVec;
typedef Eigen::Matrix<double, 6*NRMK_DRIVE_NUM, 1> vecJVec;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> Matrixnd;  
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> Matrix6xn;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM+1> Matrix6xn_1;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> MassMat;


// Robot Struct
typedef struct STATE{
	JVec q;			// x, y, th, q1, q2, q3, q4, q5, q6
	JVec q_dot;		// vx, vy, wz, dq1, dq2, dq3, dq4, dq5, dq6
	JVec q_ddot;
	JVec tau;		// Fx, Fy, Tz, tau1, tau2, tau3, tau4, tau5, tau6
	JVec tau_fric;
	JVec tau_ext;
	JVec tau_aux;
	JVec e;
	JVec eint;
	JVec edot;

	SE3 	 T;                           //Task space
	SO3		 R;
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;
    
    double s_time;
}mm_state;

typedef struct ROBOT_INFO{

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;
	unsigned int idx;

	STATE act;
	STATE des;
	STATE nom;
	STATE sim;

}ROBOT_INFO;
