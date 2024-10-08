#ifndef RTINDY7CLIENT_H_
#define RTINDY7CLIENT_H_

#include "global_vars.h"

// RT TASKS 
void rt_print_run(void* param);
void rt_safety_run(void *arg);
void rt_ft_run(void* param);
void bullet_run(void *arg);
void rt_motor_run(void *arg);
void control();
void compute();
//



#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */
#define NSEC_PER_SEC 			1000000000
// double ft_offset[6] = {27.90, -56.3, -46.7, -1.269, -0.099, 4.237};

// For RT thread management
static int run = 1;
double act_max[4] = {0.0,};

// Interface to physical axes


// Robotous FT EtherCAT
union DeviceConfig
{
	uint8_t u8Param[4];
	uint32_t u32Param;
};


// EtherCAT System interface object


// When all slaves or drives reach OP mode,
// system_ready becomes 1.


// Global time (beginning from zero)
double gt_offset = 0;

// Trajectory parameers
double traj_time=0;
int motion=-1;
int modeControl = 0;
int motioncnt = 0;

Vector3d x_offset;
SO3 R_offset;

// Controller Gains
JVec NRIC_Kp;
JVec NRIC_Ki;
JVec NRIC_K;
JVec NRIC_gamma;

JVec Kp_n;
JVec Kd_n;
JVec K_n;

Twist Task_Kp;
Twist Task_Kv;
Twist Task_Ki;
JVec Task_K;

Matrix6d A_, D_, K_;

JVec des_int;






#endif  // /* RTINDY7CLIENT_H_ */