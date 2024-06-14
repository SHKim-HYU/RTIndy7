/*
 * main.cpp
 *
 *  Created on: 2023. 11. 26.
 *      Author: Minchang Sung
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "main.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <alchemy/task.h>
#include <alchemy/timer.h>

class RTNode : public rclcpp::Node
{
public:
    RTNode() : Node("rt_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        rt_task_create(&task_, "RT_Task", 0, 99, T_JOINABLE);
        rt_task_start(&task_, &RTNode::run, this);
    }

private:
    static void run(void *arg)
    {
        auto node = static_cast<RTNode*>(arg);
        rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(1e6)); // 1ms period (1e6 ns)
        
        sensor_msgs::msg::JointState joint_state;
        joint_state.name = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};
        joint_state.position.resize(6);

        while (rclcpp::ok())
        {
            for (size_t i = 0; i < joint_state.position.size(); ++i) {
                joint_state.position[i] = info.act.q(i);
            }

            joint_state.header.stamp = node->now();
            node->publisher_->publish(joint_state);
            rt_task_wait_period(NULL); // Wait for the next period
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    RT_TASK task_;
};



RT_TASK print_task;
RT_TASK control_task;
RT_TASK safety_task;
RT_TASK ft_task;

NRMK_Master nrmk_master;
NRMKHelper::ServoAxis Axis[NUM_AXIS];

UINT16	StatusWord[NUM_SLAVES] = {0,};
INT32 	ActualPos[NUM_SLAVES] = {0,};
INT32 	ActualVel[NUM_SLAVES] = {0,};
INT16 	ActualTor[NUM_SLAVES] = {0,};
UINT32	DataIn[NUM_SLAVES] = {0,};
UINT8	ModeOfOperationDisplay[NUM_SLAVES] = {0,};
// Tool
UINT8	IStatus[NUM_SLAVES] = {0,};
UINT8	IButton[NUM_SLAVES] = {0,};
INT16	FTRawFx[NUM_SLAVES] = {0,};
INT16	FTRawFy[NUM_SLAVES] = {0,};
INT16	FTRawFz[NUM_SLAVES] = {0,};
INT16	FTRawTx[NUM_SLAVES] = {0,};
INT16	FTRawTy[NUM_SLAVES] = {0,};
INT16	FTRawTz[NUM_SLAVES] = {0,};
UINT8	FTOverloadStatus[NUM_SLAVES] = {0,};
UINT8	FTErrorFlag[NUM_SLAVES] = {0,};
// Tool
UINT8	ILed[NUM_SLAVES] = {0,};
UINT8	IGripper[NUM_SLAVES] = {0,};
UINT32	FTConfigParam[NUM_SLAVES] = {0,};
UINT8	LEDMode[NUM_SLAVES] = {0,};
UINT8	LEDG[NUM_SLAVES] = {0,};
UINT8	LEDR[NUM_SLAVES] = {0,};
UINT8	LEDB[NUM_SLAVES] = {0,};

////////////// RxPDO //////////////
// Drive
INT32 	TargetPos[NUM_SLAVES] = {0,};
INT32 	TargetVel[NUM_SLAVES] = {0,};
INT16 	TargetTor[NUM_SLAVES] = {0,};
UINT8 	ModeOfOperation[NUM_SLAVES] = {0,};
UINT16	Controlword[NUM_SLAVES] = {0,};
#ifdef __CB__

UINT8	StatusCode[NUM_SLAVES] = {0,};
UINT8	DI5V[NUM_SLAVES] = {0,};
UINT8	DI1[NUM_SLAVES] = {0,};
UINT8	DI2[NUM_SLAVES] = {0,};
UINT16	AI1[NUM_SLAVES] = {0,};
UINT16	AI2[NUM_SLAVES] = {0,};
INT16	FTRawFxCB[NUM_SLAVES] = {0,};
INT16	FTRawFyCB[NUM_SLAVES] = {0,};
INT16	FTRawFzCB[NUM_SLAVES] = {0,};
INT16	FTRawTxCB[NUM_SLAVES] = {0,};
INT16	FTRawTyCB[NUM_SLAVES] = {0,};
INT16	FTRawTzCB[NUM_SLAVES] = {0,};
UINT8	FTOverloadStatusCB[NUM_SLAVES] = {0,};
UINT8	FTErrorFlagCB[NUM_SLAVES] = {0,};
UINT8	RS485RxCnt[NUM_SLAVES] = {0,};
UINT8	RS485RxD0[NUM_SLAVES] = {0,};
UINT8	RS485RxD1[NUM_SLAVES] = {0,};
UINT8	RS485RxD2[NUM_SLAVES] = {0,};
UINT8	RS485RxD3[NUM_SLAVES] = {0,};
UINT8	RS485RxD4[NUM_SLAVES] = {0,};
UINT8	RS485RxD5[NUM_SLAVES] = {0,};
UINT8	RS485RxD6[NUM_SLAVES] = {0,};
UINT8	RS485RxD7[NUM_SLAVES] = {0,};
UINT8	RS485RxD8[NUM_SLAVES] = {0,};
UINT8	RS485RxD9[NUM_SLAVES] = {0,};

UINT8	ControlCode[NUM_SLAVES] = {0,};
UINT8	DO5V[NUM_SLAVES] = {0,};
UINT8	TO[NUM_SLAVES] = {0,};
UINT8	DO[NUM_SLAVES] = {0,};
UINT16	AO1[NUM_SLAVES] = {0,};
UINT16	AO2[NUM_SLAVES] = {0,};
UINT32	FTConfigParamCB[NUM_SLAVES] = {0,};
UINT8	RS485ConfigParam[NUM_SLAVES] = {0,};
UINT8	RS485CMD[NUM_SLAVES] = {0,};
UINT8	RS485TxCnt[NUM_SLAVES] = {0,};
UINT8	RS485TxD0[NUM_SLAVES] = {0,};
UINT8	RS485TxD1[NUM_SLAVES] = {0,};
UINT8	RS485TxD2[NUM_SLAVES] = {0,};
UINT8	RS485TxD3[NUM_SLAVES] = {0,};
UINT8	RS485TxD4[NUM_SLAVES] = {0,};
UINT8	RS485TxD5[NUM_SLAVES] = {0,};
UINT8	RS485TxD6[NUM_SLAVES] = {0,};
UINT8	RS485TxD7[NUM_SLAVES] = {0,};
UINT8	RS485TxD8[NUM_SLAVES] = {0,};
UINT8	RS485TxD9[NUM_SLAVES] = {0,};
#endif

#ifdef __CASADI__
// #include <dlfcn.h>

// typedef long long int casadi_int;
// typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

unsigned long periodIndysim =0;

CS_Indy7 cs_indy7;
CS_Indy7 cs_sim_indy7;
#endif

#ifdef __LR__
LR_Control lr_control;
#endif
JOINT_INFO info;
double period = 0;
double gt = 0;
bool run=0;

bool system_ready = 0;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodCompute = 0, worstCompute = 0;
unsigned long periodEcat = 0, worstEcat = 0;
unsigned long periodBuffer = 0, worstBuffer = 0;
unsigned int overruns = 0;


/****************************************************************************/
void signal_handler(int signum) {
	// rt_task_delete(&RTIndy7_task);
	// rt_task_delete(&safety_task);
	rt_task_delete(&print_task);    
	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
        printf("║                Servo drives Stopped!               ║\n");
        printf("╚════════════════════════════════════════════════════╝\n");	
        
    nrmk_master.deinit();
    exit(1);
}
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);
	mlockall(MCL_CURRENT|MCL_FUTURE);




	// For CST (cyclic synchronous torque) control
	if (nrmk_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, CYCLE_NS) == -1)
	{
		printf("System Initialization Failed\n");
	    return 0;
	}
	for (int i = 0; i < NUM_AXIS+1; ++i)
		ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;

	// For trajectory interpolation
	initAxes();
	for(int i=0;i<NUM_SLAVES;i++)
		nrmk_master.setServoOn(i);
    run =1;
	period=((double) CYCLE_NS)/((double) NSEC_PER_SEC);	//period in second unit

#ifdef __CASADI__

    cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json");
    
    rt_task_create(&control_task, "rt_control_task", 0, 98, 0);
    rt_task_start(&control_task, &rt_control_run, NULL);    
    //RT task start
#else if __LR__

    lr_control=LR_Control();
	const char* urdfFilePath = "/home/robot/ros_ws/colcon_ws/src/indy7_ros2_description/indy7/indy7.urdf";
   Vector3d g = Vector3d::Zero();
    g(0) = 0;
    g(1) =0;
    g(2) = -9.87;	
	lr_control.LRSetup(urdfFilePath,g);    	

    
    rt_task_create(&control_task, "rt_lr_control_run", 0, 98, 0);
    rt_task_start(&control_task, &rt_lr_control_run, NULL);    
#endif

    rt_task_create(&print_task, "rt_print_task", 0, 5, 0);
    rt_task_start(&print_task, &rt_print_run, NULL);

    rt_task_create(&safety_task, "rt_safety_task", 0, 98, 0);
    rt_task_start(&safety_task, &rt_safety_run, NULL);


    // rt_task_create(&ft_task, "rt_ft_task", 0, 5, 0);
    // rt_task_start(&ft_task, &rt_ft_run, NULL);


    //qt thread
    #ifdef __QT__
    pthread_t qt_thread;
    pthread_attr_t qt_attr;
    struct sched_param qt_param;
    if(1){
        pthread_attr_init(&qt_attr);
        pthread_attr_setschedpolicy(&qt_attr, SCHED_RR );
        qt_param.sched_priority = sched_get_priority_max(SCHED_RR )-50;
        pthread_attr_setschedparam(&qt_attr, &qt_param);
        pthread_create(&qt_thread, &qt_attr, qt_run, NULL);
        //set_thread_affinity(qt_thread, 2); // 2번 코어에 할당
    }    
    //RT task END
    #endif



    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTNode>());
    rclcpp::shutdown();

	sigset_t set;
	int sig,ret;
	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGTERM);
	sigaddset(&set, SIGHUP);
	pthread_sigmask(SIG_BLOCK, &set, NULL);

	ret = sigwait(&set, &sig);
    signal_handler(sig);

    return 0;
}



