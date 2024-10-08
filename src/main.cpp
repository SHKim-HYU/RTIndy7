#include "main.h"


CS_Indy7 cs_indy7;
CS_Indy7 cs_nom_indy7;

RT_TASK safety_task;
RT_TASK motor_task;
RT_TASK bullet_task;
RT_TASK print_task;
RT_TASK ft_task;

using namespace std;
using namespace lr;


unsigned int cycle_ns = 1000000; // 1 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
int system_ready = 0;
unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodLoop = 0, worstLoop = 0;
unsigned int overruns = 0;
double gt=0;
ROBOT_INFO info;
NRMKHelper::ServoAxis_Core Axis[NRMK_DRIVE_NUM];
Master ecat_master;
EcatNRMK_Drive ecat_drive[NRMK_DRIVE_NUM];
EcatNRMK_Tool ecat_tool[NRMK_TOOL_NUM];
Twist F_tmp;
double manipulability;

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

	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);

    rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &rt_safety_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &rt_motor_run, NULL);

	rt_task_create(&bullet_task, "bullet_task", 0, 60, 0);
    // rt_task_set_affinity(&bullet_task, &cpuset_rt1);
    rt_task_start(&bullet_task, &bullet_run, NULL);

    rt_task_create(&print_task, "print_task", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt1);
    rt_task_start(&print_task, &rt_print_run, NULL);

	rt_task_create(&ft_task, "ft_task", 0, 90, 0);
    rt_task_set_affinity(&ft_task, &cpuset_rt1);
    rt_task_start(&ft_task, &rt_ft_run, NULL);
	
    // Must pause here
    sigset_t set;
    int sig, ret;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    ret = sigwait(&set, &sig);
    // Finalize
    signal_handler(0);

    return 0;
}

