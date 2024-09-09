#include "bullet_server.h"

void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void *bullet_run(void *arg)
{
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

	b3sim = new b3RobotSimulatorClientAPI();
    bool isConnected = b3sim->connect(eCONNECT_GUI);
    if (!isConnected)
    {
        printf("Cannot connect\n");
        return NULL;
    }

    b3sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	b3sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 1);
    b3sim->setNumSolverIterations(1);

    
    btVector3 targetPos(0,0,0);
    b3sim->resetDebugVisualizerCamera(2, -45, 135, targetPos);
    b3sim->setTimeOut(10);

    b3sim->loadURDF("/opt/bullet3/data/plane.urdf");
    int nomId = b3sim->loadURDF("/home/robot/robot_ws/temp/RTIndy7/description/indy7_dualarm.urdf");
    int actId = b3sim->loadURDF("/home/robot/robot_ws/temp/RTIndy7/description/indy7_dualarm.urdf");

    b3sim->setRealTimeSimulation(false);
    b3robot_nom = new Bullet_Indy7(b3sim,nomId);	
    b3robot_act = new Bullet_Indy7(b3sim,actId);	

    b3RobotSimulatorChangeVisualShapeArgs visual_opt;
    btVector4 rgba(0,1,0,0.3);
    int numJoint = b3sim->getNumJoints(nomId);
    for(int i=0; i<=numJoint; i++)
    {   
        visual_opt.m_objectUniqueId = nomId;
        visual_opt.m_linkIndex = i;
        visual_opt.m_hasRgbaColor = true;
        visual_opt.m_rgbaColor = rgba;
        b3sim->changeVisualShape(visual_opt);
    }

    JVec q_sim, q_act;
    q_sim = JVec::Zero();
    q_act = JVec::Zero();

    char *devname_nom, *devname_act;
	int fd, ret;

	if (asprintf(&devname_nom, "/dev/rtp%d", XDDP_PORT_SIM) < 0)
		fail("asprintf");

	sockfd_nom = open(devname_nom, O_RDWR);
	free(devname_nom);
	if (sockfd_nom < 0)
		fail("open");
    

    if (asprintf(&devname_act, "/dev/rtp%d", XDDP_PORT_ACT) < 0)
		fail("asprintf");

	sockfd_act = open(devname_act, O_RDWR);
	free(devname_act);
	if (sockfd_act < 0)
		fail("open");


	while(true) 
    {
        next_period.tv_nsec += 1*FIXED_TIMESTEP; //10ms
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
        /* Get the next message from realtime_thread. */
		ret = read(sockfd_nom, bullet_nom, BUFLEN_BULLET);
        Eigen::Map<JVec> q_sim(bullet_nom->position, ROBOT_DOF);

        ret = read(sockfd_act, bullet_act, BUFLEN_BULLET);
        Eigen::Map<JVec> q_act(bullet_act->position, ROBOT_DOF);	

        b3robot_nom->reset_q(q_sim);		
        b3robot_act->reset_q(q_act);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
	}

    b3sim->disconnect();

	return NULL;
}

int main(int argc, char **argv)
{
	sigset_t set;
	int sig;

	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGTERM);
	// sigaddset(&set, SIGHUP);
	// pthread_sigmask(SIG_BLOCK, &set, NULL);

    pthread_attr_init(&bullet_attr);
    pthread_attr_setschedpolicy(&bullet_attr, SCHED_FIFO );
    bullet_param.sched_priority = sched_get_priority_max(SCHED_FIFO )-1;
    pthread_attr_setschedparam(&bullet_attr, &bullet_param);
    pthread_create(&bullet_thread, &bullet_attr, bullet_run, NULL);
    
	sigwait(&set, &sig);
	pthread_cancel(bullet_thread);
	pthread_join(bullet_thread, NULL);

	return 0;
}
