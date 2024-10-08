
#include "../global_vars.h"

// Bullet task
static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

void bullet_run(void *arg)
{
	struct sockaddr_ipc addr_nom, addr_act;
	uint addr_nom_len = sizeof(addr_nom);
	uint addr_act_len = sizeof(addr_act);
	int socket_nom, socket_act;
	int ret_nom, ret_act;
	struct timespec ts;
	size_t poolsz;
	size_t BUFLEN = sizeof(packet::JointState);

	struct packet::JointState *bullet_nom_msg = (packet::JointState *)malloc(BUFLEN);
	struct packet::JointState *bullet_act_msg = (packet::JointState *)malloc(BUFLEN);
	
    rt_task_set_periodic(NULL, TM_NOW, 100*cycle_ns); // 100ms

	socket_nom = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_nom < 0) {
		perror("socket_nom");
		exit(EXIT_FAILURE);
	}
	socket_act = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_act < 0) {
		perror("socket_act");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	if( __cobalt_setsockopt(socket_nom, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_nom, 0, sizeof(addr_nom));
	addr_nom.sipc_family = AF_RTIPC;
	addr_nom.sipc_port = XDDP_PORT_SIM;

	if( __cobalt_setsockopt(socket_act, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_act, 0, sizeof(addr_act));
	addr_act.sipc_family = AF_RTIPC;
	addr_act.sipc_port = XDDP_PORT_ACT;

	if(__cobalt_bind(socket_nom, (struct sockaddr *)&addr_nom, sizeof(addr_nom)) == -1)
		fail("bind");
	if(__cobalt_bind(socket_act, (struct sockaddr *)&addr_act, sizeof(addr_act)) == -1)
		fail("bind");

    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle
		if(system_ready)
		{
			Quaterniond quaternion(cs_nom_indy7.getRMat());		
			for(int i=0; i<ROBOT_DOF; i++)
			{
				bullet_nom_msg->position[i] = info.nom.q(i);
				bullet_act_msg->position[i] = info.act.q(i);
			}
			ret_nom = __cobalt_sendto(socket_nom, bullet_nom_msg, BUFLEN, 0, (struct sockaddr *) &addr_nom, addr_nom_len);
			ret_act = __cobalt_sendto(socket_act, bullet_act_msg, BUFLEN, 0, (struct sockaddr *) &addr_act, addr_act_len);
		}
	}
	close(socket_nom);
	close(socket_act);

	return;
}