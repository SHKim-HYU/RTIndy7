#include "bullet_run.h"
#include <RobotSimulator/b3RobotSimulatorClientAPI.h>
#include <SimRobot/Robot.h>
b3RobotSimulatorClientAPI *sim;
Robot *robot,*left_arm,*right_arm;
Robot *left_arm_des,*right_arm_des;
void* bullet_run(void* param) {
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    //sdplog setup
    
    sim = new b3RobotSimulatorClientAPI();
    bool isConnected;
    isConnected = sim->connect(eCONNECT_GUI);
    if (isConnected)
    {
        printf("\n\n\nBullet Connected!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n");
    }    
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);
    btVector3 targetPos(0,0,0);
    sim->resetDebugVisualizerCamera(3.5, -45, 135, targetPos);    

    const char* urdfFilePath_body = "../urdf/indy7/body.urdf";
    const char* urdfFilePath = "../urdf/indy7/indy7.urdf";
    const char* urdfFilePath_visual = "../urdf/indy7/indy7_visual.urdf";
    b3RobotSimulatorLoadUrdfFileArgs urdf_args_left;
    b3RobotSimulatorLoadUrdfFileArgs urdf_args_right;

    btVector3 startPos(0 ,0.15634 ,0.37722);
    btQuaternion startOrn(-0.4999999, 0, 0, 0.8660255);
    urdf_args_right.m_startPosition = startPos;
    urdf_args_right.m_startOrientation = startOrn;
    btVector3 startPos2(0 ,-0.15634 ,0.37722);
    btQuaternion startOrn2(0, -0.4999999, 0.8660255, 0);
    urdf_args_left.m_startPosition = startPos2;
    urdf_args_left.m_startOrientation = startOrn2;
    int robotId_right_visual = sim->loadURDF(urdfFilePath_visual,urdf_args_right);
    int robotId_left_visual = sim->loadURDF(urdfFilePath_visual,urdf_args_left);
    urdf_args_right.m_flags |= URDF_USE_INERTIA_FROM_FILE;
    urdf_args_right.m_flags |= URDF_USE_SELF_COLLISION;
    urdf_args_left.m_flags |= URDF_USE_INERTIA_FROM_FILE;
    urdf_args_left.m_flags |= URDF_USE_SELF_COLLISION;
    int robotId_right = sim->loadURDF(urdfFilePath,urdf_args_right);
    int robotId_left = sim->loadURDF(urdfFilePath,urdf_args_left);
    int bodyId = sim->loadURDF(urdfFilePath_body);

    int planeId = sim->loadURDF("../urdf/indy7/plane.urdf");

    //std::cout<<"\n\n\n\nrobotId : "<< robotId<<"\n\n\n\n"<<std::endl; 


    sim->setRealTimeSimulation(false);    
    //sim->setTimeStep(0.1);
    left_arm = new Robot(sim,robotId_left);
     right_arm = new  Robot(sim,robotId_right);
     left_arm_des =new  Robot(sim,robotId_left_visual);
     right_arm_des =new  Robot(sim,robotId_right_visual);

//    usleep(10000000);
    int draw_id;
    b3RobotSimulatorAddUserDebugLineArgs line_args;
	line_args.m_colorRGB[0]=0;
	line_args.m_colorRGB[1]=0;
	line_args.m_colorRGB[2]=0;    
    line_args.m_lifeTime = 1.0/59.0;
    line_args.m_lineWidth = 2;
    line_args.m_parentObjectUniqueId = robotId_left;
    line_args.m_parentLinkIndex = 7;
    while (run) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS*33.33333;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }

        // Vector4d p =Vector4d::Zero();
        // p(3) = 1;
        // Vector4d p2 =Vector4d::Zero();

        // p2(0) = info.act.F_ext[0]/100.0;
        // p2(1) = info.act.F_ext[1]/100.0;
        // p2(2) = info.act.F_ext[2]/100.0;
        // p2(3) = 1;
	    // btVector3 fromXYZ(p(0),p(1),p(2));
        // btVector3 toXYZ(p2(0),p2(1),p2(2));
        // draw_id=sim->addUserDebugLine(fromXYZ,toXYZ,line_args);



        left_arm_des->reset_q(info.des.q_l);
        right_arm_des->reset_q(info.des.q_r);
        left_arm->reset_q(info.act.q_l);
        right_arm->reset_q(info.act.q_r);
        //sim->stepSimulation();
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

