#ifndef WAYPOINT_RUN_H
#define WAYPOINT_RUN_H


//waypoint
#include "lr_control_run.h"
#include "include/type.h"
#include <LR/include/liegroup_robotics.h>


using namespace lr;
void JointWayPointGenerator(std::vector<JVec>& way_points,std::vector<double>& delays,JVec q0){
    JVec RandJVec1=JVec::Zero();
    JVec RandJVec2=JVec::Zero();
    JVec RandJVec3=JVec::Zero();
    JVec RandJVec4=JVec::Zero();
    JVec RandJVec5=JVec::Zero();
    JVec RandJVec6=JVec::Zero();

    double min_range = -1.5708;
    double max_range = 1.5708;    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min_range, max_range);
    JVec q1,q2,q3;

    q1<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;
    q2<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;
    q3<<0.14558084, 0.813371 ,1.3075236,0.614656,1.03065,-0.0693011;

    way_points.push_back(q0);
    way_points.push_back(q0);
    way_points.push_back(q1);
    way_points.push_back(q2);
    way_points.push_back(q3);
    way_points.push_back(q1);

    delays.push_back(5.0);
    delays.push_back(5.0);
    delays.push_back(5.0);
    delays.push_back(5.0);
    delays.push_back(5.0);
    

};


void TaskWayPointGenerator(std::vector<SE3>& task_way_points,std::vector<double>& task_delays,const SE3 X0){
    double box_size = 0.05;
    task_way_points.push_back(X0);
    task_way_points.push_back(X0);
    Vector6d V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12;
    V1 <<box_size,box_size,-box_size,0.0,0.0,0.0;
    V2 <<-box_size,box_size,-box_size,0.0,0.0,0.0;
    V3 <<-box_size,-box_size,-box_size,0.0,0.0,0.0;
    V4 <<box_size,-box_size,-box_size,0.0,0.0,0.0;
    V5 <<box_size,-box_size,box_size,0.0,0.0,0.0;
    V6 <<box_size,box_size,box_size,0.0,0.0,0.0;
    V7 <<-box_size,box_size,box_size,0.0,0.0,0.0;
    V8 <<-box_size,-box_size,box_size,0.0,0.0,0.0;
    V9 <<0.0,0.0,0.0,1.5708,0.0,0.0;
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V1)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V2)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V3)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V4)));

    task_way_points.push_back(X0*MatrixExp6(VecTose3(V5)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V6)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V7)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V8)));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(V9)));
    task_way_points.push_back(X0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);

    
};
#endif // WAYPOINT_RUN_H
