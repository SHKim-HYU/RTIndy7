#ifndef ECAT_RUN_H
#define ECAT_RUN_H

#include "../main.h"

//ECAT

void* rt_ecat_run(void* param);
void readEcatData();
void writeEcatData();
int initAxes();
void rt_ecat_setup_start(RTIME& beginCycle,RTIME& endCycle,RTIME& beginRead, RTIME& beginReadbuf, RTIME& beginWrite, RTIME &beginWritebuf,RTIME& beginCompute, int& ft_init_cnt);
void rt_ecat_setup_end(RTIME& beginCycle,RTIME& endCycle,RTIME& beginRead, RTIME& beginReadbuf, RTIME& beginWrite, RTIME &beginWritebuf,RTIME& beginCompute, int& ft_init_cnt);
#endif // ECAT_RUN_H