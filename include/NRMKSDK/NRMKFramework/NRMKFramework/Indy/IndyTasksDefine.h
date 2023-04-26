/*
 * IndyTasksDefine.h
 *
 *  Created on: September 20, 2018
 *      Author: Hanter Jung
 */

#ifndef INDYTASKSDEFINE_H
#define INDYTASKSDEFINE_H

#pragma once

#if defined (__INDY7__)
#define INDY_SYS_ROBOT_NAME "NRMK-Indy7"
#elif defined(__INDY15__)
#define INDY_SYS_ROBOT_NAME "NRMK-Indy15"
#elif defined(__INDY3__)
#define INDY_SYS_ROBOT_NAME "NRMK-Indy3"
#elif defined(__INDY5__)
#define INDY_SYS_ROBOT_NAME "NRMK-Indy5"
#elif defined(__INDY10__)
#define INDY_SYS_ROBOT_NAME "NRMK-Indy10"
#elif defined(__INDYRP__)
#define INDY_SYS_ROBOT_NAME "NRMK-IndyRP"
#elif defined(__INDYRP2__)
#define INDY_SYS_ROBOT_NAME "NRMK-IndyRP2"
#elif defined(__OPTI5__)
#define INDY_SYS_ROBOT_NAME "NRMK-Opti5"
#elif defined(__OPTI10__)
#define INDY_SYS_ROBOT_NAME "NRMK-Opti10"
#elif defined(__INDYP1__)
#define INDY_SYS_ROBOT_NAME "NRMK-IndyP1"
#endif


#if defined(_JOINT_6DOF)
#define INDY_SYS_ROBOT_DOF_STR "6DOF"
#elif defined (_JOINT_7DOF)
#define INDY_SYS_ROBOT_DOF_STR "7DOF"
#endif

#endif /* INDYTASKSDEFINE_H */
