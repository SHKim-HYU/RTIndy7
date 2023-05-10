#ifndef NRMKINDY_CONSTANTINDY_H_
#define NRMKINDY_CONSTANTINDY_H_

#pragma once

/*** Workspace ***/
#if defined (__INDYRP__)
#define INDY_ROBOT_NAME "NRMK-IndyRP"
#define INDY_HOME_POS_X  0.415519
#define INDY_HOME_POS_Y  -0.152895
#define INDY_HOME_POS_Z  0.576556

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.06
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.06

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.06
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.06

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.035	//From Gear of Last Body(Target Body)
#define INDY_FT_Z_DISP 0.0675	//From TCP
#define INDY_TASK_SPACE_DISTANCE 0.85

#elif defined (__INDYRP2__)
#define INDY_ROBOT_NAME "NRMK-IndyRP2"
#define INDY_HOME_POS_X  -0.565628
#define INDY_HOME_POS_Y  -0.000639
#define INDY_HOME_POS_Z  0.051074

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.1
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.1

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.1
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.1

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.060
#define INDY_FT_Z_DISP 0.05
#define INDY_TASK_SPACE_DISTANCE 1.0

#elif defined (__INDY7__)
#define INDY_ROBOT_NAME "NRMK-Indy7"
#define INDY_HOME_POS_X  -0.565628
#define INDY_HOME_POS_Y  -0.000639
#define INDY_HOME_POS_Z  0.051074

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.1
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.1

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.1
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.1

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.061
#define INDY_FT_Z_DISP 0.0675
#define INDY_TASK_SPACE_DISTANCE 1.0

#elif defined (__OPTI5__)
#define INDY_ROBOT_NAME "NRMK-Opti5"
#define INDY_HOME_POS_X  0.415519
#define INDY_HOME_POS_Y  -0.152895
#define INDY_HOME_POS_Z  0.576556

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.15
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.15

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.15
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.15

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.5
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.25

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.060
#define INDY_FT_Z_DISP 0.05
#define INDY_TASK_SPACE_DISTANCE 1.0

#elif defined (__INDY3__)
#define INDY_ROBOT_NAME "NRMK-Indy3"
#define INDY_HOME_POS_X  -0.565628
#define INDY_HOME_POS_Y  -0.000639
#define INDY_HOME_POS_Z  0.051074

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.1
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.1

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.1
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.1

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.077
#define INDY_FT_Z_DISP 0.05
#define INDY_TASK_SPACE_DISTANCE 0.7

#elif defined (__INDY5__)
#define INDY_ROBOT_NAME "NRMK-Indy5"
#define INDY_HOME_POS_X  -0.565628
#define INDY_HOME_POS_Y  -0.000639
#define INDY_HOME_POS_Z  0.051074

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.1
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.1

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.1
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.1

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.077
#define INDY_FT_Z_DISP 0.05
#define INDY_TASK_SPACE_DISTANCE 0.85

#elif defined (__INDY10__)
#define INDY_ROBOT_NAME "NRMK-Indy10"
#define INDY_HOME_POS_X  -0.565628
#define INDY_HOME_POS_Y  -0.000639
#define INDY_HOME_POS_Z  0.051074

#define INDY_HOME_POS_DELT_X_MAX INDY_HOME_POS_X + 0.1
#define INDY_HOME_POS_DELT_X_MIN INDY_HOME_POS_X - 0.1

#define INDY_HOME_POS_DELT_Y_MAX INDY_HOME_POS_Y + 0.1
#define INDY_HOME_POS_DELT_Y_MIN INDY_HOME_POS_Y - 0.1

#define INDY_HOME_POS_DELT_Z_MAX INDY_HOME_POS_Z + 0.05
#define INDY_HOME_POS_DELT_Z_MIN INDY_HOME_POS_Z - 0.05

//FIXME change to correct values
#define INDY_TCP_Z_DISP 0.084
#define INDY_FT_Z_DISP 0.05
#define INDY_TASK_SPACE_DISTANCE 1.0
#endif /* __INDY10__ */

/*** SW Limit (-0.5~-1.0 diff for HW Limit) ***/
#if defined (__INDYRP__)
#define INDY_JOINT0_POSLIMIT_MAX 					130
#define INDY_JOINT0_POSLIMIT_MIN 					-130
#define INDY_JOINT1_POSLIMIT_MAX 					150
#define INDY_JOINT1_POSLIMIT_MIN 					-60
#define INDY_JOINT2_POSLIMIT_MAX 					80
#define INDY_JOINT2_POSLIMIT_MIN 					-100
#define INDY_JOINT3_POSLIMIT_MAX 					180
#define INDY_JOINT3_POSLIMIT_MIN 					-90
#define INDY_JOINT4_POSLIMIT_MAX 					170
#define INDY_JOINT4_POSLIMIT_MIN 					-170
#define INDY_JOINT5_POSLIMIT_MAX 					200
#define INDY_JOINT5_POSLIMIT_MIN 					-200
#define INDY_JOINT6_POSLIMIT_MAX 					0
#define INDY_JOINT6_POSLIMIT_MIN 					0
#define INDY_JOINT0_VELLIMIT 						179
#define INDY_JOINT1_VELLIMIT 						179
#define INDY_JOINT2_VELLIMIT 						179
#define INDY_JOINT3_VELLIMIT 						179
#define INDY_JOINT4_VELLIMIT 						179
#define INDY_JOINT5_VELLIMIT	 					179
#define INDY_JOINT6_VELLIMIT	 					0
#define INDY_JOINT0_VELLIMIT_DT						90
#define INDY_JOINT1_VELLIMIT_DT						90
#define INDY_JOINT2_VELLIMIT_DT						90
#define INDY_JOINT3_VELLIMIT_DT						90
#define INDY_JOINT4_VELLIMIT_DT						90
#define INDY_JOINT5_VELLIMIT_DT 					90
#define INDY_JOINT6_VELLIMIT_DT 					0

#elif defined (__INDY7__)
#define INDY_JOINT0_POSLIMIT_MAX 					175
#define INDY_JOINT0_POSLIMIT_MIN 					-175
#define INDY_JOINT1_POSLIMIT_MAX 					175
#define INDY_JOINT1_POSLIMIT_MIN 					-175
#define INDY_JOINT2_POSLIMIT_MAX 					175
#define INDY_JOINT2_POSLIMIT_MIN 					-175
#define INDY_JOINT3_POSLIMIT_MAX 					175
#define INDY_JOINT3_POSLIMIT_MIN 					-175
#define INDY_JOINT4_POSLIMIT_MAX 					175
#define INDY_JOINT4_POSLIMIT_MIN 					-175
#if defined (__INDY7_PROTO__) || defined (__INDY7_EARLY__) || defined (__INDY7_MID__)
#define INDY_JOINT5_POSLIMIT_MAX 					175		//175
#define INDY_JOINT5_POSLIMIT_MIN 					-175	//-175
#else
#define INDY_JOINT5_POSLIMIT_MAX 					215		//175
#define INDY_JOINT5_POSLIMIT_MIN 					-215	//-175
#endif
#define INDY_JOINT6_POSLIMIT_MAX 					0
#define INDY_JOINT6_POSLIMIT_MIN 					0
#define INDY_JOINT0_VELLIMIT 						159
#define INDY_JOINT1_VELLIMIT 						159
#define INDY_JOINT2_VELLIMIT 						159
#define INDY_JOINT3_VELLIMIT 						179
#define INDY_JOINT4_VELLIMIT 						179
#define INDY_JOINT5_VELLIMIT	 					179
#define INDY_JOINT6_VELLIMIT	 					0
#define INDY_JOINT0_VELLIMIT_DT						90
#define INDY_JOINT1_VELLIMIT_DT						90
#define INDY_JOINT2_VELLIMIT_DT						90
#define INDY_JOINT3_VELLIMIT_DT						90
#define INDY_JOINT4_VELLIMIT_DT						90
#define INDY_JOINT5_VELLIMIT_DT 					90
#define INDY_JOINT6_VELLIMIT_DT 					0

#elif defined (__OPTI5__) || defined (__OPTI10__)
#define INDY_JOINT0_POSLIMIT_MAX 					359
#define INDY_JOINT0_POSLIMIT_MIN 					-359
#define INDY_JOINT1_POSLIMIT_MAX 					359
#define INDY_JOINT1_POSLIMIT_MIN 					-359
#define INDY_JOINT2_POSLIMIT_MAX 					359
#define INDY_JOINT2_POSLIMIT_MIN 					-359
#define INDY_JOINT3_POSLIMIT_MAX 					359
#define INDY_JOINT3_POSLIMIT_MIN 					-359
#define INDY_JOINT4_POSLIMIT_MAX 					359
#define INDY_JOINT4_POSLIMIT_MIN 					-359
#define INDY_JOINT5_POSLIMIT_MAX 					359
#define INDY_JOINT5_POSLIMIT_MIN 					-359
#define INDY_JOINT6_POSLIMIT_MAX 					0
#define INDY_JOINT6_POSLIMIT_MIN 					0
#if defined(__OPTI10__)
#define INDY_JOINT0_VELLIMIT 						122.5
#define INDY_JOINT1_VELLIMIT 						122.5
#else // __OPTI5__
#define INDY_JOINT0_VELLIMIT 						177
#define INDY_JOINT1_VELLIMIT 						177
#endif
#define INDY_JOINT2_VELLIMIT 						177
#define INDY_JOINT3_VELLIMIT 						177
#define INDY_JOINT4_VELLIMIT 						177
#define INDY_JOINT5_VELLIMIT	 					177
#define INDY_JOINT6_VELLIMIT	 					0
#if defined(__OPTI10__)
#define INDY_JOINT0_VELLIMIT_DT						60
#define INDY_JOINT1_VELLIMIT_DT						60
#else   // __OPTI5__
#define INDY_JOINT0_VELLIMIT_DT						90
#define INDY_JOINT1_VELLIMIT_DT						90
#endif
#define INDY_JOINT2_VELLIMIT_DT						90
#define INDY_JOINT3_VELLIMIT_DT						90
#define INDY_JOINT4_VELLIMIT_DT						90
#define INDY_JOINT5_VELLIMIT_DT 					90
#define INDY_JOINT6_VELLIMIT_DT	 					0

#elif defined (__INDY3__) || defined (__INDY5__) || defined (__INDY10__)
#define INDY_JOINT0_POSLIMIT_MAX 					175
#define INDY_JOINT0_POSLIMIT_MIN 					-175
#define INDY_JOINT1_POSLIMIT_MAX 					175
#define INDY_JOINT1_POSLIMIT_MIN 					-175
#define INDY_JOINT2_POSLIMIT_MAX 					175
#define INDY_JOINT2_POSLIMIT_MIN 					-175
#define INDY_JOINT3_POSLIMIT_MAX 					175
#define INDY_JOINT3_POSLIMIT_MIN 					-175
#define INDY_JOINT4_POSLIMIT_MAX 					175
#define INDY_JOINT4_POSLIMIT_MIN 					-175
#define INDY_JOINT5_POSLIMIT_MAX 					175
#define INDY_JOINT5_POSLIMIT_MIN 					-175
#define INDY_JOINT6_POSLIMIT_MAX 					0
#define INDY_JOINT6_POSLIMIT_MIN 					0
#if defined(__INDY10__)
#define INDY_JOINT0_VELLIMIT 						65
#define INDY_JOINT1_VELLIMIT 						65
#else
#define INDY_JOINT0_VELLIMIT 						95
#define INDY_JOINT1_VELLIMIT 						95
#endif
#define INDY_JOINT2_VELLIMIT 						95
#define INDY_JOINT3_VELLIMIT 						95
#define INDY_JOINT4_VELLIMIT 						95
#define INDY_JOINT5_VELLIMIT	 					95
#define INDY_JOINT6_VELLIMIT	 					0
#if defined(__INDY10__)
#define INDY_JOINT0_VELLIMIT_DT						45
#define INDY_JOINT1_VELLIMIT_DT						45
#else
#define INDY_JOINT0_VELLIMIT_DT						60
#define INDY_JOINT1_VELLIMIT_DT						60
#endif
#define INDY_JOINT2_VELLIMIT_DT						60
#define INDY_JOINT3_VELLIMIT_DT						60
#define INDY_JOINT4_VELLIMIT_DT						60
#define INDY_JOINT5_VELLIMIT_DT 					60
#define INDY_JOINT6_VELLIMIT_DT 					0

#elif defined (__INDYP1__)
#define INDY_JOINT0_POSLIMIT_MAX 					180
#define INDY_JOINT0_POSLIMIT_MIN 					-180
#define INDY_JOINT1_POSLIMIT_MAX 					180
#define INDY_JOINT1_POSLIMIT_MIN 					-180
#define INDY_JOINT2_POSLIMIT_MAX 					180
#define INDY_JOINT2_POSLIMIT_MIN 					-180
#define INDY_JOINT3_POSLIMIT_MAX 					180
#define INDY_JOINT3_POSLIMIT_MIN 					-180
#define INDY_JOINT4_POSLIMIT_MAX 					180
#define INDY_JOINT4_POSLIMIT_MIN 					-180
#define INDY_JOINT5_POSLIMIT_MAX 					180
#define INDY_JOINT5_POSLIMIT_MIN 					-180
#define INDY_JOINT6_POSLIMIT_MAX 					180
#define INDY_JOINT6_POSLIMIT_MIN 					-180
#define INDY_JOINT0_VELLIMIT 						179
#define INDY_JOINT1_VELLIMIT 						179
#define INDY_JOINT2_VELLIMIT 						179
#define INDY_JOINT3_VELLIMIT 						179
#define INDY_JOINT4_VELLIMIT 						179
#define INDY_JOINT5_VELLIMIT	 					179
#define INDY_JOINT6_VELLIMIT	 					179
#define INDY_JOINT0_VELLIMIT_DT						90
#define INDY_JOINT1_VELLIMIT_DT						90
#define INDY_JOINT2_VELLIMIT_DT						90
#define INDY_JOINT3_VELLIMIT_DT						90
#define INDY_JOINT4_VELLIMIT_DT						90
#define INDY_JOINT5_VELLIMIT_DT 					90
#define INDY_JOINT6_VELLIMIT_DT 					90

#elif defined (__INDYRP2__)
#define INDY_JOINT0_POSLIMIT_MAX 					175
#define INDY_JOINT0_POSLIMIT_MIN 					-175
#define INDY_JOINT1_POSLIMIT_MAX 					175
#define INDY_JOINT1_POSLIMIT_MIN 					-175
#define INDY_JOINT2_POSLIMIT_MAX 					175
#define INDY_JOINT2_POSLIMIT_MIN 					-175
#define INDY_JOINT3_POSLIMIT_MAX 					175
#define INDY_JOINT3_POSLIMIT_MIN 					-175
#define INDY_JOINT4_POSLIMIT_MAX 					175
#define INDY_JOINT4_POSLIMIT_MIN 					-175
#define INDY_JOINT5_POSLIMIT_MAX 					175
#define INDY_JOINT5_POSLIMIT_MIN 					-175
#define INDY_JOINT6_POSLIMIT_MAX 					175
#define INDY_JOINT6_POSLIMIT_MIN 					-175
#define INDY_JOINT0_VELLIMIT 						159
#define INDY_JOINT1_VELLIMIT 						159
#define INDY_JOINT2_VELLIMIT 						159
#define INDY_JOINT3_VELLIMIT 						159
#define INDY_JOINT4_VELLIMIT 						179
#define INDY_JOINT5_VELLIMIT	 					179
#define INDY_JOINT6_VELLIMIT	 					179
#define INDY_JOINT0_VELLIMIT_DT						90
#define INDY_JOINT1_VELLIMIT_DT						90
#define INDY_JOINT2_VELLIMIT_DT						90
#define INDY_JOINT3_VELLIMIT_DT						90
#define INDY_JOINT4_VELLIMIT_DT						90
#define INDY_JOINT5_VELLIMIT_DT 					90
#define INDY_JOINT6_VELLIMIT_DT 					90

#endif

#if defined (__INDY3__) || defined (__INDY5__) || defined (__INDY10__)
#define INDY_JOINT_POSLIMIT_CLOSE_PADDING	30	//Used in direct teaching mode for only Elfin Series
#elif defined (__OPTI5__) || defined (__OPTI10__)
#define INDY_JOINT_POSLIMIT_CLOSE_PADDING	20	//Used in direct teaching mode for only Opti
#else
#define INDY_JOINT_POSLIMIT_CLOSE_PADDING	10	//Used in direct teaching mode
#endif

//minmax == 0 ? max : min
#define INDY_JOINT_POSLIMIT(joint, minmax) 	(joint==0?(minmax==0?INDY_JOINT0_POSLIMIT_MAX:INDY_JOINT0_POSLIMIT_MIN):	\
											(joint==1?(minmax==0?INDY_JOINT1_POSLIMIT_MAX:INDY_JOINT1_POSLIMIT_MIN):	\
											(joint==2?(minmax==0?INDY_JOINT2_POSLIMIT_MAX:INDY_JOINT2_POSLIMIT_MIN):	\
											(joint==3?(minmax==0?INDY_JOINT3_POSLIMIT_MAX:INDY_JOINT3_POSLIMIT_MIN):	\
											(joint==4?(minmax==0?INDY_JOINT4_POSLIMIT_MAX:INDY_JOINT4_POSLIMIT_MIN):	\
											(joint==5?(minmax==0?INDY_JOINT5_POSLIMIT_MAX:INDY_JOINT5_POSLIMIT_MIN):	\
											(joint==6?(minmax==0?INDY_JOINT6_POSLIMIT_MAX:INDY_JOINT6_POSLIMIT_MIN): 0)))))))

#define INDY_JOINT_VELLIMIT(joint, minmax) 	(joint==0?(minmax==0?INDY_JOINT0_VELLIMIT:-INDY_JOINT0_VELLIMIT):	\
											(joint==1?(minmax==0?INDY_JOINT1_VELLIMIT:-INDY_JOINT1_VELLIMIT):	\
											(joint==2?(minmax==0?INDY_JOINT2_VELLIMIT:-INDY_JOINT2_VELLIMIT):	\
											(joint==3?(minmax==0?INDY_JOINT3_VELLIMIT:-INDY_JOINT3_VELLIMIT):	\
											(joint==4?(minmax==0?INDY_JOINT4_VELLIMIT:-INDY_JOINT4_VELLIMIT):	\
											(joint==5?(minmax==0?INDY_JOINT5_VELLIMIT:-INDY_JOINT5_VELLIMIT):	\
											(joint==6?(minmax==0?INDY_JOINT6_VELLIMIT:-INDY_JOINT6_VELLIMIT): 0)))))))

#define INDY_JOINT_VELLIMIT_DT(joint, minmax) 	(joint==0?(minmax==0?INDY_JOINT0_VELLIMIT_DT:-INDY_JOINT0_VELLIMIT_DT):	\
												(joint==1?(minmax==0?INDY_JOINT1_VELLIMIT_DT:-INDY_JOINT1_VELLIMIT_DT):	\
												(joint==2?(minmax==0?INDY_JOINT2_VELLIMIT_DT:-INDY_JOINT2_VELLIMIT_DT):	\
												(joint==3?(minmax==0?INDY_JOINT3_VELLIMIT_DT:-INDY_JOINT3_VELLIMIT_DT):	\
												(joint==4?(minmax==0?INDY_JOINT4_VELLIMIT_DT:-INDY_JOINT4_VELLIMIT_DT):	\
												(joint==5?(minmax==0?INDY_JOINT5_VELLIMIT_DT:-INDY_JOINT5_VELLIMIT_DT):	\
												(joint==6?(minmax==0?INDY_JOINT6_VELLIMIT_DT:-INDY_JOINT6_VELLIMIT_DT): 0)))))))

//ldh
#define INDY_JOINT_TORQUELIMIT_SIM(joint, minmax) 	(joint==0?(minmax==0?INDY_JOINT0_TORQUELIMIT_MAX_SIM:INDY_JOINT0_TORQUELIMIT_MIN_SIM):	\
											(joint==1?(minmax==0?INDY_JOINT1_TORQUELIMIT_MAX_SIM:INDY_JOINT1_TORQUELIMIT_MIN_SIM):	\
											(joint==2?(minmax==0?INDY_JOINT2_TORQUELIMIT_MAX_SIM:INDY_JOINT2_TORQUELIMIT_MIN_SIM):	\
											(joint==3?(minmax==0?INDY_JOINT3_TORQUELIMIT_MAX_SIM:INDY_JOINT3_TORQUELIMIT_MIN_SIM):	\
											(joint==4?(minmax==0?INDY_JOINT4_TORQUELIMIT_MAX_SIM:INDY_JOINT4_TORQUELIMIT_MIN_SIM):	\
											(joint==5?(minmax==0?INDY_JOINT5_TORQUELIMIT_MAX_SIM:INDY_JOINT5_TORQUELIMIT_MIN_SIM):	\
											(joint==6?(minmax==0?INDY_JOINT6_TORQUELIMIT_MAX_SIM:INDY_JOINT6_TORQUELIMIT_MIN_SIM): 0)))))))





/*** Control & Level ***/
// FIXME Not complete values. Confirm min/max velocity, move to robot define file
// Boundary Conditions (Vel/Acc Limit for Trapezodial Interpolator
#define INDY_LIMIT_BOUNDARY_LEVEL_MIN 1
#define INDY_LIMIT_BOUNDARY_LEVEL_MAX 9

extern const double INDY_BOUNDARY_JOINT_VEL_MIN[10];	//max 10
extern const double INDY_BOUNDARY_JOINT_VEL_MAX[10];
extern const double INDY_BOUNDARY_JOINT_ACC_MIN[10];
extern const double INDY_BOUNDARY_JOINT_ACC_MAX[10];

#if defined(__INDYRP__) || defined(__INDY7__) || defined(__INDYRP2__) || defined(__INDYP1__)
#define INDY_LIMIT_TASK_DISP_VEL_MIN	0.1
#define INDY_LIMIT_TASK_DISP_VEL_MAX	0.9
#define INDY_LIMIT_TASK_ROT_VEL_MIN		(20*DEGREE)
#define INDY_LIMIT_TASK_ROT_VEL_MAX		(120*DEGREE)
#elif defined(__INDY3__) || defined(__INDY5__) || defined(__INDY10__)
#define INDY_LIMIT_TASK_DISP_VEL_MIN	0.06
#define INDY_LIMIT_TASK_DISP_VEL_MAX	0.54
#define INDY_LIMIT_TASK_ROT_VEL_MIN		(10*DEGREE)
#define INDY_LIMIT_TASK_ROT_VEL_MAX		(87*DEGREE)
#elif defined(__OPTI5__) || defined(__OPTI10__)
#define INDY_LIMIT_TASK_DISP_VEL_MIN	0.1
#define INDY_LIMIT_TASK_DISP_VEL_MAX	0.9
#define INDY_LIMIT_TASK_ROT_VEL_MIN		(20*DEGREE)
#define INDY_LIMIT_TASK_ROT_VEL_MAX		(120*DEGREE)
#endif

#if defined(__INDYRP__) || defined(__INDY7__) || defined(__INDYRP2__) || defined(__INDYP1__)
#define INDY_LIMIT_TASK_DISP_ACC_MIN	0.2
#define INDY_LIMIT_TASK_DISP_ACC_MAX	1.8
#define INDY_LIMIT_TASK_ROT_ACC_MIN		(40*DEGREE)
#define INDY_LIMIT_TASK_ROT_ACC_MAX		(240*DEGREE)
#elif defined(__INDY3__) || defined(__INDY5__) || defined(__INDY10__)
#define INDY_LIMIT_TASK_DISP_ACC_MIN	0.12
#define INDY_LIMIT_TASK_DISP_ACC_MAX	1.08
#define INDY_LIMIT_TASK_ROT_ACC_MIN		(20*DEGREE)
#define INDY_LIMIT_TASK_ROT_ACC_MAX		(174*DEGREE)
#elif defined(__OPTI5__) || defined(__OPTI10__)
#define INDY_LIMIT_TASK_DISP_ACC_MIN	0.2
#define INDY_LIMIT_TASK_DISP_ACC_MAX	1.8
#define INDY_LIMIT_TASK_ROT_ACC_MIN		(40*DEGREE)
#define INDY_LIMIT_TASK_ROT_ACC_MAX		(240*DEGREE)
#endif

#define INDY_LIMIT_TASK_BLEND_RADIUS_MIN		0.02		//Meter
#define INDY_LIMIT_TASK_BLEND_RADIUS_MAX		0.2			//Meter
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MIN		0.005		//Meter
#if defined (__INDYRP__) || defined(__INDYP1__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.66		//Meter
#elif defined (__INDY7__) || defined(__INDYRP2__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.72
#elif defined (__INDY3__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.53
#elif defined (__INDY5__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.69
#elif defined (__INDY10__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.84
#elif defined (__INDY15__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.93
#elif defined (__OPTI5__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.69
#elif defined (__OPTI10__)
#define INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX		0.84
#endif
#define INDY_LIMIT_JOINT_BLEND_RADIUS_MIN		(3*DEGREE)	//Radians
#define INDY_LIMIT_JOINT_BLEND_RADIUS_MAX		(27*DEGREE)	//Radians


#define INDY_LIMIT_JOINT_BLEND_RADIOUS_DIFF		(INDY_LIMIT_JOINT_BLEND_RADIUS_MAX-INDY_LIMIT_JOINT_BLEND_RADIUS_MIN) / (INDY_LIMIT_BLEND_LEVEL_MAX-INDY_LIMIT_BLEND_LEVEL_MIN)
#define INDY_JOINT_BLEND_RADIOUS(level) 		(INDY_LIMIT_JOINT_BLEND_RADIUS_MIN + (INDY_LIMIT_JOINT_BLEND_RADIOUS_DIFF*(level-INDY_LIMIT_BLEND_LEVEL_MIN)))
#define INDY_LIMIT_TASK_BLEND_RADIOUS_DIFF		(INDY_LIMIT_TASK_BLEND_RADIUS_MAX-INDY_LIMIT_TASK_BLEND_RADIUS_MIN) / (INDY_LIMIT_BLEND_LEVEL_MAX-INDY_LIMIT_BLEND_LEVEL_MIN)
#define INDY_TASK_BLEND_RADIOUS(level) 			(INDY_LIMIT_TASK_BLEND_RADIUS_MIN + (INDY_LIMIT_TASK_BLEND_RADIOUS_DIFF*(level-INDY_LIMIT_BLEND_LEVEL_MIN)))
#define INDY_LIMIT_TASK_PL_BLEND_RADIOUS_DIFF	(INDY_LIMIT_TASK_PL_BLEND_RADIUS_MAX-INDY_LIMIT_TASK_PL_BLEND_RADIUS_MIN) / (INDY_LIMIT_BLEND_LEVEL_MAX-INDY_LIMIT_BLEND_LEVEL_MIN)
#define INDY_TASK_PL_BLEND_RADIOUS(level) 		(INDY_LIMIT_TASK_PL_BLEND_RADIUS_MIN + (INDY_LIMIT_TASK_PL_BLEND_RADIOUS_DIFF*(level-INDY_LIMIT_BLEND_LEVEL_MIN)))


//ldh - we don't know system id(inertia) perfectly. (especially, IndyRP, Indy7, IndyRP2, Indy15)
//19.03.11 - test Indy7. others from motor spec. IndyRP, IndyP1.. don't know
/*** Torque Limit in Simulation ***/
#if defined (__INDYRP__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-35
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					35
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-100
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					100
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					20
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDYP1__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-35
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					35
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-100
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					100
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDY7__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-92
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					92
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-92
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					92
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-35
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					35
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					20
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDYRP2__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-92
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					92
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-92
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					92
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-35
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					35
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-35
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					35
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDY15__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-100
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					100
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-300
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					300
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-100
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					100
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					40
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDY3__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-45
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					45
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-45
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					45
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-27
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					27
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-27
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					27
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-9
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					9
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-9
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					9
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDY5__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-80
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					80
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-80
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					80
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-45
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					45
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-45
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					45
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-9
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					9
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-9
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					9
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__INDY10__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-140
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					140
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-140
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					140
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-80
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					80
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-80
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					80
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-27
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					27
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-27
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					27
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__OPTI5__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-101
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					101
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-101
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					101
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					20
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					20
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#elif defined (__OPTI10__)
#define INDY_JOINT0_TORQUELIMIT_MIN_SIM					-169
#define INDY_JOINT0_TORQUELIMIT_MAX_SIM					169
#define INDY_JOINT1_TORQUELIMIT_MIN_SIM					-169
#define INDY_JOINT1_TORQUELIMIT_MAX_SIM					169
#define INDY_JOINT2_TORQUELIMIT_MIN_SIM					-101
#define INDY_JOINT2_TORQUELIMIT_MAX_SIM					101
#define INDY_JOINT3_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT3_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT4_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT4_TORQUELIMIT_MAX_SIM					40
#define INDY_JOINT5_TORQUELIMIT_MIN_SIM					-40
#define INDY_JOINT5_TORQUELIMIT_MAX_SIM					40
//dummy
#define INDY_JOINT6_TORQUELIMIT_MIN_SIM					-20
#define INDY_JOINT6_TORQUELIMIT_MAX_SIM					20
#endif


/*** RobotSepc Types ***/
#define INDY_ROBOTSPEC_TYPE_NONE						0
#define INDY_ROBOTSPEC_TYPE_ZERO_POSITION				1
#define INDY_ROBOTSPEC_TYPE_HOME_POSITION				2
#define INDY_ROBOTSPEC_TYPE_GAIN_TUNING					3
#define INDY_ROBOTSPEC_TYPE_GAIN_TUNING_TASK			4
#define INDY_ROBOTSPEC_TYPE_GAIN_TUNING_TASK_IMPEDANCE	5
#define INDY_ROBOTSPEC_TYPE_COLLISION_DETECTION_USER	6
#define INDY_ROBOTSPEC_TYPE_COLLISION_DETECTION			7
#define INDY_ROBOTSPEC_TYPE_COLLISION_LEVEL				8
#define INDY_ROBOTSPEC_TYPE_WAYPOINT_TIME				9
#define INDY_ROBOTSPEC_TYPE_TOOL_PROPERTIES				10
#define INDY_ROBOTSPEC_TYPE_ROBOT_MOUNT_ANGLE			11
#define INDY_ROBOTSPEC_TYPE_RATEDTAU					12
#define INDY_ROBOTSPEC_TYPE_TORQUE_CONSTANT				13
#define INDY_ROBOTSPEC_TYPE_SERIAL_NUMBER	    		100
#define INDY_ROBOTSPEC_TYPE_SLAVES_INIT                 200
#define INDY_ROBOTSPEC_TYPE_SLAVES_CONNECTION           201

/*** File Locations ***/
#if defined (__INDYRP__)
#define INDY_SETTING_XML_SERIAL_NUMBER 					"/home/user/release/RobotSpecs/IndyRP/SerialNumberIndyRP.xml"
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/IndyRP/ZeroPosIndyRP.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/IndyRP/HomePosIndyRP.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/IndyRP/GainTuningIndyRP.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/IndyRP/TaskGainTuningIndyRP.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/IndyRP/TaskGainTuningIndyRP-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/IndyRP/CollisionDetectionUserIndyRP.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/IndyRP/CollisionDetectionIndyRP.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/IndyRP/CollisionLevelIndyRP.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/IndyRP/WaypointTimeIndyRP.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/IndyRP/ToolPropertiesIndyRP.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/IndyRP/MountAnglesIndyRP.xml"
#define INDY_SETTING_XML_RATEDTAU					"/home/user/release/RobotSpecs/IndyRP/RatedTauIndy7.xml"
#define INDY_SETTING_XML_TORQUE_CONSTANT				"/home/user/release/RobotSpecs/IndyRP/TorqueConstantIndy7.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/IndyRP/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/IndyRP/"
#elif defined (__INDY7__)
#define INDY_SETTING_XML_SERIAL_NUMBER 					"/home/user/release/RobotSpecs/Indy7/SerialNumberIndy7.xml"
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Indy7/ZeroPosIndy7.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Indy7/HomePosIndy7.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Indy7/GainTuningIndy7.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Indy7/TaskGainTuningIndy7.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Indy7/TaskGainTuningIndy7-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Indy7/CollisionDetectionUserIndy7.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Indy7/CollisionDetectionIndy7.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Indy7/CollisionLevelIndy7.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Indy7/WaypointTimeIndy7.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Indy7/ToolPropertiesIndy7.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/Indy7/MountAnglesIndy7.xml"
#define INDY_SETTING_XML_RATEDTAU						"/home/user/release/RobotSpecs/Indy7/RatedTauIndy7.xml"
#define INDY_SETTING_XML_TORQUE_CONSTANT				"/home/user/release/RobotSpecs/Indy7/TorqueConstantIndy7.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Indy7/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Indy7/"
#elif defined (__OPTI5__)
#define INDY_SETTING_XML_SERIAL_NUMBER 					"/home/user/release/RobotSpecs/IndyRP/SerialNumberIndyRP.xml"
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Opti5/ZeroPosOpti5.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Opti5/HomePosOpti5.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Opti5/GainTuningOpti5.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Opti5/TaskGainTuningOpti5.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Opti5/TaskGainTuningOpti5-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Opti5/CollisionDetectionUserOpti5.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Opti5/CollisionDetectionOpti5.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Opti5/CollisionLevelOpti5.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Opti5/WaypointTimeOpti5.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Opti5/ToolPropertiesOpti5.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/Opti5/MountAnglesOpti5.xml"
#define INDY_SETTING_XML_RATEDTAU					"/home/user/release/RobotSpecs/Opti5/RatedTauIndy7.xml"
#define INDY_SETTING_XML_TORQUE_CONSTANT				"/home/user/release/RobotSpecs/Opti5/TorqueConstantIndy7.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Opti5/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Opti5/"
#elif defined (__OPTI10__)
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Opti10/ZeroPosOpti10.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Opti10/HomePosOpti10.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Opti10/GainTuningOpti10.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Opti10/TaskGainTuningOpti10.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Opti10/TaskGainTuningOpti10-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Opti10/CollisionDetectionUserOpti10.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Opti10/CollisionDetectionOpti10.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Opti10/CollisionLevelOpti10.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Opti10/WaypointTimeOpti10.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Opti10/ToolPropertiesOpti10.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Opti10/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Opti10/"
#elif defined (__INDY3__)
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Indy3/ZeroPosIndy3.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Indy3/HomePosIndy3.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Indy3/GainTuningIndy3.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Indy3/TaskGainTuningIndy3.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Indy3/TaskGainTuningIndy3-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Indy3/CollisionDetectionUserIndy3.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Indy3/CollisionDetectionIndy3.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Indy3/CollisionLevelIndy3.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Indy3/WaypointTimeIndy3.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Indy3/ToolPropertiesIndy3.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/Indy3/MountAnglesIndy3.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Indy3/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Indy3/"
#elif defined (__INDY5__)
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Indy5/ZeroPosIndy5.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Indy5/HomePosIndy5.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Indy5/GainTuningIndy5.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Indy5/TaskGainTuningIndy5.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Indy5/TaskGainTuningIndy5-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Indy5/CollisionDetectionUserIndy5.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Indy5/CollisionDetectionIndy5.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Indy5/CollisionLevelIndy5.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Indy5/WaypointTimeIndy5.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Indy5/ToolPropertiesIndy5.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/Indy5/MountAnglesIndy5.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Indy5/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Indy5/"
#elif defined (__INDY10__)
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/Indy10/ZeroPosIndy10.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/Indy10/HomePosIndy10.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/Indy10/GainTuningIndy10.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/Indy10/TaskGainTuningIndy10.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/Indy10/TaskGainTuningIndy10-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/Indy10/CollisionDetectionUserIndy10.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/Indy10/CollisionDetectionIndy10.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/Indy10/CollisionLevelIndy10.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/Indy10/WaypointTimeIndy10.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/Indy10/ToolPropertiesIndy10.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/Indy10/MountAnglesIndy10.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/Indy10/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/Indy10/"
#elif defined (__INDYRP2__)
#define INDY_SETTING_XML_SERIAL_NUMBER 					"/home/user/release/RobotSpecs/IndyRP2/SerialNumberIndyRP2.xml"
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/IndyRP2/ZeroPosIndyRP2.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/IndyRP2/HomePosIndyRP2.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/IndyRP2/GainTuningIndyRP2.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/IndyRP2/TaskGainTuningIndyRP2.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/IndyRP2/TaskGainTuningIndyRP2-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION_USER		"/home/user/release/RobotSpecs/IndyRP2/CollisionDetectionUserIndyRP2.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/IndyRP2/CollisionDetectionIndyRP2.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/IndyRP2/CollisionLevelIndyRP2.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/IndyRP2/WaypointTimeIndyRP2.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/IndyRP2/ToolPropertiesIndyRP2.xml"
#define INDY_SETTING_XML_ROBOT_MOUNT_ANGLE 				"/home/user/release/RobotSpecs/IndyRP2/MountAnglesIndyRP2.xml"
#define INDY_SETTING_XML_RATEDTAU						"/home/user/release/RobotSpecs/IndyRP2/RatedTauIndyRP2.xml"
#define INDY_SETTING_XML_TORQUE_CONSTANT				"/home/user/release/RobotSpecs/IndyRP2/TorqueConstantIndyRP2.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/IndyRP2/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/IndyRP2/"
#elif defined (__INDYP1__)
#define INDY_SETTING_XML_ZERO_POSITION 					"/home/user/release/RobotSpecs/IndyP1/ZeroPosIndyP1.xml"
#define INDY_SETTING_XML_HOME_POSITION 					"/home/user/release/RobotSpecs/IndyP1/HomePosIndyP1.xml"
#define INDY_SETTING_XML_GAIN_TUNING 					"/home/user/release/RobotSpecs/IndyP1/GainTuningIndyP1.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK 				"/home/user/release/RobotSpecs/IndyP1/TaskGainTuningIndyP1.xml"
#define INDY_SETTING_XML_GAIN_TUNING_TASK_IMPEDANCE 	"/home/user/release/RobotSpecs/IndyP1/TaskGainTuningIndyP1-Impedance.xml"
#define INDY_SETTING_XML_COLLISION_DETECTION 			"/home/user/release/RobotSpecs/IndyP1/CollisionDetectionIndyP1.xml"
#define INDY_SETTING_XML_COLLISION_LEVEL 				"/home/user/release/RobotSpecs/IndyP1/CollisionLevelIndyP1.xml"
#define INDY_SETTING_XML_WAYPOINT_TIME 					"/home/user/release/RobotSpecs/IndyP1/WaypointTimeIndyP1.xml"
#define INDY_SETTING_XML_TOOL_PROPERTIES 				"/home/user/release/RobotSpecs/IndyP1/ToolPropertiesIndyP1.xml"
#define INDY_SETTING_JSON_USER_CONFIG					"/home/user/Conty/IndyP1/UserConfig.json"
#define INDY_CONTY_DIR 									"/home/user/Conty/IndyP1/"
#endif
#define INDY_CONTY_DIR_BASE 							"/home/user/Conty/"

/* LOGGING FILES */
#define INDY_FILE_LOG_PERIOD				"/home/user/release/LogData/indyLog_"
#define INDY_FILE_LOG_EVENT					"/home/user/release/LogData/indyEventLog_"
#define INDY_FILE_LOG_BUFFER				"/home/user/release/LogData/indyEventBuffLog_"

/* CONTY FILES */
#define INDY_CONTY_DEFAULT_PROGRAM			"DefaultProgram.json"
#define INDY_CADKIT_SAVED_PROGRAM			"CadkitProgram.json"
#define INDY_CONTY_SAVED_TOOLINFO			"ToolsInfo.json"

#endif /* NRMKINDY_CONSTANTINDY_H_ */
