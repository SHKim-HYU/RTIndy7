/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "RTIndy7Client.h"


JointInfo info;
JointInfo sim;

MR_Indy7 mr_indy7;

// Xenomai RT tasks
RT_TASK RTIndy7_task;
#ifdef __BULLET__
RT_TASK bullet_task;
#endif
#ifdef __CASADI__
RT_TASK indysim_task;
#endif
RT_TASK safety_task;
RT_TASK print_task;

// #ifdef __BULLET__
// b3RobotSimulatorClientAPI* sim;
// #endif

//For Trajectory management
//Task

//////////////////////////////////////////////////////////////////
#ifdef __CASADI__

	JVec indy7_FD()
	{
		RTIME start, end;
		// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_fd.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_fd.so");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "aba");
	    if (dlerror()) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    // Allocate input/output buffers and work vectors dlrj
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[18];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    // double input_pos[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    // double input_vel[] = {info.act.q_dot(0),info.act.q_dot(1),info.act.q_dot(2),info.act.q_dot(3),info.act.q_dot(4),info.act.q_dot(5)};
	    // double input_tau[] = {info.act.tau(0),info.act.tau(1),info.act.tau(2),info.act.tau(3),info.act.tau(4),info.act.tau(5)};

	    double input_pos[sz_arg] = {-0.128198, -0.059945, 1.243974, 3.058833, -2.110628, 0.131757};
	    double input_vel[sz_arg] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    double input_tau[sz_arg] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[3*i] = &input_pos[i];
	        arg[3*i+1] = &input_vel[i];
	        arg[3*i+2] = &input_tau[i];
	    }

	    // Set output buffers
	    double output_values[6];
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.");
	    }
	    end = rt_timer_read();

	    // Print the result
	    rt_printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        rt_printf("%lf ", output_values[i]);
	    }
	    rt_printf("\n");
	    rt_printf("[cs]computation time for \"FD\": %lius\n", (end-start)/1000);
	    
	    JVec result_vector;

	    for (casadi_int i = 0; i < sz_res; ++i) {
        	result_vector(i) = output_values[i];
    	}

	    // Free the handle
	    dlclose(handle);

	    return result_vector;
	}

	int indy7_FD_rk()
	{
		RTIME start, end;
		double dt=((double) cycle_ns)/((double) NSEC_PER_SEC);
		// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_fd.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_fd.so");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "aba");
	    if (dlerror()) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    // Allocate input/output buffers and work vectors dlrj
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];
	    double temp_q[6], temp_q_dot[6], temp_tau[6];

	    int mem = 0;  // No thread-local memory management
	    JVec k1, k2, k3, k4;

	    // Set output buffers
	    double output_values[6];
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    start = rt_timer_read();
	    
	    // 1st stage
	    // Set input values
	    for (casadi_int i = 0; i < sz_arg; ++i) {
		    temp_q[i] = info.act.q(i);
		    temp_q_dot[i] = info.act.q_dot(i);
		    temp_tau[i] = info.act.tau(i);
		    // temp_tau[i] = 0.0;

		    arg[i] = &temp_q[i];
		    arg[i + 6] = &temp_q_dot[i];
		    arg[i + 12] = &temp_tau[i];
		}

	    // Evaluate the function	    
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    for (casadi_int i = 0; i < sz_res; ++i) {
        	k1(i) = output_values[i];
    	}

    	JVec _q1 = info.act.q + 0.5 * dt * info.act.q_dot;
    	JVec _q_dot1 = info.act.q_dot + 0.5 * dt * k1;
    	JVec _q_dot_tmp = 0.5 * dt * k1;

	    // 2nd stage
	    // Set input values
	    for (casadi_int i = 0; i < sz_arg; ++i) {
		    temp_q[i] = _q1(i);
		    temp_q_dot[i] = _q_dot1(i);

		    arg[i] = &temp_q[i];
		    arg[i + 6] = &temp_q_dot[i];
		}

	    // Evaluate the function	    
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    for (casadi_int i = 0; i < sz_res; ++i) {
        	k2(i) = output_values[i];
    	}
    	JVec _q2 = info.act.q + 0.5 * dt * _q_dot1;
    	JVec _q_dot2 = info.act.q_dot + 0.5 * dt * k2;

    	// 3th stage
	    // Set input values
	    for (casadi_int i = 0; i < sz_arg; ++i) {
		    temp_q[i] = _q2(i);
		    temp_q_dot[i] = _q_dot2(i);

		    arg[i] = &temp_q[i];
		    arg[i + 6] = &temp_q_dot[i];
		}

	    // Evaluate the function	    
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    for (casadi_int i = 0; i < sz_res; ++i) {
        	k3(i) = output_values[i];
    	}
    	JVec _q3 = info.act.q + dt * _q_dot2;
    	JVec _q_dot3 = info.act.q_dot + dt * k3;

	   	// 4th stage
	    // Set input values
	    for (casadi_int i = 0; i < sz_arg; ++i) {
		    temp_q[i] = _q3(i);
		    temp_q_dot[i] = _q_dot3(i);

		    arg[i] = &temp_q[i];
		    arg[i + 6] = &temp_q_dot[i];
		}

	    // Evaluate the function	    
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.");
	    }

	    for (casadi_int i = 0; i < sz_res; ++i) {
        	k4(i) = output_values[i];
    	}

    	info.nom.q = info.act.q + (dt / 6.0) * (info.act.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
    	info.nom.q_dot = info.act.q_dot + (dt / 6.0) * (k1 + 2 * (k2 + k3) + k4);

	    end = rt_timer_read();

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}


	JVec indy7_G()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_G.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_G.so, error: %s\n");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "generalized_gravity");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"generalized_gravity\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors dlrj
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[6];
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management

	    JVec G;

	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();

	    // Print the result
	    rt_printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        rt_printf("%lf ", output_values[i]);
	    }
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        G(i) = output_values[i];
	    }

	    rt_printf("[cs]computation time for \"G\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    mr_indy7.Gvec(info.act.q);
	    end = rt_timer_read();
	    
	    rt_printf("[mr]computation time for \"G\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return G;
	}
	MassMat indy7_M()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_M.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_M.so, error: %s\n");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "M");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"M\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[sz_arg];
	    double* res[sz_res*sz_res];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    MassMat M;

	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();
	    
	    
	    
	    // Print the result
	    printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {
	            rt_printf("%lf ", output_values[i * sz_res + j]);
	        }
	        rt_printf("\n");
	    }

	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {   
	        	M(j,i) = output_values[i * sz_res + j];
	        }
	    }

	    rt_printf("[cs]computation time for \"M\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    MassMat M_mr = mr_indy7.Mmat(info.act.q);
	    end = rt_timer_read();
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {
	            rt_printf("%lf ", M_mr(i,j));
	        }
	        rt_printf("\n");
	    }
	    rt_printf("[mr]computation time for \"M\": %lius\n", (end-start)/1000);


	    // Free the handle
	    dlclose(handle);

	    return M;
	}

	MassMat indy7_Minv()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_Minv.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_Minv.so, error: %s\n");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "Minv");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"Minv\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[sz_arg];
	    double* res[sz_res*sz_res];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {
	            printf("%lf ", output_values[i * sz_res + j]);
	        }
	        printf("\n");
	    }

	    MassMat Minv;

	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {   
	        	Minv(j,i) = output_values[i * sz_res + j];
	        }
	    }

	    rt_printf("[cs]computation time for \"Minv\": %lius\n", (end-start)/1000);
	    
	    // Free the handle
	    dlclose(handle);

	    return Minv;
	}

	MassMat indy7_C()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_C.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_C.so, error: %s\n");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "coriolis");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"C\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[2*sz_arg];
	    double* res[sz_res*sz_res];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_pos[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    double input_vel[] = {info.act.q_dot(0),info.act.q_dot(1),info.act.q_dot(2),info.act.q_dot(3),info.act.q_dot(4),info.act.q_dot(5)};

	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[2*i] = &input_pos[i];
	        arg[2*i+1] = &input_vel[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management

	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();

	    // Print the result
	    printf("C Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {
	            printf("%lf ", output_values[i * sz_res + j]);
	        }
	        printf("\n");
	    }

	    MassMat C;

	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {   
	        	C(j,i) = output_values[i * sz_res + j];
	        }
	    }

		rt_printf("[cs]computation time for \"C\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    mr_indy7.Cvec(info.act.q, info.act.q_dot);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"C\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return C;
	}

	SE3 indy7_FK()
	{
	    RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_fk_ee.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_fk_ee.so, error: %s\n");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "fk_T");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"fk_T\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 4;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[16];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[16]; // 4x4x7 matrix
	    for (casadi_int i = 0; i < 16; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    SE3 T_ee;

	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < 4; ++i) {
	    //     for (casadi_int j = 0; j < 4; ++j) {
	    //     	T_ee(j,i)=output_values[4*i+j];
	    //     }
	    // }
	    // cout<<T_ee<<endl;
	    rt_printf("[cs]computation time for \"FK\": %lius\n", (end-start)/1000);
	    start = rt_timer_read();
	    mr_indy7.T_s(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"FK\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return T_ee;
	}

	Jacobian indy7_J_b()
	{
		RTIME start, end;
		// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_J_b.so", RTLD_LAZY);
	    if (handle == 0) {
	        throw std::runtime_error("Cannot open indy7_J_b.so");
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "J_b");
	    if (dlerror()) {
	        throw std::runtime_error("Failed to retrieve \"J_b\" function.\n");
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[36];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        throw std::runtime_error("Function evaluation failed.\n");
	    }
	    end = rt_timer_read();
	    
	    Jacobian J_b;

	    // Print the result
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {   
	        	J_b(j,i) = output_values[i * sz_res + j];
	        }
	    }
	    // rt_printf("[cs]computation time for \"J_b\": %lius\n", (end-start)/1000);

	    start = rt_timer_read();
	    mr_indy7.J_b(info.act.q);
	    end = rt_timer_read();
	    // rt_printf("[mr]computation time for \"J_b\": %lius\n", (end-start)/1000);
	    Jacobian J_b_MR = mr_indy7.J_b(info.act.q);
	    
	    // cout<<"CS:"<<J_b<<endl;
	    // cout<<"MR:"<<J_b_MR<<endl;
	    
	    // Free the handle
	    dlclose(handle);

	    return J_b;
	}
	int indy7_J_s()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_J_s.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_J_s.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "J_s");
	    if (dlerror()) {
	        printf("Failed to retrieve \"J_s\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[6];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    for (casadi_int i = 0; i < sz_arg; ++i) {
	        arg[i] = &input_values[i];
	    }

	    // Set output buffers
	    double output_values[36]; // 6x6 matrix
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        res[i] = &output_values[i];
	    }

	    // Evaluate the function
	    int mem = 0;  // No thread-local memory management
	    
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();
	    
	    
	    // Print the result
	    // printf("Result:\n");
	    // for (casadi_int i = 0; i < sz_res; ++i) {
	    //     for (casadi_int j = 0; j < sz_res; ++j) {
	    //         printf("%g ", output_values[i * sz_res + j]);
	    //     }
	    //     printf("\n");
	    // }
	    rt_printf("[cs]computation time for \"J_s\": %lius\n", (end-start)/1000);

	    start = rt_timer_read();
	    mr_indy7.J_s(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"J_s\": %lius\n", (end-start)/1000);


	    // Free the handle
	    dlclose(handle);

	    return 0;
	}
#endif
//////////////////////////////////////////////////////////////////

void signal_handler(int signum);

void saveLogData(){}

int initAxes()
{
	for (int i = 0; i < NUM_AXIS; i++)
	{	
		Axis[i].setGearRatio(gearRatio[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ENC_CORE);
		Axis[i].setTauADC(TauADC[i]);
		Axis[i].setTauK(TauK[i]);
		Axis[i].setZeroPos(zeroPos[i]);

		Axis[i].setDirQ(dirQ[i]);
		Axis[i].setDirTau(dirTau[i]);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
	}
	
	return 1;
}
/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
	    	info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 2:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
	    	// info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	// info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	traj_time = 3.0;
	    	motion++;
	    	// motion=1;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	// info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	// info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
	    	traj_time = 3.0;
	    	motion=1;
	    	break;
	    default:
	    	info.q_target(0)=info.act.q(0); 	info.q_target(1)=info.act.q(1); 	info.q_target(2)=info.act.q(2);
	    	info.q_target(3)=info.act.q(3); 	info.q_target(4)=info.act.q(4); 	info.q_target(5)=info.act.q(5);
	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<NUM_AXIS;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();

		info.des.q(i)=Axis[i].getDesPosInRad();
		info.des.q_dot(i)=Axis[i].getDesVelInRad();
		info.des.q_ddot(i)=Axis[i].getDesAccInRad();
	}
}


int compute()
{
	Jacobian J_b = indy7_J_b();
	// rt_printf("Jb: \n");
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(0,0), J_b(0,1), J_b(0,2), J_b(0,3), J_b(0,4), J_b(0,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(1,0), J_b(1,1), J_b(1,2), J_b(1,3), J_b(1,4), J_b(1,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(2,0), J_b(2,1), J_b(2,2), J_b(2,3), J_b(2,4), J_b(2,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(3,0), J_b(3,1), J_b(3,2), J_b(3,3), J_b(3,4), J_b(3,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(4,0), J_b(4,1), J_b(4,2), J_b(4,3), J_b(4,4), J_b(4,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b(5,0), J_b(5,1), J_b(5,2), J_b(5,3), J_b(5,4), J_b(5,5));
	
	// Jacobian J_b_MR = mr_indy7.J_b(info.act.q);
	// rt_printf("Jb_MR: \n");
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(0,0), J_b_MR(0,1), J_b_MR(0,2), J_b_MR(0,3), J_b_MR(0,4), J_b_MR(0,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(1,0), J_b_MR(1,1), J_b_MR(1,2), J_b_MR(1,3), J_b_MR(1,4), J_b_MR(1,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(2,0), J_b_MR(2,1), J_b_MR(2,2), J_b_MR(2,3), J_b_MR(2,4), J_b_MR(2,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(3,0), J_b_MR(3,1), J_b_MR(3,2), J_b_MR(3,3), J_b_MR(3,4), J_b_MR(3,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(4,0), J_b_MR(4,1), J_b_MR(4,2), J_b_MR(4,3), J_b_MR(4,4), J_b_MR(4,5));
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", J_b_MR(5,0), J_b_MR(5,1), J_b_MR(5,2), J_b_MR(5,3), J_b_MR(5,4), J_b_MR(5,5));
	// rt_printf("\n\n");

	info.act.tau_ext = J_b.transpose()*info.act.F;

	return 0;
}
void readEcatData(){
	// Drive
	nrmk_master.readBuffer(0x60410, StatusWord);
	nrmk_master.readBuffer(0x60640, ActualPos);
	nrmk_master.readBuffer(0x606c0, ActualVel);
	nrmk_master.readBuffer(0x60770, ActualTor);
	nrmk_master.readBuffer(0x60610, ModeOfOperationDisplay);

#ifdef __CB__
	// IO Module
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x61001, StatusCode);
	nrmk_master.readBuffer(0x61002, DI5V);
	nrmk_master.readBuffer(0x61003, DI1);
	nrmk_master.readBuffer(0x61004, DI2);
	nrmk_master.readBuffer(0x61005, AI1);
	nrmk_master.readBuffer(0x61006, AI2);
	nrmk_master.readBuffer(0x61007, FTRawFxCB);
	nrmk_master.readBuffer(0x61008, FTRawFyCB);
	nrmk_master.readBuffer(0x61009, FTRawFzCB);
	nrmk_master.readBuffer(0x610010, FTRawTxCB);
	nrmk_master.readBuffer(0x610011, FTRawTyCB);
	nrmk_master.readBuffer(0x610012, FTRawTzCB);
	nrmk_master.readBuffer(0x610013, FTOverloadStatusCB);
	nrmk_master.readBuffer(0x610014, FTErrorFlagCB);
	nrmk_master.readBuffer(0x610015, RS485RxCnt);
	nrmk_master.readBuffer(0x610016, RS485RxD0);
	nrmk_master.readBuffer(0x610017, RS485RxD1);
	nrmk_master.readBuffer(0x610018, RS485RxD2);
	nrmk_master.readBuffer(0x610019, RS485RxD3);
	nrmk_master.readBuffer(0x610020, RS485RxD4);
	nrmk_master.readBuffer(0x610021, RS485RxD5);
	nrmk_master.readBuffer(0x610022, RS485RxD6);
	nrmk_master.readBuffer(0x610023, RS485RxD7);
	nrmk_master.readBuffer(0x610024, RS485RxD8);
	nrmk_master.readBuffer(0x610025, RS485RxD9);
#endif

	// Tool
	// [ToDo] 0x61001~0x610025 addition iteratively
	nrmk_master.readBuffer(0x60001, IStatus);
	nrmk_master.readBuffer(0x60002, IButton);
	nrmk_master.readBuffer(0x60003, FTRawFx);
	nrmk_master.readBuffer(0x60004, FTRawFy);
	nrmk_master.readBuffer(0x60005, FTRawFz);
	nrmk_master.readBuffer(0x60006, FTRawTx);
	nrmk_master.readBuffer(0x60007, FTRawTy);
	nrmk_master.readBuffer(0x60008, FTRawTz);
	nrmk_master.readBuffer(0x60009, FTOverloadStatus);
	nrmk_master.readBuffer(0x600010, FTErrorFlag);	
	
	for(int i=0; i<NUM_AXIS;i++)
	{

		Axis[i].setCurrentPosInCnt(ActualPos[i+NUM_IO_MODULE]);
		Axis[i].setCurrentVelInCnt(ActualVel[i+NUM_IO_MODULE]);
		Axis[i].setCurrentTorInCnt(ActualTor[i+NUM_IO_MODULE]);
		
		Axis[i].setCurrentTime(gt);

		info.act.q(i) = Axis[i].getCurrPosInRad();
		info.act.q_dot(i) = Axis[i].getCurrVelInRad();
		info.act.tau(i) = Axis[i].getCurrTorInNm();

		if(!system_ready)
		{
			Axis[i].setTarPosInRad(info.act.q(i));
			Axis[i].setDesPosInRad(info.act.q(i));
		}

	}
	
	// Update RFT data
	info.act.F(0) = (double)FTRawFx[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(1) = (double)FTRawFy[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(2) = (double)FTRawFz[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(3) = (double)FTRawTx[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(4) = (double)FTRawTy[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(5) = (double)FTRawTz[NUM_IO_MODULE+NUM_AXIS] / torque_divider;

#ifdef __CB__
	info.act.F_CB(0) = (double)FTRawFxCB[0] / force_divider;
	info.act.F_CB(1) = (double)FTRawFyCB[0] / force_divider;
	info.act.F_CB(2) = (double)FTRawFzCB[0] / force_divider;
	info.act.F_CB(3) = (double)FTRawTxCB[0] / torque_divider;
	info.act.F_CB(4) = (double)FTRawTyCB[0] / torque_divider;
	info.act.F_CB(5) = (double)FTRawTzCB[0] / torque_divider;
#endif

	// info.act.F<<(double)FTRawFx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawFz[NUM_IO_MODULE+NUM_AXIS]
	//           <<(double)FTRawTx[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTy[NUM_IO_MODULE+NUM_AXIS]<<(double)FTRawTz[NUM_IO_MODULE+NUM_AXIS];
}

void writeEcatData(){
	for(int i=0;i<NUM_AXIS;i++){
		Axis[i].setDesTorInNm(info.des.tau(i));
		TargetTor[i+NUM_IO_MODULE]=Axis[i].getDesTorInCnt();
	}
	// TO DO: write data to actuators in EtherCAT system interface
	nrmk_master.writeBuffer(0x60710, TargetTor);
	// nrmk_master.writeBuffer(0x60600, ModeOfOperation);
}

// RTIndy7_task
void RTIndy7_run(void *arg)
{
	RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	// nrmk_master.syncEcatMaster();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6d::Zero();
	info.des.F_CB = Vector6d::Zero();

	JVec eint = JVec::Zero();
	JVec e = JVec::Zero();

	int ft_init_cnt = 0;

	while (run)
	{
		beginCycle = rt_timer_read();
		periodEcat = 0;
		periodBuffer = 0;

		beginRead = rt_timer_read();
		nrmk_master.processTxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginRead;
		
		// Read data in EtherCAT Buffer
		beginReadbuf = rt_timer_read();
		readEcatData();	
		periodBuffer += (unsigned long) rt_timer_read() - beginReadbuf;
	
		beginCompute = rt_timer_read();
		if(system_ready){

			// Trajectory Generation
			trajectory_generation();
			
			//[ToDo] Add MPC Function 
			compute();	

			// Calculate Joint controller
			// info.des.tau = mr_indy7.ComputedTorqueControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot) + info.act.tau_ext; // calcTorque
			// info.des.tau = mr_indy7.ComputedTorqueControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot); // calcTorque
			// info.des.tau = mr_indy7.Gravity( info.act.q ) + info.act.tau_ext; // calcTorque
			// info.des.tau = mr_indy7.Gravity( info.act.q ); // calcTorque
			e = info.des.q-info.act.q;
			eint = eint + e*period;
			
			// info.des.tau = mr_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot,eint) + info.act.tau_ext;
			info.des.tau = mr_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot,eint);
			info.des.G = mr_indy7.Gravity( info.act.q ); // calcTorque

			// info.des.tau = info.nom.tau;

			// mr_indy7.saturationMaxTorque(info.des.tau,MAX_TORQUES);
		}
		else
		{

			info.des.tau = mr_indy7.Gravity( info.act.q ); // calcTorque
		}
		periodCompute  = (unsigned long) rt_timer_read() - beginCompute;
		
		// Write data in EtherCAT Buffer
		beginWritebuf = rt_timer_read();
		writeEcatData();
		periodBuffer += (unsigned long) rt_timer_read() - beginWritebuf;

		beginWrite = rt_timer_read();
		nrmk_master.processRxDomain();
		periodEcat += (unsigned long) rt_timer_read() - beginWrite;

		endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		
		if (nrmk_master.isSystemReady())
		{	
			if(ft_init_cnt==0)
			{
				// Set bias
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Set Filter 100Hz
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Start
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
#ifdef __CB__
				FTConfigParamCB[0]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
#endif
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else
				system_ready=1;	//all drives have been done

			gt+= period;
			
			if (periodEcat > worstEcat)	worstEcat = periodEcat;
			if (periodBuffer > worstBuffer)	worstBuffer = periodBuffer;
			if (periodCompute > worstCompute) worstCompute = periodCompute;
			if (periodCycle > cycle_ns) overruns++;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
	}
}

#ifdef __CASADI__
// IndySim task
void indysim_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	// Load the shared library
    void* fd_handle = dlopen("../lib/URDF2CASADI/indy7_fd.so", RTLD_LAZY);
    if (fd_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fd.so");
    }
    
    void* G_handle = dlopen("../lib/URDF2CASADI/indy7_G.so", RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open indy7_G.so");
    }
    void* M_handle = dlopen("../lib/URDF2CASADI/indy7_M.so", RTLD_LAZY);
    if (M_handle == 0) {
        throw std::runtime_error("Cannot open indy7_M.so");
    }
    void* C_handle = dlopen("../lib/URDF2CASADI/indy7_C.so", RTLD_LAZY);
    if (C_handle == 0) {
        throw std::runtime_error("Cannot open indy7_C.so");
    }

    void* J_handle = dlopen("../lib/URDF2CASADI/indy7_J_b.so", RTLD_LAZY);
    if (J_handle == 0) {
        throw std::runtime_error("Cannot open indy7_J_b.so");
    }


    // Reset error
    dlerror();

    // Function evaluation
    eval_t fd_eval = (eval_t)dlsym(fd_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    
    eval_t G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    eval_t M_eval = (eval_t)dlsym(M_handle, "M");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    eval_t C_eval = (eval_t)dlsym(C_handle, "coriolis");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

    eval_t J_eval = (eval_t)dlsym(J_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
   
    // Allocate input/output buffers and work vectors dlrj
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* fd_arg[3*sz_arg];
    const double* G_arg[sz_arg];
    const double* M_arg[sz_arg];
    const double* C_arg[2*sz_arg];
    const double* J_arg[sz_arg];

    double* fd_res[sz_res];
    double* G_res[sz_res];
    double* M_res[sz_res*sz_res];
    double* C_res[sz_res*sz_res];
    double* J_res[sz_res*sz_res];

    casadi_int fd_iw[sz_iw], G_iw[sz_iw], M_iw[sz_iw], C_iw[sz_iw], J_iw[sz_iw];
    double fd_w[sz_w], G_w[sz_w], M_w[sz_w], C_w[sz_w], J_w[sz_w];
    
    int fd_mem = 0;  // No thread-local memory management
    int G_mem = 0;  // No thread-local memory management
    int M_mem = 0;  // No thread-local memory management
    int C_mem = 0;  // No thread-local memory management
    int J_mem = 0;  // No thread-local memory management
    

    double temp_q[6], temp_q_dot[6], temp_tau[6];
    
    JVec k1, k2, k3, k4;
    Jacobian J_b;
    MassMat M, C;
    JVec G;

	JVec e = JVec::Zero();
    JVec edot = JVec::Zero();
    JVec eint = JVec::Zero();

    // Set output buffers
    double fd_values[6], G_values[6], M_values[36], C_values[36], J_values[36];
    for (casadi_int i = 0; i < 6; ++i) {
        fd_res[i] = &fd_values[i];
        G_res[i] = &G_values[i];
    }
    for (casadi_int i = 0; i < 36; ++i) {
        M_res[i] = &M_values[i];
        C_res[i] = &C_values[i];
        J_res[i] = &J_values[i];
    }

	int cnt = -1;    

    // // Free the handle
    // dlclose(handle);

	Hinf_Kp = Matrix6d::Zero();
    Hinf_Kv = Matrix6d::Zero();
    Hinf_K_gamma = Matrix6d::Zero();

    for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_4 ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_6 ;

            break;
        }
    }

	// loop
	while(1)
	{
		if(system_ready)
		{
			beginCycle = rt_timer_read();

			
			if(motion>1 && cnt == -1)
			{
				cnt=0;
			}

			////////////////   Implicit Euler method   /////////////////

			/////////////////  RK4   //////////////////
			if(cnt == 0)
			{
				// state update
				JVec _q = info.act.q;
				// JVec _q_dot = info.act.q_dot;
				JVec _q_dot = JVec::Zero();
				JVec _tau = JVec::Zero();

				// 1st stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q(i);
				    temp_q_dot[i] = _q_dot(i);
				    temp_tau[i] = _tau(i);
				    // temp_tau[i] = 0.0;

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}
				
			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
			    //k1
			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k1(i) = fd_values[i];
		    	}
		    	
		    	// k2
		    	JVec _q1 = info.act.q + 0.5 * period * info.act.q_dot;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
		    	JVec _q_dot1 = info.act.q_dot + 0.5 * period * k1; // k2(q_dot)
		    	// JVec _q_dot1 = info.act.q_dot + 0.5 * period * k1;
		    	// JVec _q1 = info.act.q + 0.5 * period * _q_dot1;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
		    	

			    // 2nd stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q1(i);
				    temp_q_dot[i] = _q_dot1(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i+1] = &temp_q_dot[i];
				    fd_arg[3*i+2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k2(i) = fd_values[i]; //k2(q_ddot)
		    	}
		    	JVec _q2 = info.act.q + 0.5 * period * _q_dot1;
		    	JVec _q_dot2 = info.act.q_dot + 0.5 * period * k2;
		    	// JVec _q_dot2 = info.act.q_dot + 0.5 * period * k2;
		    	// JVec _q2 = info.act.q + 0.5 * period * _q_dot2;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
		    	

		    	// 3th stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q2(i);
				    temp_q_dot[i] = _q_dot2(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k3(i) = fd_values[i];
		    	}
		    	JVec _q3 = info.act.q + period * _q_dot2;
		    	JVec _q_dot3 = info.act.q_dot + period * k3;
		    	// JVec _q_dot3 = info.act.q_dot + period * k3;
		    	// JVec _q3 = info.act.q + period * _q_dot3;

			   	// 4th stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q3(i);
				    temp_q_dot[i] = _q_dot3(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k4(i) = fd_values[i];
		    	}

		    	info.nom.q = info.act.q + (period / 6.0) * (info.act.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    	info.nom.q_dot = info.act.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
				
		    	// info.nom.q_dot = info.act.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
		    	// info.nom.q = info.act.q + (period / 6.0) * (_q_dot1 + 2 * (_q_dot2 + _q_dot3) + info.nom.q_dot);
		    	
		    	// Compute Mass matrix
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    M_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (M_eval(M_arg, M_res, M_iw, M_w, M_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
				for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		M(j,i) = M_values[i * sz_res + j];
		        	}
		    	}

		    	// Compute C matrix
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    temp_q_dot[i] = info.nom.q_dot(i);

				    C_arg[2*i] = &temp_q[i];				    
				    C_arg[2*i+1] = &temp_q_dot[i];				    
				}
				// Evaluate the function	    
			    if (C_eval(C_arg, C_res, C_iw, C_w, C_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
				for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		C(j,i) = C_values[i * sz_res + j];
		        	}
		    	}

		    	// Compute body jacobian
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    J_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (J_eval(J_arg, J_res, J_iw, J_w, J_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
				for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		J_b(j,i) = J_values[i * sz_res + j];
		        	}
		    	}

				// Gravity Comp.
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    G_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (G_eval(G_arg, G_res, G_iw, G_w, G_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
		  	    for (casadi_int i = 0; i < sz_res; ++i) {
		        	G(i) = G_values[i];
		    	}

				// Calculate Joint controller
				e = info.des.q-info.nom.q;
				edot = info.des.q_dot-info.nom.q_dot;
				eint = eint + e*period;

				info.nom.tau_ext = J_b.transpose()*info.act.F;

			    JVec ddq_ref = info.des.q_ddot + Hinf_Kv*edot + Hinf_Kp*e;
			    JVec dq_ref = info.des.q_dot + Hinf_Kv*edot + Hinf_Kp*e;
			    // rt_printf("ddq_ref: %lf, %lf, %lf, %lf, %lf, %lf \n", ddq_ref(0), ddq_ref(1), ddq_ref(2), ddq_ref(3), ddq_ref(4), ddq_ref(5));

			    info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint) - info.nom.tau_ext;
			    // info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
			    // info.nom.tau = M*ddq_ref + C*info.nom.q_dot;

				cnt++;
			}
			else if(cnt==1)
			{
				// 1st stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    temp_q_dot[i] = info.nom.q_dot(i);
				    temp_tau[i] = info.nom.tau(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k1(i) = fd_values[i];
		    	}

		    	JVec _q1 = info.nom.q + 0.5 * period * info.nom.q_dot;
		    	JVec _q_dot1 = info.nom.q_dot + 0.5 * period * k1;
				// JVec _q_dot1 = info.nom.q_dot + 0.5 * period * k1;
				// JVec _q1 = info.nom.q + 0.5 * period * _q_dot1;


			    // 2nd stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q1(i);
				    temp_q_dot[i] = _q_dot1(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k2(i) = fd_values[i];
		    	}

		    	JVec _q2 = info.nom.q + 0.5 * period * _q_dot1;
		    	JVec _q_dot2 = info.nom.q_dot + 0.5 * period * k2;
		    	// JVec _q_dot2 = info.nom.q_dot + 0.5 * period * k2;
		    	// JVec _q2 = info.nom.q + 0.5 * period * _q_dot2;


		    	// 3th stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q2(i);
				    temp_q_dot[i] = _q_dot2(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k3(i) = fd_values[i];
		    	}

		    	JVec _q3 = info.nom.q + period * _q_dot2;
		    	JVec _q_dot3 = info.nom.q_dot + period * k3;
		    	// JVec _q_dot3 = info.nom.q_dot + period * k3;
		    	// JVec _q3 = info.nom.q + period * _q_dot3;

			   	// 4th stage
			    // Set input values
			    for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = _q3(i);
				    temp_q_dot[i] = _q_dot3(i);

				    fd_arg[3*i] = &temp_q[i];
				    fd_arg[3*i + 1] = &temp_q_dot[i];
				    fd_arg[3*i + 2] = &temp_tau[i];
				}

			    // Evaluate the function	    
			    if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	k4(i) = fd_values[i];
		    	}

		    	info.nom.q = info.nom.q + (period / 6.0) * (info.nom.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    	info.nom.q_dot = info.nom.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
				
		    	// info.nom.q_dot = info.nom.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
		    	// info.nom.q = info.nom.q + (period / 6.0) * (_q_dot1 + 2 * (_q_dot2 + _q_dot3) + info.nom.q_dot);

		    	// Compute Mass matrix
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    M_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (M_eval(M_arg, M_res, M_iw, M_w, M_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
				for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		M(j,i) = M_values[i * sz_res + j];
		        	}
		    	}

		    	// Compute C matrix
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    temp_q_dot[i] = info.nom.q_dot(i);

				    C_arg[2*i] = &temp_q[i];				    
				    C_arg[2*i+1] = &temp_q_dot[i];				    
				}
				// Evaluate the function	    
			    if (C_eval(C_arg, C_res, C_iw, C_w, C_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }
				for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		C(j,i) = C_values[i * sz_res + j];
		        	}
		    	}

		    	// Compute body jacobian
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    J_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (J_eval(J_arg, J_res, J_iw, J_w, J_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	for (casadi_int j = 0; j < sz_res; ++j){
		        		J_b(j,i) = J_values[i * sz_res + j];
		        	}
		    	}
		    	
		    	// Gravity Comp.
		    	for (casadi_int i = 0; i < sz_arg; ++i) {
				    temp_q[i] = info.nom.q(i);
				    G_arg[i] = &temp_q[i];				    
				}
				// Evaluate the function	    
			    if (G_eval(G_arg, G_res, G_iw, G_w, G_mem)) {
			        throw std::runtime_error("Function evaluation failed.");
			    }

			    for (casadi_int i = 0; i < sz_res; ++i) {
		        	G(i) = G_values[i];
		    	}
		    	
			    // Calculate Joint controller
				e = info.des.q-info.nom.q;
				edot = info.des.q_dot-info.nom.q_dot;
				eint = eint + e*period;

				info.nom.tau_ext = J_b.transpose()*info.act.F;

			    JVec ddq_ref = info.des.q_ddot + Hinf_Kv*edot + Hinf_Kp*e;
			    JVec dq_ref = info.des.q_dot + Hinf_Kv*edot + Hinf_Kp*e;
			    // rt_printf("ddq_ref: %lf, %lf, %lf, %lf, %lf, %lf \n", ddq_ref(0), ddq_ref(1), ddq_ref(2), ddq_ref(3), ddq_ref(4), ddq_ref(5));

			    info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint) - info.nom.tau_ext;
			    // info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
			    // info.nom.tau = M*ddq_ref + C*info.nom.q_dot;
			}

			endCycle = rt_timer_read();
			periodIndysim = (unsigned long) endCycle - beginCycle;
		
		}
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
#endif


#ifdef __BULLET__
// Bullet task
void bullet_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, 40*cycle_ns);

	//---------BULLET SETUP START------------------
	b3PhysicsClientHandle b3client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(b3client))
	{
	printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	exit(0);
	}
	b3RobotSimulatorClientAPI_InternalData b3data;
	b3data.m_physicsClientHandle = b3client;
	b3data.m_guiHelper = 0;
	b3RobotSimulatorClientAPI_NoDirect b3sim;
	b3sim.setInternalData(&b3data);

	b3sim.setTimeStep(FIXED_TIMESTEP);
	b3sim.resetSimulation();
	b3sim.setGravity( btVector3(0 , 0 ,0));

	int robotId = b3sim.loadURDF("/home/xeno/Indy_ws/Space_Indy7/description/indy7.urdf");
	// int robotId = b3sim.loadURDF("quadruped/minitaur.urdf");
	b3sim.setRealTimeSimulation(false);
	Bullet_Indy7 bt3indy7(&b3sim,robotId);
	
	rt_printf("Start Bullet\n");
	while (1)
	{
		beginCycle = rt_timer_read();
		if(!system_ready)
		{
			bt3indy7.reset_q(&b3sim, info.nom.q);
		}
		else
		{
			bt3indy7.reset_q(&b3sim, info.nom.q);
			b3sim.stepSimulation();

		}
		endCycle = rt_timer_read();
		periodBullet = (unsigned long) endCycle - beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
#endif

// Safety task
void safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NUM_AXIS;i++)
			{
				if(Axis[i].isLimitReached())
				{
					for(int i=0;i<NUM_AXIS;i++)
						nrmk_master.setServoOff(i+NUM_IO_MODULE);
					rt_printf("Servo Off!!\n");
					break;
				}
			}
		}
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*400);
	
	string filename = "robot_log.csv";
	ifstream checkFile(filename);

	if (checkFile.is_open())
	{
		checkFile.close();
		remove(filename.c_str());
	}

	ofstream newFile(filename);
	if(newFile.is_open())
	{
		newFile<<"Time, q_r1, q_r2, q_r3, q_r4, q_r5, q_r6, q_r3, dq_r1, dq_r2, dq_r3, dq_r4, dq_r5, dq_r6, t_r1, t_r2, t_r3, t_r4, t_r5, t_r6, G_r1, G_r2, G_r3, G_r4, G_r5, G_r6, "
		"q_n1, q_n2, q_n3, q_n4, q_n5, q_n6, q_n3, dq_n1, dq_n2, dq_n3, dq_n4, dq_n5, dq_n6, t_n1, t_n2, t_n3, t_n4, t_n5, t_n6, "
		"qd1, qd2, qd3, qd4, qd5, qd6, qd3, dqd1, dqd2, dqd3, dqd4, dqd5, dqd6\n";
		newFile.close();
	}


	ofstream csvFile(filename, ios_base::app);


	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;
			if (!nrmk_master.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Offline\n");
			if (!nrmk_master.getRxDomainStatus())
				rt_printf("RxDomain: Offline\n");
			if (!nrmk_master.getTxDomainStatus())
				rt_printf("TxDomain: Offline\n");
			// for(int i=0;i<NUM_AXIS;i++){
			// 	if (!nrmk_master.getAxisEcatStatus(i+NUM_IO_MODULE,slaveState[i]))
			// 	rt_printf("idx: %u, slaveState: %u\n",i+NUM_IO_MODULE,slaveState[i]);	
			// }
			for(int i=1;i<=NUM_AXIS;i++){
				if (!nrmk_master.getAxisEcatStatus(i,slaveState[i-1]))
				rt_printf("idx: %u, slaveState: %u",i,slaveState[i-1]);	
			}
			
			rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
			rt_printf("compute_dt= %lius, worst_dt= %lius, buffer_dt=%lius, ethercat_dt= %lius\n", periodCompute/1000, worstCompute/1000, periodBuffer/1000, periodEcat/1000);
			#ifdef __BULLET__
			rt_printf("Bullet_dt=%lius\n",periodBullet/1000);
			#endif
			#ifdef __CASADI__
			rt_printf("IndySim_dt=%lius\n",periodIndysim/1000);
			#endif
			for(int j=0; j<NUM_AXIS; ++j){
				rt_printf("ID: %d", j);
			// 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf \n",info.nom.q(j), info.nom.q_dot(j));
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
			// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
				// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
				rt_printf("\t TarTor: %f, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j), info.act.tau_ext(j));
			}

			rt_printf("ReadFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F(0),info.act.F(1),info.act.F(2),info.act.F(3),info.act.F(4),info.act.F(5));
			rt_printf("ReadFT_CB: %lf, %lf, %lf, %lf, %lf, %lf\n", info.act.F_CB(0),info.act.F_CB(1),info.act.F_CB(2),info.act.F_CB(3),info.act.F_CB(4),info.act.F_CB(5));
			rt_printf("overload: %u, error: %u\n", FTOverloadStatus[NUM_IO_MODULE+NUM_AXIS], FTErrorFlag[NUM_IO_MODULE+NUM_AXIS]);

#ifdef __CASADI__
			// indy7_M();
			// indy7_Minv();
		    // indy7_C();
		    // indy7_G();
		    // indy7_J_b();
		    // indy7_J_s();
            // indy7_FK();
            // indy7_FD();
            // indy7_FD_rk();
#endif

			rt_printf("\n");



		if(csvFile.is_open())
		{
			csvFile<<gt<<", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.act.q(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.act.q_dot(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.des.tau(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.act.F(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.nom.q(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.nom.q_dot(i) << ", ";				
			for (int i = 0; i < 6; ++i) csvFile<<info.nom.tau(i) << ", ";
			for (int i = 0; i < 6; ++i) csvFile<<info.des.q(i) << ", ";
			for (int i = 0; i < 5; ++i) csvFile<<info.des.q_dot(i) << ", ";
			csvFile<<info.des.q_dot(5)<<"\n";
		}
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
	}
	csvFile.close();
}


/****************************************************************************/
void signal_handler(int signum)
{
	rt_task_delete(&RTIndy7_task);
	
	rt_task_delete(&safety_task);
	rt_task_delete(&print_task);
#ifdef __CASADI__
	rt_task_delete(&indysim_task);
#endif
#ifdef __BULLET__
	rt_task_delete(&bullet_task);
#endif
	// FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_STOP_DEVICE;
	// FTConfigParamCB[0]=FT_STOP_DEVICE;
	// nrmk_master.writeBuffer(0x70003, FTConfigParam);
	// nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
	// nrmk_master.processRxDomain();

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


/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	// cycle_ns = 1000000; // nanosecond -> 1kHz
	cycle_ns = 250000; // nanosecond -> 4kHz
	// cycle_ns = 125000; // nanosecond -> 8kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	mr_indy7=MR_Indy7();
	mr_indy7.MRSetup();
	MAX_TORQUES<<MAX_TORQUE_1,MAX_TORQUE_2,MAX_TORQUE_3,MAX_TORQUE_4,MAX_TORQUE_5,MAX_TORQUE_6;

	// For CST (cyclic synchronous torque) control
	if (nrmk_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
	{
		printf("System Initialization Failed\n");
	    return 0;
	}
	for (int i = 0; i < NUM_AXIS; ++i)
		ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;

	// For trajectory interpolation
	initAxes();

	for(int i=0;i<NUM_SLAVES;i++)
		nrmk_master.setServoOn(i);
	
	// RTIndy7_task: create and start
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 99, 0);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

#ifdef __BULLET__
	// RTIndy7 simulation
	rt_task_create(&bullet_task, "bullet_task", 0, 80, 0);
	rt_task_start(&bullet_task, &bullet_run, NULL);
#endif

#ifdef __CASADI__
	// RTIndy7 simulation
	rt_task_create(&indysim_task, "indysim_task", 0, 96, 0);
	rt_task_start(&indysim_task, &indysim_run, NULL);
#endif

	// RTIndy7 safety
	rt_task_create(&safety_task, "safety_task", 0, 90, 0);
	rt_task_start(&safety_task, &safety_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
	pause();
	/*
	while (1)
	{
		usleep(1e5);
	}
	*/
	// Finalize
	signal_handler(0);

    return 0;
}



