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


	int indy7_G()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_G.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_G.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "generalized_gravity");
	    if (dlerror()) {
	        printf("Failed to retrieve \"generalized_gravity\" function.\n");
	        return 1;
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
	    // double input_values[] = {info.act.q(0),info.act.q(1),info.act.q(2),info.act.q(3),info.act.q(4),info.act.q(5)};
	    double input_values[] = {-0.128198, -0.059945, 1.243974, 3.058833, -2.110628, 0.131757};
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
	    start = rt_timer_read();
	    if (eval(arg, res, iw, w, mem)) {
	        printf("Function evaluation failed.\n");
	        return 1;
	    }
	    end = rt_timer_read();

	    // Print the result
	    rt_printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        rt_printf("%lf ", output_values[i]);
	    }
	    rt_printf("\n");
	    rt_printf("[cs]computation time for \"G\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    JVec G = mr_indy7.Gvec(info.act.q);
	    end = rt_timer_read();
	    rt_printf("%lf, %lf, %lf, %lf, %lf, %lf \n", G(0),G(1),G(2),G(3),G(4),G(5));
	    rt_printf("[mr]computation time for \"G\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}
	int indy7_M()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_M.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_M.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "M");
	    if (dlerror()) {
	        printf("Failed to retrieve \"M\" function.\n");
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
	    rt_printf("[cs]computation time for \"M\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    mr_indy7.Mmat(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"M\": %lius\n", (end-start)/1000);


	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_Minv()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_Minv.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_Minv.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "Minv");
	    if (dlerror()) {
	        printf("Failed to retrieve \"Minv\" function.\n");
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
	    double input_values[] = {-0.128198, -0.059945, 1.243974, 3.058833, -2.110628, 0.131757};
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
	    printf("Result:\n");
	    for (casadi_int i = 0; i < sz_res; ++i) {
	        for (casadi_int j = 0; j < sz_res; ++j) {
	            printf("%lf ", output_values[i * sz_res + j]);
	        }
	        printf("\n");
	    }
	    rt_printf("[cs]computation time for \"Minv\": %lius\n", (end-start)/1000);
	    
	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_C()
	{
		RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_C.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_C.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "coriolis");
	    if (dlerror()) {
	        printf("Failed to retrieve \"C\" function.\n");
	        return 1;
	    }

	    // Allocate input/output buffers and work vectors
	    casadi_int sz_arg = 6;
	    casadi_int sz_res = 6;
	    casadi_int sz_iw = 0;
	    casadi_int sz_w = 0;

	    const double* arg[12];
	    double* res[6];
	    casadi_int iw[sz_iw];
	    double w[sz_w];

	    // Set input values
	    double input_pos[] = {-0.128198, -0.059945, 1.243974, 3.058833, -2.110628, 0.131757};
	    double input_vel[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
	        printf("Function evaluation failed.\n");
	        return 1;
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
		rt_printf("[cs]computation time for \"C\": %lius\n", (end-start)/1000);
	    
	    start = rt_timer_read();
	    mr_indy7.Cvec(info.act.q, info.act.q_dot);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"C\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
	}

	int indy7_FK()
	{
	    RTIME start, end;
	// Load the shared library
	    void* handle = dlopen("../lib/URDF2CASADI/indy7_fk.so", RTLD_LAZY);
	    if (handle == 0) {
	        printf("Cannot open indy7_fk.so, error: %s\n", dlerror());
	        return 1;
	    }

	    // Reset error
	    dlerror();

	    // Function evaluation
	    eval_t eval = (eval_t)dlsym(handle, "fk_T");
	    if (dlerror()) {
	        printf("Failed to retrieve \"fk_T\" function.\n");
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
	    rt_printf("[cs]computation time for \"FK\": %lius\n", (end-start)/1000);
	    start = rt_timer_read();
	    mr_indy7.T_s(info.act.q);
	    end = rt_timer_read();
	    rt_printf("[mr]computation time for \"FK\": %lius\n", (end-start)/1000);

	    // Free the handle
	    dlclose(handle);

	    return 0;
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
	    double* res[6];
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
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;
	    	traj_time = 3.0;
	    	motion=1;
	    	break;
	    // default:
	    // 	motion=1;
	    // 	break;
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
	}

	// Update RFT data
	info.act.F(0) = (double)FTRawFx[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(1) = (double)FTRawFy[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(2) = (double)FTRawFz[NUM_IO_MODULE+NUM_AXIS] / force_divider;
	info.act.F(3) = (double)FTRawTx[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(4) = (double)FTRawTy[NUM_IO_MODULE+NUM_AXIS] / torque_divider;
	info.act.F(5) = (double)FTRawTz[NUM_IO_MODULE+NUM_AXIS] / torque_divider;

	info.act.F_CB(0) = (double)FTRawFxCB[0] / force_divider;
	info.act.F_CB(1) = (double)FTRawFyCB[0] / force_divider;
	info.act.F_CB(2) = (double)FTRawFzCB[0] / force_divider;
	info.act.F_CB(3) = (double)FTRawTxCB[0] / torque_divider;
	info.act.F_CB(4) = (double)FTRawTyCB[0] / torque_divider;
	info.act.F_CB(5) = (double)FTRawTzCB[0] / torque_divider;

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
			eint = eint + e*(double)0.001;
			
			info.des.tau = mr_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot,eint) + info.act.tau_ext;
			
			// info.des.tau = mr_indy7.HinfControl( info.act.q , info.act.q_dot, info.des.q, info.des.q_dot,info.des.q_ddot,eint);
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
				FTConfigParamCB[0]=FT_SET_BIAS;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Set Filter 100Hz
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_SET_FILTER_50;
				FTConfigParamCB[0]=FT_SET_FILTER_50;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
				nrmk_master.processRxDomain();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Start
				FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_START_DEVICE;
				FTConfigParamCB[0]=FT_START_DEVICE;
				nrmk_master.writeBuffer(0x70003, FTConfigParam);
				nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
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

	double dt=((double) cycle_ns)/((double) NSEC_PER_SEC);
	// Load the shared library
    void* fd_handle = dlopen("../lib/URDF2CASADI/indy7_fd.so", RTLD_LAZY);
    if (fd_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fd.so");
    }
    void* G_handle = dlopen("../lib/URDF2CASADI/indy7_G.so", RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open indy7_G.so");
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
    eval_t J_eval = (eval_t)dlsym(J_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
   
    // Allocate input/output buffers and work vectors dlrj
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* fd_arg[18];
    const double* G_arg[6];
    const double* J_arg[6];
    double* fd_res[6];
    double* G_res[6];
    double* J_res[36];
    casadi_int fd_iw[sz_iw], G_iw[sz_iw], J_iw[sz_iw];
    double fd_w[sz_w], G_w[sz_w], J_w[sz_w];
    int fd_mem = 0;  // No thread-local memory management
    int G_mem = 0;  // No thread-local memory management
    int J_mem = 0;  // No thread-local memory management
    

    double temp_q[6], temp_q_dot[6], temp_tau[6];
    
    JVec k1, k2, k3, k4;
    Jacobian J_b;

    // Set output buffers
    double fd_values[6], G_values[6], J_values[36];
    for (casadi_int i = 0; i < 6; ++i) {
        fd_res[i] = &fd_values[i];
        G_res[i] = &G_values[i];
    }
    for (casadi_int i = 0; i < 36; ++i) {
        J_res[i] = &J_values[i];
    }

	int cnt = 0;    

    // // Free the handle
    // dlclose(handle);

	// loop
	while(1)
	{
		if(system_ready)
		{
			beginCycle = rt_timer_read();

			////////////////   Implicit Euler method   /////////////////
			


			/////////////////  RK4   //////////////////
			// if(cnt == 0)
			// {
			// 	// state update
			// 	JVec _q = info.act.q;
			// 	JVec _q_dot = info.act.q_dot;
			// 	JVec _tau = info.act.tau;

			// 	// 1st stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q(i);
			// 	    temp_q_dot[i] = _q_dot(i);
			// 	    temp_tau[i] = _tau(i);
			// 	    // temp_tau[i] = 0.0;

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}
				
			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k1(i) = fd_values[i];
		    // 	}
		    	
		    // 	JVec _q1 = info.act.q + 0.5 * dt * info.act.q_dot;
		    // 	JVec _q_dot1 = info.act.q_dot + 0.5 * dt * k1;

			//     // 2nd stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q1(i);
			// 	    temp_q_dot[i] = _q_dot1(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i+1] = &temp_q_dot[i];
			// 	    fd_arg[3*i+2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k2(i) = fd_values[i];
		    // 	}
		    // 	JVec _q2 = info.act.q + 0.5 * dt * _q_dot1;
		    // 	JVec _q_dot2 = info.act.q_dot + 0.5 * dt * k2;

		    // 	// 3th stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q2(i);
			// 	    temp_q_dot[i] = _q_dot2(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k3(i) = fd_values[i];
		    // 	}
		    // 	JVec _q3 = info.act.q + dt * _q_dot2;
		    // 	JVec _q_dot3 = info.act.q_dot + dt * k3;

			//    	// 4th stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q3(i);
			// 	    temp_q_dot[i] = _q_dot3(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k4(i) = fd_values[i];
		    // 	}

		    // 	info.nom.q = info.act.q + (dt / 6.0) * (info.act.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    // 	info.nom.q_dot = info.act.q_dot + (dt / 6.0) * (k1 + 2 * (k2 + k3) + k4);
				

		    // 	// Compute body jacobian
		    // 	for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = info.nom.q(i);
			// 	    J_arg[i] = &temp_q[i];				    
			// 	}
			// 	// Evaluate the function	    
			//     if (J_eval(J_arg, J_res, J_iw, J_w, J_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	for (casadi_int j = 0; j < sz_res; ++j){
		    //     		J_b(j,i) = J_values[i * sz_res + j];
		    //     	}
		    //     	rt_printf("J: %lf, %lf, %lf, %lf, %lf, %lf \n", J_b(0,i), J_b(1,i), J_b(2,i), J_b(3,i), J_b(4,i), J_b(5,i));
		    // 	}


			// 	// Gravity Comp.
		    // 	for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = info.nom.q(i);
			// 	    G_arg[i] = &temp_q[i];				    
			// 	}
			// 	// Evaluate the function	    
			//     if (G_eval(G_arg, G_res, G_iw, G_w, G_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	info.nom.tau(i) = G_values[i];
		    // 	}

			// 	cnt++;
			// }
			// else
			// {
			// 	// 1st stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = info.nom.q(i);
			// 	    temp_q_dot[i] = info.nom.q_dot(i);
			// 	    temp_tau[i] = info.nom.tau(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k1(i) = fd_values[i];
		    // 	}

		    // 	JVec _q1 = info.nom.q + 0.5 * dt * info.nom.q_dot;
		    // 	JVec _q_dot1 = info.nom.q_dot + 0.5 * dt * k1;
		    // 	JVec _q_dot_tmp = 0.5 * dt * k1;

			//     // 2nd stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q1(i);
			// 	    temp_q_dot[i] = _q_dot1(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k2(i) = fd_values[i];
		    // 	}

		    // 	JVec _q2 = info.nom.q + 0.5 * dt * _q_dot1;
		    // 	JVec _q_dot2 = info.nom.q_dot + 0.5 * dt * k2;

		    // 	// 3th stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q2(i);
			// 	    temp_q_dot[i] = _q_dot2(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k3(i) = fd_values[i];
		    // 	}

		    // 	JVec _q3 = info.nom.q + dt * _q_dot2;
		    // 	JVec _q_dot3 = info.nom.q_dot + dt * k3;

			//    	// 4th stage
			//     // Set input values
			//     for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = _q3(i);
			// 	    temp_q_dot[i] = _q_dot3(i);

			// 	    fd_arg[3*i] = &temp_q[i];
			// 	    fd_arg[3*i + 1] = &temp_q_dot[i];
			// 	    fd_arg[3*i + 2] = &temp_tau[i];
			// 	}

			//     // Evaluate the function	    
			//     if (fd_eval(fd_arg, fd_res, fd_iw, fd_w, fd_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	k4(i) = fd_values[i];
		    // 	}

		    // 	info.nom.q = info.nom.q + (dt / 6.0) * (info.nom.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    // 	info.nom.q_dot = info.nom.q_dot + (dt / 6.0) * (k1 + 2 * (k2 + k3) + k4);

		    // 	// Compute body jacobian
		    // 	for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = info.nom.q(i);
			// 	    J_arg[i] = &temp_q[i];				    
			// 	}
			// 	// Evaluate the function	    
			//     if (J_eval(J_arg, J_res, J_iw, J_w, J_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	for (casadi_int j = 0; j < sz_res; ++j){
		    //     		J_b(j,i) = J_values[i * sz_res + j];
		    //     	}
		    // 	}

		    // 	info.nom.tau_ext = J_b.transpose()*info.act.F;
		    // 	// rt_printf("tau_ext: %lf, %lf, %lf, %lf, %lf, %lf \n", tau_ext(0),tau_ext(1),tau_ext(2),tau_ext(3),tau_ext(4),tau_ext(5));

		    // 	// Gravity Comp.
		    // 	for (casadi_int i = 0; i < sz_arg; ++i) {
			// 	    temp_q[i] = info.nom.q(i);
			// 	    G_arg[i] = &temp_q[i];				    
			// 	}
			// 	// Evaluate the function	    
			//     if (G_eval(G_arg, G_res, G_iw, G_w, G_mem)) {
			//         throw std::runtime_error("Function evaluation failed.");
			//     }

			//     for (casadi_int i = 0; i < sz_res; ++i) {
		    //     	info.nom.tau(i) = G_values[i] - info.nom.tau_ext(i);
		    // 	}
		    // 	// rt_printf("G: %lf, %lf, %lf, %lf, %lf, %lf \n", info.nom.tau(0),info.nom.tau(1),info.nom.tau(2),info.nom.tau(3),info.nom.tau(4),info.nom.tau(5));
			// }
			


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
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	//---------BULLET SETUP START------------------
	kPhysClient = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!kPhysClient)
	{
		printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
		exit(0);
	}
	
	command = b3InitConfigureOpenGLVisualizer(kPhysClient);
	b3ConfigureOpenGLVisualizerSetVisualizationFlags(command, COV_ENABLE_GUI, 0);
	b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	b3ConfigureOpenGLVisualizerSetVisualizationFlags(command, COV_ENABLE_SHADOWS, 0);
	b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	b3SetTimeOut(kPhysClient, 10);

	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	command = b3InitSyncBodyInfoCommand(kPhysClient);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	statusType = b3GetStatusType(statusHandle);

	// set fixed time step
	command = b3InitPhysicsParamCommand(kPhysClient);
	ret = b3PhysicsParamSetTimeStep(command, FIXED_TIMESTEP);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	b3InitResetSimulationCommand(kPhysClient);
	ret = b3PhysicsParamSetRealTimeSimulation(command, false);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	ret = b3PhysicsParamSetGravity(command,0,0,-9.8);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);

	// load test
	// command = b3LoadUrdfCommandInit(kPhysClient, "TwoJointRobot_wo_fixedJoints.urdf");
	command = b3LoadUrdfCommandInit(kPhysClient, "/home/xeno/Indy_ws/RTIndy7/description/indy7.urdf");
	int flags = URDF_USE_INERTIA_FROM_FILE;
	b3LoadUrdfCommandSetFlags(command, flags);
	b3LoadUrdfCommandSetUseFixedBase(command, true);
	// q.setEulerZYX(0, 0, 0);
	// b3LoadUrdfCommandSetStartOrientation(command, q[0], q[1], q[2], q[3]);
	b3LoadUrdfCommandSetUseMultiBody(command, true);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	statusType = b3GetStatusType(statusHandle);
	b3Assert(statusType == CMD_URDF_LOADING_COMPLETED);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		twojoint = b3GetStatusBodyIndex(statusHandle);
	}

	//disable default linear/angular damping
	b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(kPhysClient);
	double linearDamping = 0;
	double angularDamping = 0;
	b3ChangeDynamicsInfoSetLinearDamping(command, twojoint, linearDamping);
	b3ChangeDynamicsInfoSetAngularDamping(command, twojoint, angularDamping);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	int numJoints = b3GetNumJoints(kPhysClient, twojoint);
	printf("twojoint numjoints = %d\n", numJoints);
	// Loop through all joints
	for (int i = 0; i < numJoints; ++i)
	{
		b3GetJointInfo(kPhysClient, twojoint, i, &jointInfo[i]);
		if (jointInfo[i].m_jointName[0] && jointInfo[i].m_jointType!=eFixedType)
		{
			jointNameToId[std::string(jointInfo[i].m_jointName)] = i;
			actuated_joint_name.push_back(jointInfo[i].m_jointName);
			actuated_joint_id.push_back(i);	
			actuated_joint_num++;
		}
		else
		{
			jointNameToId[std::string(jointInfo[i].m_jointName)] = i;
		}
		// Reset before torque control - see #1459
		command = b3JointControlCommandInit2(kPhysClient, twojoint, CONTROL_MODE_VELOCITY);
		b3JointControlSetDesiredVelocity(command, jointInfo[i].m_uIndex, 0);
		b3JointControlSetMaximumForce(command, jointInfo[i].m_uIndex, 0);
		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	}

	// loop
	while (1)
	{
		beginCycle = rt_timer_read();

		// get joint values
		command = b3RequestActualStateCommandInit(kPhysClient, twojoint);
		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
		for (int i = 0; i < actuated_joint_num; ++i)
        {
            int jointIndex = actuated_joint_id[i];
            b3GetJointState(kPhysClient, statusHandle, jointIndex, &b3state);
            info.nom.q(i) = b3state.m_jointPosition;
            info.nom.q_dot(i) = b3state.m_jointVelocity;
            info.nom.tau(i) = b3state.m_jointMotorTorque;
        }

		// apply some torque
        for (int i = 0; i < actuated_joint_num; ++i)
        {
            int jointIndex = actuated_joint_id[i];
            b3GetJointInfo(kPhysClient, twojoint, jointIndex, &jointInfo[jointIndex]);
            command = b3JointControlCommandInit2(kPhysClient, twojoint, CONTROL_MODE_TORQUE);
            b3JointControlSetDesiredForceTorque(command, jointInfo[jointIndex].m_uIndex, info.des.tau(i));
            statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
        }

		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, b3InitStepSimulationCommand(kPhysClient));

		endCycle = rt_timer_read();
		periodBullet = (unsigned long) endCycle - beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle

	}

	// b3RobotSimulatorClientAPI_InternalData b3data;
	// b3data.m_physicsClientHandle = b3client;
	// b3data.m_guiHelper = 0;
	// b3RobotSimulatorClientAPI_NoDirect b3sim;
	// b3sim.setInternalData(&b3data);
	// b3sim.setTimeStep(FIXED_TIMESTEP);
	// b3sim.resetSimulation();
	// b3sim.setGravity( btVector3(0 , 0 ,0));

	// int robotId = b3sim.loadURDF("/home/xeno/Indy_ws/RTIndy7/description/indy7.urdf");
	// // int robotId = b3sim.loadURDF("quadruped/minitaur.urdf");
	// b3sim.setRealTimeSimulation(false);
	// Bullet_Indy7 bt3indy7(&b3sim,robotId);
	// // indy7.reset_q(&b3sim, info.act.q);

	// info.nom.q = JVec::Zero();
	// info.nom.q_dot = JVec::Zero();
	// info.nom.q_ddot = JVec::Zero();
	// info.nom.F = Vector6d::Zero();
	// info.nom.F_CB = Vector6d::Zero();

	// rt_printf("Start Bullet\n");
	// while (1)
	// {
	// 	beginCycle = rt_timer_read();
	// 	if(!system_ready)
	// 	{
	// 		bt3indy7.reset_q(&b3sim, info.act.q);
	// 	}
	// 	else
	// 	{
	// 		info.nom.q = bt3indy7.get_q(&b3sim);
	// 		info.nom.q_dot = bt3indy7.get_qdot(&b3sim);
	// 		// bt3indy7.reset_q(&b3sim, info.act.q);
	// 		bt3indy7.set_torque(&b3sim,  info.des.tau , MAX_TORQUES);
	// 		b3sim.stepSimulation();

	// 		// rt_printf("q_sim: %f, %f, %f, %f, %f, %f, \n", info.nom.q(0),info.nom.q(1),info.nom.q(2),info.nom.q(3),info.nom.q(4),info.nom.q(5));
	// 	}
	// 	endCycle = rt_timer_read();
	// 	periodBullet = (unsigned long) endCycle - beginCycle;
	// 	rt_task_wait_period(NULL); //wait for next cycle
	// }
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
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
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
	b3DisconnectSharedMemory(kPhysClient);
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
	cycle_ns = 1000000; // nanosecond -> 1kHz
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
	rt_task_create(&bullet_task, "bullet_task", 0, 95, 0);
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



