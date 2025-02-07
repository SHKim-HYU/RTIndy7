#include "CS_Indy7.h"


CS_Indy7::CS_Indy7()
{
    robotModel = "indy7";
    n_dof = 6;

    Task_Kp = Matrix6d::Zero();
    Task_Kv = Matrix6d::Zero();
    Task_Ki = Matrix6d::Zero();
    Task_K = JMat::Zero();

    Hinf_Kp = JMat::Zero();
    Hinf_Kv = JMat::Zero();
    Hinf_Ki = JMat::Zero();
    Hinf_K_gamma = JMat::Zero();

    NRIC_Kp = JMat::Zero();
    NRIC_Ki = JMat::Zero();
    NRIC_K_gamma = JMat::Zero();

    Kp = JMat::Zero();
    Kv = JMat::Zero();
    K = JMat::Zero();

    e = JVec::Zero();
    eint = JVec::Zero();

    r_floor << X_com, Y_com, Z_com;
    r_ceil = VecToso3(r_floor); 
    G_tool << 0, 0, -mass_tool*9.8, 0, 0, 0;
    
    for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_4 ;

            break;
        case 4:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_6 ;

            break;
#ifdef __RP__
        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_7 ;

            break;
#endif
        }
    }
   for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            K(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 75.0;
            Kv(i,i) = 55.0;
            K(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            K(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            K(i,i)=1.0;
            break;
        case 6:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            K(i,i)=1.0;
            break;
        }
    }

    
}

void CS_Indy7::CSSetup(const string& _modelPath, double _period)// : loader_(_modelPath), period[sec]
{
	JsonLoader loader_ = JsonLoader(_modelPath);

	robotModel = loader_.getValue("name").asString();
	n_dof = std::stoi(loader_.getValue("n_dof").asString());
    period = _period;

    lambda_int = Twist::Zero();
    gamma_int = Twist::Zero();
    F_eff_int = Twist::Zero();

    r_floor << X_com, Y_com, Z_com;
    r_ceil = VecToso3(r_floor); 
    G_tool << 0, 0, -mass_tool*9.8, 0, 0, 0;
	G_FT << 0, 0, -mass_FT*9.8, 0, 0, 0;

    A_tool=Matrix6d::Zero(); B_tool=Matrix6d::Zero();
    A_tool(0,0) = mass_tool; A_tool(1,1) = mass_tool; A_tool(2,2) = mass_tool;
    A_tool(3,3) = Ixx; A_tool(4,4) = Iyy; A_tool(5,5) = Izz; 

    for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_4 ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_6 ;

            break;
#ifdef __RP__
        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_7 ;

            break;
#endif
        }
    }
   for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            K(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 75.0;
            Kv(i,i) = 55.0;
            K(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            K(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=1.0;
            break;
        case 6:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            K(i,i)=1.0;
            break;
        }
    }
	
	// Friction Parameters
    Fc << F_c;
    Fv1 << F_v1;
    Fv2 << F_v2; 

	// Load the shared library
    string casadi_path = "../lib/URDF2CASADI/";
    casadi_path = casadi_path + robotModel + "/" + robotModel;

    string func_path = casadi_path + "_fd.so";
    FD_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FD_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fd.so");
    }
    func_path = casadi_path + "_M.so";
    M_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (M_handle == 0) {
        throw std::runtime_error("Cannot open indy7_M.so");
    }
    func_path = casadi_path + "_Minv.so";
    Minv_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (Minv_handle == 0) {
        throw std::runtime_error("Cannot open indy7_Minv.so");
    }
    func_path = casadi_path + "_C.so";
    C_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (C_handle == 0) {
        throw std::runtime_error("Cannot open indy7_C.so");
    }
    func_path = casadi_path + "_G.so";
    G_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open indy7_G.so");
    }
    func_path = casadi_path + "_fk_ee.so";
    FK_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FK_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fk_ee.so");
    }
    func_path = casadi_path + "_J_b.so";
    J_b_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_b_handle == 0) {
        throw std::runtime_error("Cannot open indy7_J_b.so");
    }
    func_path = casadi_path + "_J_s.so";
    J_s_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_s_handle == 0) {
        throw std::runtime_error("Cannot open indy7_J_s.so");
    }
    func_path = casadi_path + "_dJ_b.so";
    dJ_b_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (dJ_b_handle == 0) {
        throw std::runtime_error("Cannot open indy7_dJ_b.so");
    }
    func_path = casadi_path + "_dJ_s.so";
    dJ_s_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (dJ_s_handle == 0) {
        throw std::runtime_error("Cannot open indy7_dJ_s.so");
    }

    // Reset error
    dlerror();
    // Function evaluation
    FD_eval = (eval_t)dlsym(FD_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    M_eval = (eval_t)dlsym(M_handle, "M");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    Minv_eval = (eval_t)dlsym(Minv_handle, "Minv");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

    C_eval = (eval_t)dlsym(C_handle, "coriolis");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    FK_eval = (eval_t)dlsym(FK_handle, "fk_T");
    if (dlerror()) {
        throw std::runtime_error("Failed to retrieve \"fk_T\" function.\n");
    }
    J_b_eval = (eval_t)dlsym(J_b_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    J_s_eval = (eval_t)dlsym(J_s_handle, "J_s");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    dJ_b_eval = (eval_t)dlsym(dJ_b_handle, "dJ_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    dJ_s_eval = (eval_t)dlsym(dJ_s_handle, "dJ_s");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
}

void CS_Indy7::setPIDgain(JVec _Kp, JVec _Kd, JVec _K)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        Kp(i,i) = _Kp(i);
        Kv(i,i) = _Kd(i);
        K(i,i) = _K(i);
    }
}

void CS_Indy7::setHinfgain(JVec _Hinf_Kp, JVec _Hinf_Kd, JVec _Hinf_Ki, JVec _Hinf_K_gamma)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        Hinf_Kp(i,i) = _Hinf_Kp(i);
        Hinf_Kv(i,i) = _Hinf_Kd(i);
        Hinf_Ki(i,i) = _Hinf_Ki(i);
        Hinf_K_gamma(i,i) = _Hinf_K_gamma(i);
    }
}

void CS_Indy7::setNRICgain(JVec _NRIC_Kp, JVec _NRIC_Ki, JVec _NRIC_K, JVec _NRIC_gamma)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        NRIC_Kp(i,i) = _NRIC_Kp(i);
        NRIC_Ki(i,i) = _NRIC_Ki(i);
        NRIC_K_gamma(i,i) = _NRIC_K(i)+1/ _NRIC_gamma(i);
    }
}

void CS_Indy7::setTaskgain(Twist _Kp, Twist _Kv, JVec _K)
{
    Task_Kp = _Kp.asDiagonal();
    Task_Kv = _Kv.asDiagonal();
    Task_K = _K.asDiagonal();
}

void CS_Indy7::setTaskImpedancegain(Matrix6d _A, Matrix6d _D, Matrix6d _K)
{
    A_ = _A;
    D_ = _D;
    K_ = _K;
}

void CS_Indy7::updateRobot(JVec _q, JVec _dq, JVec _ddq)
{
    q=_q; dq=_dq; ddq=_ddq;

    M = computeM(_q);
    Minv = computeMinv(_q);
    C = computeC(_q, _dq);
    G = computeG(_q); 

    J_b = computeJ_b(_q);
    manipulability = computeManipulability(_q);
    dJ_b = computeJdot_b(_q, _dq);

    T_ee = computeFK(_q);

    V_b = J_b*_dq;
    V_dot = J_b*_ddq + dJ_b*_dq;

    isUpdated=true;   
}

JVec CS_Indy7::computeFD(JVec _q, JVec _dq, JVec _tau)
{
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[3*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    double input_tau[sz_arg];

    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        input_tau[i] = _tau(i);
        arg[3*i] = &input_pos[i];
        arg[3*i+1] = &input_vel[i];
        arg[3*i+2] = &input_tau[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x1 Vector
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FD_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        // if(!isnan(output_values[i]))
            ddq_res(i) = output_values[i];
        // else
        //     cout<<[ERROR NaN]<<endl;
    }

    // // fd method
    // Minv = computeMinv(_q);
    // C = computeC(_q,_dq);
    // G = computeG(_q);

    // ddq_res = Minv*(_tau-C*_dq-G);
    
    return ddq_res;

}

void CS_Indy7::computeRK45(JVec _q, JVec _dq, JVec _tau, JVec &_q_nom, JVec &_dq_nom, JVec &_ddq_nom)
{
    JVec k1, k2, k3, k4;

    // state update
    JVec _q0 = _q;
    // JVec _q_dot = info.act.q_dot;
    JVec _q_dot0 = _dq;
    // JVec _tau = _tau;

    // 1st stage
    k1 = computeFD(_q, _q_dot0, _tau);
    JVec _q1 = _q + 0.5 * period * _dq;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    JVec _q_dot1 = _dq + 0.5 * period * k1; // k2(q_dot)
    
    // 2nd stage
    k2 = computeFD(_q1, _q_dot1, _tau);
    JVec _q2 = _q + 0.5 * period * _q_dot1;
    JVec _q_dot2 = _dq + 0.5 * period * k2;
    
    // 3th stage
    k3 = computeFD(_q2, _q_dot2, _tau);
    JVec _q3 = _q + period * _q_dot2;
    JVec _q_dot3 = _dq + period * k3;
    
    // 4th stage
    k4 = computeFD(_q3, _q_dot3, _tau);
    _q_nom = _q + (period / 6.0) * (_dq + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
    _dq_nom = _dq + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
    _ddq_nom = k1;
}

Twist CS_Indy7::computeF_Tool(Twist _dx, Twist _ddx)
{
    Twist res;
    Matrix6d adj = ad(_dx);
    Matrix6d AdT = Matrix6d::Zero(); 

	AdT.block<3,3>(0,0) = R_ee.transpose();
    F_FT = AdT * G_FT;
    AdT.block<3,3>(3,3) = R_ee.transpose();
    AdT.block<3,3>(3,0) = -R_ee.transpose()*r_ceil;
    
    B_tool = A_tool*adj - adj.transpose()*A_tool;

    res = A_tool*_ddx + B_tool*_dx + F_FT + AdT*G_tool;
    return res;
}

Twist CS_Indy7::computeF_Threshold(Twist _F)
{
    Twist res;

    for(int i=0; i<6; i++)
    {
		if(i<3)
		{
			if(0.8>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=_F(i);
			}
		}
		else if(i==5)
		{
			if(0.005>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=1*_F(i);
				// res(i)=0.0;
			}
		}
		else 
		{
			if(0.05>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=1*_F(i);
			}
		}
    }
    return res;
}

MassMat CS_Indy7::computeM(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> M_res = M_cs(arg);

    // return M;
    
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (M_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            M(j,i) = output_values[i * sz_res + j];
        }
    }

    return M;
}

MassMat CS_Indy7::computeMinv(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> Minv_res = Minv_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (Minv_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            Minv(j,i) = output_values[i * sz_res + j];
        }
    }

    return Minv;
}

MassMat CS_Indy7::computeC(JVec _q, JVec _dq)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // casadi::DM dq_dm = casadi::DM(vector<double>(_dq.data(), _dq.data() + _dq.size()));
    // map<string, casadi::DM> arg;
    // arg["q"] = q_dm; arg["dq"] = dq_dm;
    // map<string, casadi::DM> C_res = C_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[2*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        arg[2*i] = &input_pos[i];
        arg[2*i+1] = &input_vel[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (C_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            C(j,i) = output_values[i * sz_res + j];
        }
    }

    return C;
}

JVec CS_Indy7::computeG(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> G_res = G_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (G_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        G(i) = output_values[i];
    }

    return G;
}

SE3 CS_Indy7::computeFK(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> FK_res = FK_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = 4;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FK_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            T_ee(j,i) = output_values[i * sz_res + j];
        }
    }

    R_ee = T_ee.block<3,3>(0,0);

    return T_ee;
}

Jacobian CS_Indy7::computeJ_b(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_b_res = J_b_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_b_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_b(j,i) = output_values[i * sz_res + j];
        }
    }

    return J_b;
}

Jacobian CS_Indy7::computeJdot_b(JVec _q, JVec _dq)
{
    T_M = computeFK(JVec::Zero());
	Slist = computeJ_s(JVec::Zero());
	Blist = Ad(TransInv(T_M)) * Slist;
    dJ_b = dJacobianBody(T_M, Blist, _q, _dq);

    return dJ_b;
}

Jacobian CS_Indy7::computeJ_s(JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_s_res = J_s_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_s(j,i) = output_values[i * sz_res + j];
        }
    }

    return J_s;
}

Jacobian CS_Indy7::computeJdot_s(JVec _q, JVec _dq)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> dJ_s_res = dJ_s_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[2*sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        arg[2*i] = &input_pos[i];
        arg[2*i+1] = &input_vel[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (dJ_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            dJ_s(j,i) = output_values[i * 6 + j];
        }
    }

    return dJ_s;
}

double CS_Indy7::computeManipulability(JVec _q)
{
    computeJ_s(_q);

    double tmp;

    manipulability = sqrt((J_b*J_b.transpose()).determinant());

    return manipulability;
}

MassMat CS_Indy7::getM()
{
    return M;
}
MassMat CS_Indy7::getMinv()
{
    return Minv;
}
MassMat CS_Indy7::getC()
{
    return C;
}
JVec CS_Indy7::getG()
{
    return G;
}
SE3 CS_Indy7::getFK()
{
    return T_ee;
}
SO3 CS_Indy7::getRMat()
{
    return R_ee;
}
Jacobian CS_Indy7::getJ_b()
{
    return J_b;
}
Jacobian CS_Indy7::getJ_s()
{
    return J_s;
}
Jacobian CS_Indy7::getJdot_b()
{
    return dJ_b;
}
Jacobian CS_Indy7::getJdot_s()
{
    return dJ_s;
}
Twist CS_Indy7::getBodyTwist()
{
    return V_b;
}
double CS_Indy7::getManipulability()
{
    return manipulability;
}

JVec CS_Indy7::FrictionEstimation(JVec _dq)
{
    JVec tau_fric;
    
    for (int i = 0; i<NRMK_DRIVE_NUM; i++)
	{
		if(_dq(i)>0.0)
			tau_fric(i) = Fc(i)+Fv1(i)*(1-exp(-fabs(_dq(i)/Fv2(i))));
		else if(_dq(i)<0.0)
			tau_fric(i) = -(Fc(i)+Fv1(i)*(1-exp(-fabs(_dq(i)/Fv2(i)))));
		else
			tau_fric(i) = 0.0;
	}

    return tau_fric;
}

JVec CS_Indy7::ComputedTorqueControl( JVec _q,JVec _dq,JVec q_des,JVec dq_des,JVec ddq_des)
{
    JVec e = q_des-_q;
    JVec edot = dq_des-_dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref + C*_dq + G;
        isUpdated = false;
    }
    else
    {
        M = computeM(_q);
        C = computeC(_q, _dq);
        G = computeG(_q);
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref+C*_dq+G;
    }
    return tau;   
}

JVec CS_Indy7::ComputedTorqueControl( JVec _q,JVec _dq,JVec q_des,JVec dq_des,JVec ddq_des, JVec _tau_ext)
{
    JVec e = q_des-_q;
    JVec edot = dq_des-_dq;

    // apperent inertia, spring, damper
    MassMat m_ = MassMat::Zero();
    JVec k_, d_;

    m_.diagonal()<< 5, 2.5, 1.25, 0.25, 0.25, 0.25;
    k_ << 500.0, 500.0, 300.0, 200.0, 200.0, 200.0;
    d_ << 50.0, 50.0, 30.0, 20.0, 20.0, 20.0;
    
    
    tau_ext = _tau_ext;
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des + m_.inverse()*(d_.cwiseProduct(edot) + k_.cwiseProduct(e) + tau_ext);
        tau = M*ddq_ref + C*_dq + G + tau_ext;
        isUpdated = false;
    }
    else
    {
        M = computeM(_q);
        C = computeC(_q, _dq);
        G = computeG(_q);
        JVec ddq_ref = ddq_des + m_.inverse()*(d_.cwiseProduct(edot) + k_.cwiseProduct(e) + tau_ext);
        tau = M*ddq_ref+C*_dq+G + tau_ext;
    }
    // tau = K.cwiseProduct(tau);
    return tau;    
}

JVec CS_Indy7::PassivityInverseDynamicControl( JVec _q, JVec _dq, JVec q_des, JVec dq_des, JVec ddq_des)
{
    JVec e = q_des-_q;
    JVec edot = dq_des-_dq;
    eint = eint + e*period;	
        
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        JVec dq_ref = dq_des + Kv*e + Kp*eint;
        JVec tau_ref = K*(dq_ref-_dq);

        tau = M*ddq_ref + C*dq_ref + G + tau_ref;
        isUpdated = false;
    }
    else
    {
        M = computeM(_q);
        C = computeC(_q, _dq);
        G = computeG(_q);
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        JVec dq_ref = dq_des + Kv*e + Kp*eint;
        JVec tau_ref = K*(dq_ref-_dq);

        tau = M*ddq_ref + C*dq_ref + G + tau_ref;
    }
    return tau;   
}

JVec CS_Indy7::PassivityInverseDynamicControl( JVec _q, JVec _dq, JVec q_des, JVec dq_des, JVec ddq_des, JVec _tau_ext)
{
    JVec e = q_des-_q;
    JVec edot = dq_des-_dq;
    eint = eint + e*period;	
        
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        JVec dq_ref = dq_des + Kv*e + Kp*eint;
        JVec tau_ref = K*(dq_ref-_dq);

        tau = M*ddq_ref + C*dq_ref + G + tau_ref;
        isUpdated = false;
    }
    else
    {
        M = computeM(_q);
        C = computeC(_q, _dq);
        G = computeG(_q);
        JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        JVec dq_ref = dq_des + Kv*e + Kp*eint;
        JVec tau_ref = K*(dq_ref-_dq);

        tau = M*ddq_ref + C*dq_ref + G + tau_ref;
    }
    return tau;   
}

JVec CS_Indy7::TaskInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot);
    JVec torques = M * qddot_ref + C * q_dot + G;

    return torques;
}

JVec CS_Indy7::TaskJTbasedInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    JMat D_scaled = JMat::Zero();
    
    for(int i=0; i<NRMK_DRIVE_NUM; i++)
    {
        JVec J_i = J_b.block<NRMK_DRIVE_NUM,1>(0,i);
        D_scaled(i,i) = 1.0/(J_i.transpose()*J_i);
        // printf("%lf, ", D_scaled(i,i));
    }
    // printf("\n");
    
    JVec qddot_ref = D_scaled * J_b.transpose() * (V_dot_ref - dJ_b * q_dot);
    JVec torques = M * qddot_ref + C * q_dot + G;

    return torques;
}

JVec CS_Indy7::TaskPassivityInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_dot_ref = Task_Kv * lambda + Task_Kp * lambda_int;
    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_ref = Ad(T_err) * (V_des + dexp6(-lambda) * (lambda_dot_ref));
    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot_ref));
    
    pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    JVec q_dot_ref = invJb * V_ref;
    JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot_ref);
    JVec edot = q_dot_ref - q_dot;
    JVec tau_ref = Task_K * edot;
    
    JVec torques = M * qddot_ref + C * q_dot_ref + G + tau_ref;

    return torques;
}

JVec CS_Indy7::TaskImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot);
    
    JVec torques = M * qddot_ref + C * q_dot + G + J_b.transpose()*F_ext;

    lambda_dot += lambda_ddot_ref*period;
    lambda += lambda_dot*period;

    // printf("lambda: %lf, %lf, %lf, %lf, %lf, %lf \n", lambda(0), lambda(1), lambda(2), lambda(3), lambda(4), lambda(5));
    // printf("lambda_dot: %lf, %lf, %lf, %lf, %lf, %lf \n", lambda_dot(0), lambda_dot(1), lambda_dot(2), lambda_dot(3), lambda_dot(4), lambda_dot(5));
    // printf("lambda_ddot_ref: %lf, %lf, %lf, %lf, %lf, %lf \n\n", lambda_ddot_ref(0), lambda_ddot_ref(1), lambda_ddot_ref(2), lambda_ddot_ref(3), lambda_ddot_ref(4), lambda_ddot_ref(5));
    

    return torques;
}

JVec CS_Indy7::TaskJTbasedImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    JMat D_scaled = JMat::Zero();
    
    for(int i=0; i<NRMK_DRIVE_NUM; i++)
    {
        JVec J_i = J_b.block<NRMK_DRIVE_NUM,1>(0,i);
        D_scaled(i,i) = 1.0/(J_i.transpose()*J_i);
    }
    
    JVec qddot_ref = D_scaled*J_b.transpose() * (V_dot_ref - dJ_b * q_dot);
    
    JVec torques = M * qddot_ref + C * q_dot + G + J_b.transpose()*F_ext;

    return torques;
}

JVec CS_Indy7::TaskPassivityImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    A_.diagonal() << 3, 3, 3, 0.5, 0.5, 0.5;
    D_.diagonal() << 10, 10, 10, 1, 1, 1;
    K_.diagonal() << 1000, 1000, 1000, 100, 100, 100;

    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_dot_ref = -Task_Kv_imp * lambda - Task_Kp_imp * lambda_int + Task_Kgama_imp * gamma_int;
    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_ref = Ad(T_err) * (V_des + dexp6(-lambda) * (lambda_dot_ref));
    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot_ref));
    
    pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    JVec q_dot_ref = invJb * V_ref;
    JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot_ref);

    Twist lambda_dot_tilde_ref = lambda_dot_ref - lambda_dot;
    Twist V_tilde_ref = -Ad(T_err)*dexp6(-lambda)*lambda_dot_tilde_ref;
    JVec tau_ref = J_b.transpose()*(Task_K_imp*V_tilde_ref - F_ext);
    
    JVec torques = M * qddot_ref + C * q_dot_ref + G + tau_ref;

    return torques;
}

JVec CS_Indy7::TaskComplianceImpedanceControl(SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    Twist V_dot_err = V_dot_des - Ad(invT_err)*V_dot + ad(V_err)*Ad(invT_err)*V_b;

    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;
    Twist lambda_ddot = dlog6(-lambda)*V_dot_err + ddlog6(-lambda, -lambda_dot)*V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    F_eff_int += F_eff * period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    // JVec torques = J_b.transpose()*(dlog6(-lambda).transpose()*(A_lambda*lambda_ddot+ K_lambda*lambda + D_lambda*lambda_dot))+G;// + (F_des + 1.0 * F_eff + 0.1 * F_eff_int)) + G ;
    JVec torques = J_b.transpose()*(D_*lambda_dot + K_*lambda)+G;

    return torques;
}

JVec CS_Indy7::TaskStablePD(SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    Matrix6d AdInvT = Ad(invT_err);

    Twist V_err = V_des - AdInvT * V_b;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    pinvJacobian JT = J_b.transpose();
    Matrix6d Kv_bar = dlog6(-lambda).transpose()*Task_Kv*period*dlog6(-lambda);

    double a0=1/2.50;
    double b0=75.0;
    double b = b0*exp(-pow(J_b.determinant(),2)/pow(a0,2));
    Twist Ke = Task_Kp*(lambda+lambda_dot*period);
    double fcut = 10.0;
    JVec qddot;

    // if (Ke.norm()<fcut)
    // {
        qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*dlog6(-lambda).transpose()*(Task_Kp*(lambda+lambda_dot*period)+Task_Kv*lambda_dot
                    +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq);
    // }
    // else
    // {
    //     JVec err_tmp = lambda+lambda_dot*period;
    //     qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*(fcut*err_tmp/err_tmp.norm()+Task_Kv*lambda_dot
    //                 +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq);
    // }

    return qddot;
}

JVec CS_Indy7::TaskStablePDImpedance(SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    Matrix6d AdInvT = Ad(invT_err);

    Twist V_err = V_des - AdInvT * V_b;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    pinvJacobian JT = J_b.transpose();

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;
    Twist V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    Twist V_ref = V_b + V_dot_ref*period;
    lambda_dot = dlog6(-lambda)*V_ref*period;
    lambda += lambda_dot*period;
    T_ref = T_ee*MatrixExp6(VecTose3(lambda));

    // printf("lambda: %lf, %lf, %lf, %lf, %lf, %lf \n", lambda(0), lambda(1), lambda(2), lambda(3), lambda(4), lambda(5));
    // printf("lambda_dot: %lf, %lf, %lf, %lf, %lf, %lf \n", lambda_dot(0), lambda_dot(1), lambda_dot(2), lambda_dot(3), lambda_dot(4), lambda_dot(5));
    // printf("lambda_ddot_ref: %lf, %lf, %lf, %lf, %lf, %lf \n\n", lambda_ddot_ref(0), lambda_ddot_ref(1), lambda_ddot_ref(2), lambda_ddot_ref(3), lambda_ddot_ref(4), lambda_ddot_ref(5));
    
    T_err = TransInv(T_ee)*T_ref;
    invT_err = TransInv(T_err);
    AdInvT = Ad(invT_err);

    V_err = V_ref - AdInvT * V_b;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    Matrix6d Kv_bar = Task_Kv*period*dlog6(-lambda);
    double a0=1/25.0;
    double b0=50.0;
    double b = b0*exp(-pow(J_b.determinant(),2)/pow(a0,2));
    Twist Ke = Task_Kp*(lambda+lambda_dot*period);
    double fcut = 10.0;
    JVec qddot;

    if (Ke.norm()<fcut)
    {
        qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*(Task_Kp*(lambda+lambda_dot*period)+Task_Kv*lambda_dot
                    +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq);
    }
    else
    {
        JVec err_tmp = lambda+lambda_dot*period;
        qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*(fcut*err_tmp/err_tmp.norm()+Task_Kv*lambda_dot
                    +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq);
    }    

    return qddot;
}

void CS_Indy7::TaskAdmittance(SE3 T_des, Twist V_des, Twist V_dot_des, SE3 &T_adm, Twist &V_adm, Twist &V_dot_adm, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ref)*T_des;
    SE3 invT_err = TransInv(T_err);
    Matrix6d AdInvT = Ad(invT_err);

    Twist V_err = V_des - AdInvT * V_ref;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();

    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;
    
    printf("lambda: %lf, %lf, %lf %lf, %lf, %lf\n", lambda(0), lambda(1), lambda(2), lambda(3), lambda(4), lambda(5));
    printf("lam_dot: %lf, %lf, %lf %lf, %lf, %lf\n", lambda_dot(0), lambda_dot(1), lambda_dot(2), lambda_dot(3), lambda_dot(4), lambda_dot(5));
    printf("lam_ddot_ref: %lf, %lf, %lf %lf, %lf, %lf\n", lambda_ddot_ref(0), lambda_ddot_ref(1), lambda_ddot_ref(2), lambda_ddot_ref(3), lambda_ddot_ref(4), lambda_ddot_ref(5));
    V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    V_ref += V_dot_ref*period;
    lambda_dot = dlog6(-lambda)*V_ref*period;
    T_ref = T_ref*MatrixExp6(VecTose3(lambda_dot));
    printf("lam_dot_ref: %lf, %lf, %lf %lf, %lf, %lf\n", lambda_dot(0), lambda_dot(1), lambda_dot(2), lambda_dot(3), lambda_dot(4), lambda_dot(5));
    V_dot_adm =  V_dot_ref;
    V_adm = V_ref;
    T_adm = T_ref;
}

void CS_Indy7::resetTaskAdmittance()
{
    V_dot_ref = Twist::Zero();
    V_ref = V_b;
    T_ref = T_ee;
}

void CS_Indy7::saturationMaxTorque(JVec &torque, JVec MAX_TORQUES)
{
    for(int i =0;i<NRMK_DRIVE_NUM;i++){
        if(abs(torque(i))> MAX_TORQUES(i)){
            if(torque(i)>0) torque(i) = MAX_TORQUES(i);
            else torque(i) = -MAX_TORQUES(i);
        }
    }
}

JVec CS_Indy7::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des)
{
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

JVec CS_Indy7::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des, JVec _tau_ext)
{
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

JVec CS_Indy7::NRIC(JVec q_r, JVec dq_r, JVec q_n, JVec dq_n)
{
    JVec e = q_r-q_n;
    JVec edot = dq_r - dq_n;
    eint = eint + e*period;	
    
    tau = NRIC_K_gamma * (edot + NRIC_Kp*e + NRIC_Ki*eint);

    return tau;
}

double CS_Indy7::computeAlpha(JVec edot, JVec tau_c, JVec tau_ext)
{
    double edotc, edotx, alpha;
    edotc = edot.transpose() * tau_c;
    edotx = edot.transpose() * tau_ext;
    if(edotc>0 && edotx >0)
    {
        if(edotc<edotx)
            alpha=1.0;
        else
            alpha = edotx/edotc;
    }
    else
        alpha = 0;

    return alpha;
}