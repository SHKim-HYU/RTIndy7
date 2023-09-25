#include "CS_Indy7.h"



CS_Indy7::CS_Indy7(const string& _modelPath) : loader_(_modelPath)
{
	robotModel = loder_.getValue("name").asString();
	n_dof = loader_.getValue("n_dof").asInt();
	
	this->q.resize(this->n_dof);
	this->dq.resize(this->n_dof);
	this->ddq.resize(this->n_dof);
	this->M.resize(this->n_dof, this->n_dof);
	this->Minv.resize(this->n_dof, this->n_dof);
	this->C.resize(this->n_dof, this->n_dof);
	this->G.resize(this->n_dof);
	this->J_b.resize(6, this->n_dof);
	this->J_s.resize(6, this->n_dof);

	this->Kp.resize(this->n_dof, this->n_dof);
    this->Kv.resize(this->n_dof, this->n_dof);
    this->Ki.resize(this->n_dof, this->n_dof);

    this->Hinf_Kp.resize( this->n_dof, this->n_dof);
    this->Hinf_Kv.resize( this->n_dof, this->n_dof);
    this->Hinf_Ki.resize( this->n_dof, this->n_dof);
    this->Hinf_K_gamma.resize( this->n_dof, this->n_dof);

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
        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_6 ;

            break;
        }
    }
   for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 75.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            Ki(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        case 6:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        }
    }

	load_casadi_function(loader_);

}

CS_Indy7::load_casadi_function(JsonLoader& loader)
{
    string FD_path = loader_.getValue("forward_dynamics_path");
    string M_path = loader_.getValue("mass_matrix_path");
    string Minv_path = loader_.getValue("mass_inverse_matrix_path");
    string C_path = loader_.getValue("coriolis_path");
    string G_path = loader_.getValue("gravity_path");
    string FK_path = loader_.getValue("forward_kinematics_path");
    string Js_path = loader_.getValue("Jacobian_space_path");
    string Jb_path = loader_.getValue("Jacobian_body_path");

    fd_cs = casadi::Function::load(FD_path);
    M_cs = casadi::Function::load(M_path);
    Minv_cs = casadi::Function::load(Minv_path);
    C_cs = casadi::Function::load(C_path);
    G_cs = casadi::Function::load(G_path);
    J_s_cs = casadi::Function::load(FK_path);
    J_b_cs = casadi::Function::load(Js_path);
    FK_cs = casadi::Function::load(Jb_path);

	// // Load the shared library
    // fd_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_fd.so", RTLD_LAZY);
    // if (fd_handle == 0) {
    //     throw std::runtime_error("Cannot open indy7_fd.so");
    // }
    // G_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_G.so", RTLD_LAZY);
    // if (G_handle == 0) {
    //     throw std::runtime_error("Cannot open indy7_G.so");
    // }
    // M_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_M.so", RTLD_LAZY);
    // if (M_handle == 0) {
    //     throw std::runtime_error("Cannot open indy7_M.so");
    // }
    // C_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_C.so", RTLD_LAZY);
    // if (C_handle == 0) {
    //     throw std::runtime_error("Cannot open indy7_C.so");
    // }

    // J_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_J_b.so", RTLD_LAZY);
    // if (J_handle == 0) {
    //     throw std::runtime_error("Cannot open indy7_J_b.so");
    // }


    // // Reset error
    // dlerror();

    // // Function evaluation
    // fd_eval = (eval_t)dlsym(fd_handle, "aba");
    // if (dlerror()) {
    //     throw std::runtime_error("Function evaluation failed.");
    // }
    
    // G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    // if (dlerror()) {
    //     throw std::runtime_error("Function evaluation failed.");
    // }
    // M_eval = (eval_t)dlsym(M_handle, "M");
    // if (dlerror()) {
    //     throw std::runtime_error("Function evaluation failed.");
    // }
    // C_eval = (eval_t)dlsym(C_handle, "coriolis");
    // if (dlerror()) {
    //     throw std::runtime_error("Function evaluation failed.");
    // }

    // J_eval = (eval_t)dlsym(J_handle, "J_b");
    // if (dlerror()) {
    //     throw std::runtime_error("Function evaluation failed.");
    // }
}

CS_Indy7::updateRobot(JVec _q, JVec _dq)
{
    M = M(JVec _q);
    Minv = Minv(JVec _q);
    C = C(JVec _q, JVec _dq);
    G = G(JVec _q); 

    T_ee = FK(JVec _q);


    isUpdated=true;   
}

CS_Indy7::M(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> M_res = M_cs(arg);

    return M;
}

CS_Indy7::Minv(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> Minv_res = Minv_cs(arg);

    return M;
}

CS_Indy7::C(JVec _q, JVec _dq)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    casadi::DM dq_dm = casadi::DM(vector<double>(_dq.data(), _dq.data() + _dq.size()));
    map<string, casadi::DM> arg;
    arg["q"] = q_dm; arg["dq"] = dq_dm;
    vector<casadi::DM> C_res = C_cs(arg);

    return C;
}

CS_Indy7::G(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> G_res = G_cs(arg);

    return G;
}

CS_Indy7::FK(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> FK_res = FK_cs(arg);

    return T_ee;
}

CS_Indy7::J_b(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> J_b_res = J_b_cs(arg);

    return J_b;
}

CS_Indy7::J_s(JVec _q)
{
    casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    vector<casadi::DM> arg = {q_dm};
    vector<casadi::DM> J_s_res = J_s_cs(arg);

    return J_s;
}

CS_Indy7::ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des)
{
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    
    if(isUpdated)
    {
        JVec ddq_ref = Kv*edot+Kp*e;
        JVec torq = M*ddq_ref+C*dq+G;
        isUpdated = false;
    }
    else
    {
        M = M(JVec _q);
        C = C(JVec _q, JVec _dq);
        G = G(JVec _q);
        JVec ddq_ref = Kv*edot+Kp*e;
        JVec torq = M*ddq_ref+C*dq+G;
    }
    return torq;   
}

CS_Indy7::saturationMaxTorque(JVec &torque, JVec MAX_TORQUES)
{
    for(int i =0;i<JOINTNUM;i++){
        if(abs(torque(i))> MAX_TORQUES(i)){
            if(torque(i)>0) torque(i) = MAX_TORQUES(i);
            else torque(i) = -MAX_TORQUES(i);
        }
    }
}

CS_Indy7::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint)
{
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    if(isUpdated)
    {
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        JVec torq = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = M(JVec _q);
        C = C(JVec _q, JVec _dq);
        G = G(JVec _q);
        JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        JVec dq_ref = dq_des;
        JVec torq = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return torq;
}