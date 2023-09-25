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

	load_casadi_function();

}

CS_Indy7::load_casadi_function()
{

	// Load the shared library
    fd_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_fd.so", RTLD_LAZY);
    if (fd_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fd.so");
    }
    G_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_G.so", RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open indy7_G.so");
    }
    M_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_M.so", RTLD_LAZY);
    if (M_handle == 0) {
        throw std::runtime_error("Cannot open indy7_M.so");
    }
    C_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_C.so", RTLD_LAZY);
    if (C_handle == 0) {
        throw std::runtime_error("Cannot open indy7_C.so");
    }

    J_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_J_b.so", RTLD_LAZY);
    if (J_handle == 0) {
        throw std::runtime_error("Cannot open indy7_J_b.so");
    }


    // Reset error
    dlerror();

    // Function evaluation
    fd_eval = (eval_t)dlsym(fd_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    
    G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    M_eval = (eval_t)dlsym(M_handle, "M");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    C_eval = (eval_t)dlsym(C_handle, "coriolis");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

    J_eval = (eval_t)dlsym(J_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
}