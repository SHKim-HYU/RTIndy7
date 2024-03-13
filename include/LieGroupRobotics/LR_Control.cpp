#include "LR_Control.h"

LR_Control::LR_Control() {
    // Constructor implementation

	this->Slist.resize(6,JOINTNUM);
	this->Blist.resize(6,JOINTNUM);	
	this->Glist;	
	this->Mlist;			
	this->M.resize(4,4);	    
    this->q.resize(JOINTNUM);	
    this->q_des.resize(JOINTNUM);	
    this->dq_des.resize(JOINTNUM);	
    this->ddq_des.resize(JOINTNUM);	
    this->dq.resize(JOINTNUM);	
    this->g.resize(3);
    this->torq.resize(JOINTNUM);

    this->g<<0,0,-9.8;
    this->Kp = MatrixNd::Zero();
    this->Kv = MatrixNd::Zero();
    this->Ki = MatrixNd::Zero();
    this->Hinf_Kp = MatrixNd::Zero();
    this->Hinf_Kv = MatrixNd::Zero();
    this->Hinf_K_gamma = MatrixNd::Zero();
    this->Task_Kp = Matrix6d::Identity()*200;
    this->Task_Kv = Matrix6d::Identity()*40;
    this->Task_K =  MatrixNd::Identity()*0.2;
    JVec invL2sqr=JVec::Zero();
    invL2sqr<<1000.0,1000.0,800.0,600.0,600.0,600.0;
    JVec K=Vector6d::Zero();
    K<<50.0,50.0,50.0,50.0,50.0,50.0;
    for (int i=0; i<JOINTNUM; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;
            break;

        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        }
    }
   for (int i=0; i<JOINTNUM; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 70.0;
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
        }
    }
}

JVec LR_Control::TaskHinfControl( JVec q,JVec q_dot,JVec q_ddot, SE3 T_des,Vector6d V_des,Vector6d V_dot_des,Vector6d& ret_lambda,Vector6d& lambda_int,double dt){
    
   SE3 T = lr::FKinBody(this->M,this->Blist,q);
    Jacobian Jb = lr::JacobianBody(this->Blist,q);
    Jacobian Jb_dot = lr::dJacobianBody(this->M,this->Blist,q,q_dot);
    SE3 T_err = lr::TransInv(T)*T_des;
    SE3 invT_err = TransInv(T_err);

    Vector6d V = Jb*q_dot;
    Vector6d V_err = V_des-Ad(invT_err)*V;
    Vector6d lambda = lr::se3ToVec(lr::MatrixLog6(T_err));
    ret_lambda = lambda;
    lambda_int += lambda*dt;
    Vector6d lambda_dot = dlog6(-lambda)*V_err;
    Vector6d V_dot = Jb_dot*q_dot + Jb*q_ddot;
    Vector6d V_dot_err = V_dot_des-Ad(invT_err)*V_dot+ad(V_err)*V_dot;
    Vector6d lambda_dot_ref =Task_Kv*lambda+Task_Kp*lambda_int; 
    Vector6d lambda_ddot_ref=Task_Kv*lambda_dot +Task_Kp*lambda;

    Vector6d V_ref = Ad(T_err)*(V_des + dexp6(-lambda)*(lambda_dot_ref));
    Vector6d V_dot_ref = Ad(T_err)*(V_dot_des + (dexp6(-lambda)*lambda_ddot_ref) + ad(V_err)*V_dot - (ddexp6(-lambda,-lambda_dot)*lambda_dot));
    pinvJacobian invJb = Jb.transpose()*(Jb*Jb.transpose()).inverse();
    //Jacobian invJb = (Jb.transpose()*Jb+Matrix6d::Identity()*0.001).inverse()*Jb.transpose();
    JVec qddot_ref = invJb*(V_dot_ref-Jb_dot*q_dot);
    JVec q_dot_ref = invJb*V_ref;
    JVec edot = q_dot_ref - q_dot;

    JVec tau_ref =this->Task_K*edot;
    MassMat M =this->MassMatrix(q);
    MatrixNd C = this->CoriolisMatrix(q,dq);
    JVec G = this->GravityForces(q);    

    JVec torques = M*qddot_ref +C*q_dot_ref + G+tau_ref;
    //JVec torques =JVec::Zero(); 
    return torques;
}
MassMat LR_Control::MassMatrix(JVec q_){
    MassMat M = MassMat::Zero();
    pinocchio::Data data(model);    

    
    // Create data required by the algorithms
    //pinocchio::Data data_(model_);
    
    //Eigen::VectorXd q = q_;
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = pinocchio::randomConfiguration(model);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  //  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
//    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    
    // Allocate result container
    Eigen::MatrixXd djoint_torque_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
    Eigen::MatrixXd djoint_torque_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
    Eigen::MatrixXd djoint_torque_da = Eigen::MatrixXd::Zero(model.nv,model.nv);
    
    Eigen::VectorXd tau = pinocchio::crba(model, data, q_);
    data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();

   // pinocchio::computeRNEADerivatives(model, data, q_, v, a, djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
    
    M = data.M;
    return M;
}
MassMat LR_Control::MassMatrixInverse(JVec q_){
    MassMat Minv = MassMat::Zero();
    pinocchio::Data data(model);    

    
    // Create data required by the algorithms
    //pinocchio::Data data_(model_);
    
    //Eigen::VectorXd q = q_;
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = pinocchio::randomConfiguration(model);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  //  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
//    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    
    // Allocate result container
    Eigen::MatrixXd djoint_torque_dq = Eigen::MatrixXd::Zero(model.nv,model.nv);
    Eigen::MatrixXd djoint_torque_dv = Eigen::MatrixXd::Zero(model.nv,model.nv);
    Eigen::MatrixXd djoint_torque_da = Eigen::MatrixXd::Zero(model.nv,model.nv);
    
    // Eigen::VectorXd tau = pinocchio::crba(model, data, q_);
    // data.M.triangularView<Eigen::StrictlyLower>() =
    //   data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::computeMinverse(model, data, q_);
    data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

   // pinocchio::computeRNEADerivatives(model, data, q_, v, a, djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
    
    Minv = data.Minv;
    return Minv;
}
JVec LR_Control::GravityForces(JVec q_){
    JVec G = JVec::Zero();
    
    pinocchio::Data data(model);    
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    q =q_;
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q,v*0,a*0);
    pinocchio::computeGeneralizedGravity(model, data, q);    
    G = data.g; 
    return G;
}

MatrixNd LR_Control::CoriolisMatrix(JVec q_,JVec q_dot){
    MatrixNd C = MatrixNd::Zero();
    pinocchio::Data data(model);    
    
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    q =q_;
    v =q_dot;
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q,v,a);
    pinocchio::computeCoriolisMatrix(model, data, q, v);    
    C = data.C; 
    return C;
}
JVec LR_Control::ForwardDynamics( JVec q,JVec dq ,JVec tau){
    pinocchio::Data data(model);  
    JVec ddq=JVec::Zero();
    pinocchio::integrate(model, q, dq, tau);
    pinocchio::aba(model,data, q, dq, tau);
    ddq = data.ddq;
    return ddq;
}
JVec LR_Control::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint){
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist);
    JVec C = lr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist);
    JVec G = lr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist) ; 
    JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
    JVec torq = Mmat*ddq_ref+C+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    return torq;
}

JVec LR_Control::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint,double eef_mass){
    JVec e = q_des-q;
    JVec edot = dq_des-dq;
    MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist,eef_mass);
    JVec C = lr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist,eef_mass);
    JVec G = lr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist,eef_mass) ; 
    JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
    JVec torq = Mmat*ddq_ref+C+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    return torq;
}


void LR_Control::WayPointTaskTrajectory(std::vector<SE3> way_points, std::vector<double> delays, double now, SE3& T_des,Vector6d& V_des,Vector6d& V_dot_des){
    std::vector<double> time_list;
    time_list.push_back(0);
    double prev_time=0; 
    int idx = 0;
    int N = way_points.size();
    int N_delay = delays.size();
    time_list.resize(N_delay+1);
    double sum_delays = 0;
    for(int i =0;i<N_delay;i++){
        sum_delays+=delays.at(i);

        time_list.at(i+1)=sum_delays;
        if(now>=time_list.at(i) && now < time_list.at(i+1)){
           idx = i;
        }
    }
    if(now>=sum_delays){
        idx = N_delay-1;
    }
    
    double Tf = time_list.at(idx+1)-time_list.at(idx);
    double now_ =now-time_list.at(idx);
    lr::LieScrewTrajectory( way_points.at(idx),   way_points.at(idx+1),Vector6d::Zero(),Vector6d::Zero(),Vector6d::Zero(),Vector6d::Zero(), Tf, now_  ,T_des,V_des,V_dot_des);
}

void LR_Control::WayPointJointTrajectory(std::vector<JVec> way_points, std::vector<double> delays, double now, JVec& q_des,JVec& q_dot_des,JVec& q_ddot_des){
    std::vector<double> time_list;
    time_list.push_back(0);
    double prev_time=0; 
    int idx = 0;
    int N = way_points.size();
    int N_delay = delays.size();
    time_list.resize(N_delay+1);
    double sum_delays = 0;
    for(int i =0;i<N_delay;i++){
        sum_delays+=delays.at(i);

        time_list.at(i+1)=sum_delays;
        if(now>=time_list.at(i) && now < time_list.at(i+1)){
           idx = i;
        }
    }
    if(now>=time_list.at(time_list.size()-1)){
        idx = N_delay-1;
    }
    lr::JointTrajectory(  way_points.at(idx),   way_points.at(idx+1),  time_list.at(idx+1)-time_list.at(idx), now-time_list.at(idx)  , 0 , q_des, q_dot_des, q_ddot_des);
}

struct LR_info{
	SE3 M;
	Eigen::MatrixXd Slist;
	std::vector<SE3> Mlist;
	std::vector<Matrix6d> Glist;
};

void print_LR_info(LR_info lr_info){
	std::cout<<"=======================================M======================================="<<std::endl;
	std::cout<<lr_info.M<<std::endl;
	std::cout<<"=======================================Slist==================================="<<std::endl;
	std::cout<<lr_info.Slist<<std::endl;
	std::cout<<"=======================================Mlist==================================="<<std::endl;

	for(int i = 0;i<lr_info.Mlist.size();i++){
		SE3 M = lr_info.Mlist.at(i);
		std::cout<<"---------------------------------------M"<<i<<"--------------------------------------"<<std::endl;
		std::cout<<M<<std::endl;
	}
	std::cout<<"=======================================Glist==================================="<<std::endl;
	for(int i = 0;i<lr_info.Glist.size();i++){
		Matrix6d G = lr_info.Glist.at(i);
		std::cout<<"---------------------------------------G"<<i<<"--------------------------------------"<<std::endl;
		std::cout<<G<<std::endl;
	}

}
SE3 createTransformationMatrix(double roll, double pitch, double yaw, 
                                           double x, double y, double z) {
    SE3 transform = SE3::Identity();

    // 회전 부분 (Roll, Pitch, Yaw를 사용하여 회전 행렬 생성) ZYX-euler
    SO3 rotation;
    rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // 위치 부분
    Vector3d translation(x, y, z);

    // 변환 행렬에 회전과 위치 적용
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;

    return transform;
}

Eigen::MatrixXd list_Slist_to_matrix_Slist(std::vector<Vector6d> Slist){
	int N = Slist.size();
	Eigen::MatrixXd ret_Slist(6,N);
	for(int i=0;i<N;i++){
		ret_Slist.col(i) = Slist.at(i);
	}
	return ret_Slist;

}


void LR_Control::LRSetup(const char* urdf_path){
    pinocchio::urdf::buildModel(urdf_path,model);
    model.gravity.linear( this->g);

    urdf::ModelInterfaceSharedPtr robot = urdf::parseURDFFile(urdf_path);
	if (!robot){
		std::cerr << "ERROR: Model Parsing the URDF failed" << std::endl;
	}
	std::cout << "robot name is: " << robot->getName() << std::endl;
	// get info from parser
	std::cout << "---------- Successfully Parsed URDF ---------------" << std::endl;
	// get root link
	urdf::LinkConstSharedPtr root_link=robot->getRoot();
	std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;
	// print entire tree
	//printTree(root_link);
	
    std::stack<std::pair<urdf::LinkConstSharedPtr, int>> stack;
    stack.push(std::make_pair(root_link, 0));
	std::vector<Matrix6d> Glist;
	std::vector<SE3> Mlist;
	std::vector<Vector6d> Slist;

	SE3 T = SE3::Identity();
	SE3 prev_T_com = SE3::Identity();
    int count = 0;
    while (!stack.empty()) {
        auto current = stack.top();
        urdf::LinkConstSharedPtr link = current.first;
        int level = current.second;
        stack.pop();

        // 인덴트와 링크 이름 출력
        //std::cout << "Link: " << link->name << std::endl;

        // 해당 링크에 연결된 조인트 정보 출력
        if (link->parent_joint) {
			Matrix6d G = Matrix6d::Identity();
			const auto& inertial = link->inertial;
			const auto& joint = link->parent_joint;
			double mass= inertial->mass;
			G(0,0) = mass;
			G(1,1) = mass;
			G(2,2) = mass;
			G(3,3) = inertial->ixx;
			G(4,4) = inertial->iyy;
			G(5,5) = inertial->izz;
			G(3,4) = G(4,3)=inertial->ixy;
			G(3,5) = G(5,3)=inertial->ixz;

            const auto& position = joint->parent_to_joint_origin_transform.position;
			double jnt_x,jnt_y,jnt_z,jnt_roll,jnt_pitch,jnt_yaw,com_x,com_y,com_z,com_roll,com_pitch,com_yaw;
			joint->parent_to_joint_origin_transform.rotation.getRPY(jnt_roll,jnt_pitch,jnt_yaw);
			jnt_x = joint->parent_to_joint_origin_transform.position.x;
			jnt_y = joint->parent_to_joint_origin_transform.position.y;
			jnt_z = joint->parent_to_joint_origin_transform.position.z;

			inertial->origin.rotation.getRPY(com_roll,com_pitch,com_yaw);
			com_x = inertial->origin.position.x;
			com_y = inertial->origin.position.y;
			com_z = inertial->origin.position.z;

			SE3 Tcom = createTransformationMatrix(com_roll,com_pitch,com_yaw,com_x,com_y,com_z);
			SE3 Tcom_to_jnt = createTransformationMatrix(jnt_roll,jnt_pitch,jnt_yaw,jnt_x,jnt_y,jnt_z);
			SE3 M = lr::TransInv(prev_T_com)*T*Tcom;
			prev_T_com = T*Tcom;
			T = T*Tcom*Tcom_to_jnt;
    		Matrix3d R = T.block<3,3>(0,0);		
			Vector3d w = R*Vector3d(joint->axis.x,joint->axis.y,joint->axis.z);
			Vector3d p = T.block<3,1>(0,3);
			Vector3d v = -w.cross(p);

			if(link->parent_joint->type==urdf::Joint::REVOLUTE ||link->parent_joint->type==urdf::Joint::PRISMATIC ){
                std::cout<<"count : "<<count<<std::endl;

				Glist.push_back(G);
				Vector6d S = Vector6d::Zero();
				S.block<3,1>(0,0) = v;
				S.block<3,1>(3,0) = w;
				Slist.push_back(S);
                if(count++>0){
                    Mlist.push_back(M);
                }
				
			}

			
        }

        // 자식 링크들을 스택에 추가
        for (auto& child_link : link->child_links) {
            if (child_link) {
                stack.push(std::make_pair(child_link, level + 2));
            }
        }
    }
	Eigen::MatrixXd Slist_ = list_Slist_to_matrix_Slist(Slist);
	SE3 M = T;
	LR_info lr_info ;
	lr_info.M = M;
    Mlist.push_back(lr::TransInv(prev_T_com)*M);
	lr_info.Slist = Slist_;
	lr_info.Mlist = Mlist;
	lr_info.Glist = Glist;
    this->Slist = lr_info.Slist;
    this->M = lr_info.M;
    this->Mlist = lr_info.Mlist;
    this->Glist = lr_info.Glist;
    this->Blist = Ad(TransInv(M))*lr_info.Slist;
	print_LR_info(lr_info);    

}



