#include "LR_Control.h"

bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadLRData(const char* filename,Json::Value &rootr){
	cout<<"START ReadLRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(filename, readBuffer, BufferLength)) {
		std::cout<<"Failed"<<std::endl;
		return -1;
	}
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,rootr);
	if ( !parsingSuccessful ) { 
		std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
		return -1;
		
	}
    cout<<"END ReadLRData"<<endl;

    return 1;
}


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
    this->Task_Kp = Matrix6d::Identity()*2000;
    this->Task_Kv = Matrix6d::Identity()*200;
    JVec invL2sqr=JVec::Zero();
    invL2sqr<<80.0,160.0,160.0,160.0,80.0,80.0,8.0;
    JVec K=JVec::Zero();
    K<<80,300,10,200,10,50,1;
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

JVec LR_Control::TaskHinfControl( JVec q,JVec q_dot,JVec q_ddot, SE3 T_des,Vector6d V_des,Vector6d V_dot_des,Vector6d eint){
    
    SE3 T = lr::FKinBody(this->M,this->Blist,q);
    Jacobian Jb = lr::JacobianBody(this->Blist,q);
    Jacobian Jb_dot = lr::dJacobianBody(this->M,this->Blist,q,q_dot);
    Vector6d V = Jb*q_dot;
    SE3 T_err = TransInv(T)*T_des;
    SE3 invT_err = TransInv(T_err);
    Vector6d V_err = V_des-Ad(invT_err)*V;
    Vector6d V_dot = Jb_dot*q_dot + Jb*q_ddot;
    Vector6d V_dot_err = V_dot_des-Ad(invT_err)*V_dot+ad(V_err)*V_dot;
    Vector6d lambda = se3ToVec(MatrixLog6(T_err));
    Vector6d lambda_dot = dlog6(-lambda)*V_err;
    Vector6d lambda_ddot_ref =-Task_Kv*lambda_dot -Task_Kp*lambda;

    Vector6d V_dot_ref = Ad(T_err)*(V_dot_des - (dexp6(-lambda)*lambda_ddot_ref) + ad(V_err)*V_dot - (ddexp6(-lambda,-lambda_dot)*lambda_dot));
	JVec qddot_ref = Jb.transpose()*(Jb*Jb.transpose()+Matrix6d::Identity()*0.001).inverse()*(V_dot_ref-Jb_dot*q_dot);
    MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist);
    JVec C = lr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist);
    JVec G = lr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist) ; 
    JVec torq = Mmat*qddot_ref+C+G;

    return torq;
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
    if(now>=time_list.at(time_list.size()-1)){
        idx = N_delay-1;
    }
    
    
    lr::LieScrewTrajectory( way_points.at(idx),   way_points.at(idx+1),Vector6d::Zero(),Vector6d::Zero(),Vector6d::Zero(),Vector6d::Zero(), time_list.at(idx+1)-time_list.at(idx), now-time_list.at(idx)  ,T_des,V_des,V_dot_des);
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
void concat(const char* s1, const char* s2, char* dest, size_t destSize) {
    // destSize는 dest의 전체 크기를 나타냅니다.
    // sprintf 대신에 snprintf를 사용하여 버퍼 오버플로우 방지
    snprintf(dest, destSize, "%s%s", s1, s2);
}
void LR_Control::LRSetup(const char* urdf_path){
	Json::Value rootr;
    const char* LR_path ="LR_info.json";
    char* filepath = strdup(urdf_path);
    char LR_full_path[100]; // 
    if (filepath != NULL) {
        char* lastSlash = strrchr(filepath, '/');
        if (lastSlash) {
            *(lastSlash + 1) = '\0'; //
            concat(filepath, LR_path, LR_full_path, sizeof(LR_full_path));    
        } else {
            concat(filepath, LR_path, LR_full_path, sizeof(LR_full_path));    

        }
        free(filepath);
    } else {
        printf("ERROR.\n");
    }
	bool ret = ReadLRData(LR_full_path,rootr);
    if(ret == 1)
        printf("LR_info.json has been successfully loaded!\n");
    else
        printf("Failed to load LR_info.json!\n");


    ScrewList Blist,Slist;
	for(int i =0;i<6 ; i++){
		for(int j =0;j<JOINTNUM;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	

     cout<<"=====================Slist====================="<<endl;
     cout<<this->Slist<<endl;
    // cout<<"=====================Blist====================="<<endl;
    // cout<<this->Blist<<endl;
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		MatrixXd M = MatrixXd::Identity(4,4);
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
        // cout<<"=================M"<<i<<"============================"<<endl;
        // cout<<M<<endl;

		char str[50];		
		this->Mlist.push_back(M);
	}
	for(int i = 0;i< rootr["Glist"].size(); i++){
		MatrixXd G = MatrixXd::Identity(6,6);
		for(int j = 0;j< rootr["Glist"][0].size(); j++){
			for(int k = 0;k< rootr["Glist"][0][0].size(); k++){
				G(j,k) = rootr["Glist"][i][j][k].asDouble();
			}
		}
        // cout<<"=================G"<<i<<"============================"<<endl;
        // cout<<G<<endl;

       // G_.block<3,3>(0,0) = G.block<3,3>(3,3);
        //G_.block<3,3>(3,3) = G.block<3,3>(0,0);
		char str[50];		
		this->Glist.push_back(G);	}	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
    // cout<<"=================M================="<<endl;
    // cout<<this->M<<endl;    
	// cout<<"END MRSetup"<<endl;

}
