#include "MR_Indy7.h"
#include "../KDL/PropertyDefinition.h"

bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
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
    cout<<"END ReadMRData"<<endl;

    return 1;
}

MR_Indy7::MR_Indy7() {
    // Constructor implementation

	this->Slist.resize(6,6);
	this->Blist.resize(6,6);	
	this->Glist;	
	this->Mlist;			
	this->M.resize(4,4);	    
    this->q.resize(6);	
    this->q_des.resize(6);	
    this->dq_des.resize(6);	
    this->ddq_des.resize(6);	
    this->dq.resize(6);	
    this->g.resize(3);
    this->torq.resize(6);

    this->g<<0,0,-9.8;
    this->Kp = mr::Matrix6d::Zero();
    this->Kv = mr::Matrix6d::Zero();
    
    for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            break;
        case 1:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            break;
        case 5:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            break;
        }
    }
        

}

void MR_Indy7::Gravity( double *q, double *toq){
    this->q(0) = q[0];
    this->q(1) = q[1];
    this->q(2) = q[2];
    this->q(3) = q[3];
    this->q(4) = q[4];
    this->q(5) = q[5];
    this->torq= mr::GravityForces(this->q,this->g,this->Mlist, this->Glist, this->Slist) ;
   for(int i=0; i<6; ++i) {
        if(i==0)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_1*GEAR_RATIO_121*EFFICIENCY)*100.0;
		else if(i==1)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_2*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==2)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_200)/(double)(TORQUE_CONST_3*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==3)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_4*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==4)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_5*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==5)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_6*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else
            return;
    }    
}

void MR_Indy7::ComputedTorqueControl( double *q,double *dq,double *qdes,double *dqdes,  double *toq){
    
    this->q = Map<VectorXd>(q,6);
    this->dq = Map<VectorXd>(dq,6);
    this->q_des = Map<VectorXd>(qdes,6);
    this->dq_des = Map<VectorXd>(dqdes,6);

    //VectorXd e = this->q_des-this->q;
    //VectorXd edot = this->dq_des-this->dq;
    
    //MatrixXd Mmat = mr::MassMatrix(this->q,this->Mlist, this->Glist, this->Slist);
    //VectorXd C = mr::VelQuadraticForces(this->q, this->dq,this->Mlist, this->Glist, this->Slist);
    VectorXd gravTorq= mr::GravityForces(this->q,this->g,this->Mlist, this->Glist, this->Slist) ;
    //VectorXd ddq_ref = Kv*edot+Kp*e;
    //torq = Mmat*ddq_ref+C*this->dq + gravTorq;
    torq = gravTorq;
   for(int i=0; i<6; ++i) {
        if(i==0)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_1*GEAR_RATIO_121*EFFICIENCY)*100.0;
		else if(i==1)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_500)/(double)(TORQUE_CONST_2*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==2)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_200)/(double)(TORQUE_CONST_3*GEAR_RATIO_121*EFFICIENCY)*100.0;
        else if(i==3)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_4*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==4)
            toq[i] = -(torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_5*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else if(i==5)
            toq[i] = (torq(i))*(double)(TORQUE_ADC_100)/(double)(TORQUE_CONST_6*GEAR_RATIO_101*EFFICIENCY)*100.0;
        else
            return;
    }    
}
void MR_Indy7::MRSetup(){
	//cout<<"START MRSetup"<<endl;
	Json::Value rootr;
	bool ret = ReadMRData("MR_info.json",rootr);

	//cout<<"MR Setup 2"<<endl;
	for(int i =0;i<6 ; i++){
        std::cout<<"asdfafsd"<<std::endl;

		for(int j =0;j<6;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	
    cout<<"=================Slist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================Blist================="<<endl;
    cout<<this->Blist<<endl;
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		MatrixXd M = MatrixXd::Identity(4,4);
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
        cout<<"=================M"<<i<<"============================"<<endl;
        cout<<M<<endl;

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
        cout<<"=================G"<<i<<"============================"<<endl;
        cout<<G<<endl;

		char str[50];		
		this->Glist.push_back(G);	}	

	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
	cout<<"END MRSetup"<<endl;

}
