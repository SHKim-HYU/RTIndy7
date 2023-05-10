#include "Indy7.h"
#include <ModernRobotics.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <json/json.h>
#pragma comment(lib, "jsoncpp.lib")

using namespace Eigen;
using namespace std;
using namespace mr;
Indy7::Indy7(const std::string mr_json_path){
	cout<<"Indy7 LOAD"<<endl;
	this->actuated_joint_num = 6;
	this->MRSetup(mr_json_path);
}
bool ReadFromFile(const std::string filename, char* buffer, int len){
  FILE* r = fopen(filename.c_str(),"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData(const std::string filename,Json::Value &rootr){
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	cout<<filename<<endl;
	if (false == ReadFromFile(filename, readBuffer, BufferLength)) {
		cout<<"Failed"<<endl;
		return -1;
	}
	std::string config_doc = readBuffer;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,rootr);
	if ( !parsingSuccessful ) { 
		std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
		return -1;
		
	}
	return 1;
}
void Indy7::MRSetup(const std::string mr_json_path){
	//cout<<"START MRSetup"<<endl;
	Json::Value rootr;
	bool ret = ReadMRData(mr_json_path,rootr);

//	cout<<"MR Setup 1"<<endl;
	//cout<<"MR Setup 2"<<endl;
	for(int i =0;i<6 ; i++){
		for(int j =0;j<this->actuated_joint_num;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	
	//cout<<"MR Setup 3"<<endl;	
	
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		SE3 M = SE3::Identity();
        MatrixXd M2 = MatrixXd::Identity(4,4);
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
                M2(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
		char str[50];
		//sprintf(str,"M%d%d",i,i+1);
		
		this->Mlist.push_back(M);
        this->Mlist2.push_back(M2);
		//printMatrix(M,str);
	}
	for(int i = 0;i< rootr["Glist"].size(); i++){
		Matrix6d G = Matrix6d::Identity();
        MatrixXd G2 = MatrixXd::Identity(6,6);
		for(int j = 0;j< rootr["Glist"][0].size(); j++){
			for(int k = 0;k< rootr["Glist"][0][0].size(); k++){
				G(j,k) = rootr["Glist"][i][j][k].asDouble();
                G2(j,k) = rootr["Glist"][i][j][k].asDouble();
			}
		}
		char str[50];
		//sprintf(str,"G%d",i);
		
		this->Glist.push_back(G);
        this->Glist2.push_back(G2);
		//printMatrix(G,str);
	}	

	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
    
	//cout<<"END MRSetup"<<endl;

}

Indy7::~Indy7(){
	
}