#include "MR_DualArm.h"

bool ReadFromFile_(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData_(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile_(filename, readBuffer, BufferLength)) {
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

MR_DualArm::MR_DualArm(){
    
}
void MR_DualArm::MRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData_("MR_info.json",rootr);
	for(int i =0;i<6 ; i++){
		for(int j =0;j<12;j++){
			this->Slist(i,j) = rootr["relSlist"][i][j].asDouble();
			this->Blist(i,j) = rootr["relBlist"][i][j].asDouble();
		}
	}	
    cout<<"=================Slist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================Blist================="<<endl;
    cout<<this->Blist<<endl;
	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["relM"][i][j].asDouble();
		}
	}		
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbr(i,j) = rootr["Tbr"][i][j].asDouble();
		}
	}	 
    for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbl(i,j) = rootr["Tbl"][i][j].asDouble();
		}
	}	    
    cout<<"=================M================="<<endl;
    cout<<this->M<<endl;    

    cout<<"=================Tbr================="<<endl;
    cout<<this->Tbr<<endl;        

    cout<<"=================Tbl================="<<endl;
    cout<<this->Tbl<<endl;        

    
	cout<<"END MRSetup"<<endl;
}
