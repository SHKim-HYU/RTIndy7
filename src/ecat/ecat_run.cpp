#include "ecat_run.h"
const int 	 zeroPos[NUM_AXIS] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio[NUM_AXIS] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC[NUM_AXIS] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK[NUM_AXIS] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ[NUM_AXIS] = {-1,-1,1,-1,-1,1};
const int 	 dirTau[NUM_AXIS] = {-1,-1,1,-1,-1,1};

double force_divider =50.0;
double torque_divider = 2000.0;

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

	float FTData[6]={0,};
	 nrmk_master.readBufferFT(FTData);
	
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
	// _RFTECATV1.OutParam.FTRawFx = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawFx);
    // _RFTECATV1.OutParam.FTRawFy = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawFy);
    // _RFTECATV1.OutParam.FTRawFz = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawFz);
    // _RFTECATV1.OutParam.FTRawTx = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawTx);
    // _RFTECATV1.OutParam.FTRawTy = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawTy);
    // _RFTECATV1.OutParam.FTRawTz = EC_READ_S32(_systemVars->_txDomain_pd + _RFTECATV1.offFTRawTz);
	// Update RFT data
	info.act.F(0) = FTData[0];
	info.act.F(1) = FTData[1];
	info.act.F(2) =  FTData[2];
	info.act.F(3) =  FTData[3];
	info.act.F(4) = FTData[4];
	info.act.F(5) =  FTData[5];
	// std::cout<<info.act.F.transpose()<<std::endl;

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