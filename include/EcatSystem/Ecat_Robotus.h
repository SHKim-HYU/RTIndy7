/*
 * Ecat_robotus.h
 *
 *  Created on: 2018. 11. 1.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_ROBOTUS_H_
#define ECATSYSTEM_ECAT_ROBOTUS_H_


#include "Ecat_Slave.h"
#include "PDOConfig.h"

#define FT_START_DEVICE 0x0B
#define FT_STOP_DEVICE 0x0C
#define FT_SET_OUTPUT_RATE 0x0F
#define FT_GET_OUTPUT_RATE 0x10
#define FT_SET_BIAS 0x11
#define FT_BIAS_SUB 0x01
#define FT_UNBIAS_SUB 0x00

namespace hyuEcat{

class EcatRobotus : public Slave{

public:
	EcatRobotus():Slave(RobotusFT_VendorID, Robotus_ProductCode)
	{
		DataConfig.u8Param[0] = 0x00;
		DataConfig.u8Param[1] = 0x00;
		DataConfig.u8Param[2] = 0x00;
		DataConfig.u8Param[3] = FT_START_DEVICE;
	}
	virtual ~EcatRobotus(){}

	bool Initialized() const{return m_initialized;}
	int getOverloadStatus() const{return m_OverloadStatus;}
	int getErrorCode() const{return m_ErrorFlag;}
	void setConfigureIndex(int8_t index, uint8_t subindex)
	{
		DataConfig.u8Param[0] = 0x00;
		DataConfig.u8Param[1] = 0x00;
		DataConfig.u8Param[2] = subindex;
		DataConfig.u8Param[3] = index;
		return;
	}



	virtual void processData(size_t index, uint8_t* domain_address)
	{
		switch(index){
		case 0:
		case 1:
			//if(m_initialized && isBiased == 1)
			//{
			//	setConfigureIndex(FT_SET_BIAS, FT_UNBIAS_SUB);
			//	isBiased++;
			//}
			//else if(m_initialized && isBiased == 2)
			//{
			//	setConfigureIndex(FT_SET_BIAS, FT_BIAS_SUB);
			//	isBiased++;
			//}
			//else if(m_initialized && isBiased == 2)
			//{
			//	setConfigureIndex(FT_START_DEVICE, 0);
			//	isBiased++;
			//}
			//else
			//{
			//	setConfigureIndex(FT_START_DEVICE, 0);
			//	m_initialized = true;
			//}


			EC_WRITE_U32(domain_address, DataConfig.u32Param);
			break;
		case 2:
			EC_WRITE_U32(domain_address, 0);
			break;
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
		case 17:
			m_DF[index-2] = EC_READ_U8(domain_address);
			break;
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
			m_Raw[index-18] = EC_READ_S16(domain_address);
			break;
		case 24:
			m_OverloadStatus = ConfirmOverload(EC_READ_U8(domain_address));
			break;
		case 25:
			m_ErrorFlag = ConfirmErrorCode(EC_READ_U8(domain_address));
			break;
        default:
            std::cout << "WARNING. Robotus FT pdo index out of range." << std::endl;

		}
	}

	virtual const ec_sync_info_t* syncs(){ return &RobotusFT_syncs[0]; }

	virtual size_t syncSize(){
		return sizeof(RobotusFT_syncs)/sizeof(ec_sync_info_t);
	}

	virtual const ec_pdo_entry_info_t* channels(){
		return RobotusFT_pdo_entries;
	}

	virtual void domains(DomainMap& domains) const{
		domains = domains_;
	}

	union ConfigParam
	{
		uint8_t index[4];
		uint32_t Config;
	};


	ConfigParam m_config;
	uint32_t m_ConfigParam[2] 	= {0,};
	uint8_t m_DF[16] 			= {0,};
	int16_t m_Raw[6] 			= {0,};


private:
	union DeviceConfig
	{
		uint8_t u8Param[4];
		uint32_t u32Param;
	};

	uint8_t m_OverloadStatus 	= 0;
	uint8_t m_ErrorFlag 		= 0;
	DeviceConfig DataConfig;

	DomainMap domains_ ={
			{0, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,20, 21, 22, 23, 24, 25}}
	};
	bool m_initialized = false;

	enum Overload
	{
		Overload_Clear,
		Torque_Z_axis = 1,
		Torque_Y_axis = 2,
		Torque_X_axis = 3,
		Force_Z_axis,
		Force_Y_axis,
		Force_X_axis,
	};

	enum ErrorCode
	{
		No_Error,
		Unsupported_Command,
		Out_of_Range,
		Failed_to_Set_Parameters
	};

	struct DeviceState{
		uint8_t state;
		uint8_t filter;

	};



	Overload ConfirmOverload(uint8_t OverloadStatus)
	{
		if( (OverloadStatus & 0b00111111) == 0b00000001 )
			return Torque_Z_axis;
		else if( (OverloadStatus & 0b00111111) == 0b00000010 )
			return Torque_Y_axis;
		else if( (OverloadStatus & 0b00111111) == 0b00000100 )
			return Torque_X_axis;
		else if( (OverloadStatus & 0b00111111) == 0b00001000 )
			return Force_Z_axis;
		else if( (OverloadStatus & 0b00111111) == 0b00010000 )
			return Force_Y_axis;
		else if( (OverloadStatus & 0b00111111) == 0b00100000 )
			return Force_X_axis;

		return Overload_Clear;
	}

	ErrorCode ConfirmErrorCode(uint8_t ErrorFlag)
	{
		if( ErrorFlag == 0x01 )
			return Unsupported_Command;
		else if( ErrorFlag == 0x02 )
			return Out_of_Range;
		else if( ErrorFlag == 0x03 )
			return Failed_to_Set_Parameters;

		return No_Error;
	}

	int isBiased = 0;


};

}

#endif /* ECATSYSTEM_ECAT_ROBOTUS_H_ */
