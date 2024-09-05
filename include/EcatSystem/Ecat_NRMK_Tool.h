/*
 * Ecat_NRMK_Tool.h
 *
 *  Created on: 2024. 04. 07.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_NRMK_Tool_H_
#define ECATSYSTEM_ECAT_NRMK_Tool_H_

#include "Ecat_Slave.h"
#include "PDOConfig.h"


class EcatNRMK_Tool : public Slave
{
public:
public:
	EcatNRMK_Tool() : Slave(NRMK_Tool_VendorID, NRMK_Tool_ProductCode) {}
    virtual ~EcatNRMK_Tool() {}

    /** Returns true if NRMK_Tool has reached "operation enabled" state.
     *  The transition through the state machine is handled automatically. */

    // int NRMK_Tool_DeviceState(void) const {return state_;};

    /** Write the torque in Nm.
     *  User must first set max_torque_Nm_.
     *  Provided as a convenience to calculate target_torque_
     *  NOTE: function assumes [max_torque_ = 1000]  */
    void writeFTconfig(uint32_t config)
    {
 	   FT_configparam_ = config;
    }

	virtual void processData(size_t index, uint8_t* domain_address) //read and write PDO and if index is 8,
	{                                                               //check the state and change the flag of state
		// DATA READ WRITE
		switch(index)
		{
		//RxPDO
		case 0:
            
            EC_WRITE_U8(domain_address, iLed_);
            break;
		case 1:
            EC_WRITE_U8(domain_address, iGripper_);
            break;
		case 2:
            EC_WRITE_U32(domain_address, FT_configparam_);
            break;
		case 3:
            EC_WRITE_U8(domain_address, LED_mode_);
            break;
		case 4:
            EC_WRITE_U8(domain_address, LED_G_);
            break;
		case 5:
            EC_WRITE_U8(domain_address, LED_R_);
            break;
		case 6:
            EC_WRITE_U8(domain_address, LED_B_);
            break;

        //TxPDO    
		case 7:
			iStatus_ = EC_READ_U8(domain_address);
            break;
		case 8:
            iButton_ = EC_READ_U8(domain_address);
            break;			
		case 9:
            FT_Raw_F[0]= EC_READ_S16(domain_address);
            break;
        case 10:
            FT_Raw_F[1]= EC_READ_S16(domain_address);
            break;
		case 11:
            FT_Raw_F[2]= EC_READ_S16(domain_address);
            break;
        case 12:
            FT_Raw_T[0]= EC_READ_S16(domain_address);
            break;
        case 13:
            FT_Raw_T[1]= EC_READ_S16(domain_address);
            break;
        case 14:
            FT_Raw_T[2]= EC_READ_S16(domain_address);
            break;
        case 15:
            FT_OverloadStatus_= EC_READ_U8(domain_address);
            break;
        case 16:
            FT_ErrorFlag_= EC_READ_U8(domain_address);
            break;
		default:
			std::cout << "WARNING. NRMK_Tool pdo index out of range." << std::endl;
		}

		// CHECK FOR STATE CHANGE
		if (index==7) //if last entry  in domain
		{
			if (iStatus_ != last_iStatus_){
				// state_ = deviceState(status_word_);
				std::cout << "NRMK_Tool_POS: " << slave_position << " " << ", NRMK_Tool_STATE: " <<  iStatus_ << std::endl;
				}
			

			last_iStatus_ = iStatus_;
		}
    }

    virtual const ec_sync_info_t* syncs() { return &NRMK_Tool_syncs[0]; }

    virtual size_t syncSize() {
        return sizeof(NRMK_Tool_syncs)/sizeof(ec_sync_info_t);
    }

    virtual const ec_pdo_entry_info_t* channels() {
        return NRMK_Tool_pdo_entries;
    }

    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }


    uint8_t   iLed_              = 0;        // write
    uint8_t   iGripper_          = 0; 		// write
    uint32_t  FT_configparam_    = 0; 		// write
    uint8_t   LED_mode_          = 0; 		// write 
    uint8_t   LED_G_             = 0; 		// write 
    uint8_t   LED_R_             = 0;        // write
    uint8_t   LED_B_             = 0;        // write

    uint8_t   iStatus_           = 0;        // read
    uint32_t  iButton_           = 0; 		// read
    int16_t  FT_Raw_Fx_         = 0;        // read
    int16_t  FT_Raw_Fy_         = 0;        // read
    int16_t  FT_Raw_Fz_         = 0;        // read
    int16_t  FT_Raw_Tx_         = 0;        // read
    int16_t  FT_Raw_Ty_         = 0;        // read
    int16_t  FT_Raw_Tz_         = 0; 		// read
    int16_t FT_Raw_F[3]         = {0,};
    int16_t FT_Raw_T[3]         = {0,};
    uint8_t   FT_OverloadStatus_ = 0; 		// read
    uint8_t   FT_ErrorFlag_      = 0;        // read
    
    bool     digital_outputs_[6]        = {false}; // write
    bool     digital_inputs_[6]         = {false}; // read

private:

    uint32_t digital_output_            = 0; // write
    uint32_t digital_input_             = 0; // read (must be enabled in NRMK_Tool Motion Studio)

    DomainMap domains_ = {
        //{0, {0,1,2,3,4,5,6,7,8,9}}
    	{0, {0,1,2,3,4,5,6, 7,8,9,10,11,12,13,14,15,16}}
    };


    int last_iStatus_ = -1;


};

#endif /* ECATSYSTEM_ECAT_NRMK_Tool_H_ */