/*
 * Ecat_NRMK_Indy_Tool.h
 *
 *  Created on: 2018. 11. 15.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_NRMK_Indy_Tool_H_
#define ECATSYSTEM_ECAT_NRMK_Indy_Tool_H_

#include "Ecat_Slave.h"
#include "PDOConfig.h"



namespace hyuEcat{

class EcatNRMK_Indy_Tool : public Slave
{
public:
public:
	EcatNRMK_Indy_Tool() : Slave(NRMK_Indy_Tool_VendorID, NRMK_Indy_Tool_ProductCode) {}
    virtual ~EcatNRMK_Indy_Tool() {}

    /** Returns true if NRMK_Indy_Tool has reached "operation enabled" state.
     *  The transition through the state machine is handled automatically. */

    // int NRMK_Indy_Tool_DeviceState(void) const {return state_;};

    /** Write the torque in Nm.
     *  User must first set max_torque_Nm_.
     *  Provided as a convenience to calculate target_torque_
     *  NOTE: function assumes [max_torque_ = 1000]  */

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
            FT_Raw_Fx_= EC_READ_S16(domain_address);
            break;
        case 10:
            FT_Raw_Fy_= EC_READ_S16(domain_address);
            break;
		case 11:
            FT_Raw_Fz_= EC_READ_S16(domain_address);
            break;
        case 12:
            FT_Raw_Tx_= EC_READ_S16(domain_address);
            break;
        case 13:
            FT_Raw_Ty_= EC_READ_S16(domain_address);
            break;
        case 14:
            FT_Raw_Tz_= EC_READ_S16(domain_address);
            break;
        case 15:
            FT_OverloadStatus_= EC_READ_U8(domain_address);
            break;
        case 16:
            FT_ErrorFlag_= EC_READ_U8(domain_address);
            break;
		default:
			std::cout << "WARNING. NRMK_Indy_Tool pdo index out of range." << std::endl;
		}

		// CHECK FOR STATE CHANGE
		if (index==7) //if last entry  in domain
		{
			if (iStatus_ != last_iStatus_){
				// state_ = deviceState(status_word_);
				std::cout << "NRMK_Indy_Tool_POS: " << slave_position << " " << ", NRMK_Indy_Tool_STATE: " <<  iStatus_ << std::endl;
				}
			

			last_iStatus_ = iStatus_;
		}
    }

    virtual const ec_sync_info_t* syncs() { return &NRMK_Indy_Tool_syncs[0]; }

    virtual size_t syncSize() {
        return sizeof(NRMK_Indy_Tool_syncs)/sizeof(ec_sync_info_t);
    }

    virtual const ec_pdo_entry_info_t* channels() {
        return NRMK_Indy_Tool_pdo_entries;
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
    uint8_t   FT_OverloadStatus_ = 0; 		// read
    uint8_t   FT_ErrorFlag_      = 0;        // read
    
    bool     digital_outputs_[6]        = {false}; // write
    bool     digital_inputs_[6]         = {false}; // read


    // enum ModeOfOperation
    // {
    //     MODE_NO_MODE                = 0,
    //     MODE_PROFILED_POSITION      = 1,
    //     MODE_PROFILED_VELOCITY      = 3,
    //     MODE_PROFILED_TORQUE        = 4,
    //     MODE_INTERPOLATED_POSITION  = 7,
    //     MODE_CYCLIC_SYNC_POSITION   = 8,
    //     MODE_CYCLIC_SYNC_VELEOCITY  = 9,
    //     MODE_CYCLIC_SYNC_TORQUE     = 10
    // };

private:

    uint32_t digital_output_            = 0; // write
    uint32_t digital_input_             = 0; // read (must be enabled in NRMK_Indy_Tool Motion Studio)

    DomainMap domains_ = {
        //{0, {0,1,2,3,4,5,6,7,8,9}}
    	{0, {0,1,2,3,4,5,6, 7,8,9,10,11,12,13,14,15,16}}
    };


//========================================================
// NRMK_Indy_Tool SPECIFIC
//========================================================

    // enum DeviceState
    // {
    //     STATE_UNDEFINED 					= 0,
    //     STATE_START 						= 1,
    //     STATE_NOT_READY_TO_SWITCH_ON		= 3,
    //     STATE_SWITCH_ON_DISABLED			= 4,
    //     STATE_READY_TO_SWITCH_ON			= 5,
    //     STATE_SWITCH_ON						= 6,
    //     STATE_OPERATION_ENABLED				= 7,
    //     STATE_QUICK_STOP_ACTIVE				= 8,
    //     STATE_FAULT_REACTION_ACTIVE			= 9,
    //     STATE_FAULT							= 10
    // };

    // std::map<DeviceState,std::string> device_state_str_ = {
    //      {STATE_START,                  	"Start"},
    //      {STATE_NOT_READY_TO_SWITCH_ON, 	"Not Ready to Switch On"},
    //      {STATE_SWITCH_ON_DISABLED,     	"Switch on Disabled"},
    //      {STATE_READY_TO_SWITCH_ON,     	"Ready to Switch On"},
    //      {STATE_SWITCH_ON,              	"Switch On"},
    //      {STATE_OPERATION_ENABLED,      	"Operation Enabled"},
    //      {STATE_QUICK_STOP_ACTIVE,      	"Quick Stop Active"},
    //      {STATE_FAULT_REACTION_ACTIVE,  	"Fault Reaction Active"},
    //      {STATE_FAULT,                  	"Fault"}
    // };

    // /** returns device state based upon the status_word */
    // DeviceState deviceState(uint16_t status_word)
    // {
    //     if      ((status_word & 0b01001111) == 0b00000000){
    //         return STATE_NOT_READY_TO_SWITCH_ON;
    //     }
    //     else if ((status_word & 0b01001111) == 0b01000000){
    //         return STATE_SWITCH_ON_DISABLED;
    //     }
    //     else if ((status_word & 0b01101111) == 0b00100001){
    //         return STATE_READY_TO_SWITCH_ON;
    //     }
    //     else if ((status_word & 0b01101111) == 0b00100011){
    //     	return STATE_SWITCH_ON;
    //     }
    //     else if ((status_word & 0b01101111) == 0b00100111){
    // 		return STATE_OPERATION_ENABLED;        	
    //     }
    //     else if ((status_word & 0b01101111) == 0b00000111){
    //         return STATE_QUICK_STOP_ACTIVE;
    //     }
    //     else if ((status_word & 0b01001111) == 0b00001111){
    //         return STATE_FAULT_REACTION_ACTIVE;
    //     }
    //     else if ((status_word & 0b01001111) == 0b00001000){
    //         return STATE_FAULT;
    //     }
    //     return STATE_UNDEFINED;
    // }



    int last_iStatus_ = -1;
    // DeviceState last_state_ = STATE_START;
    // DeviceState state_ = STATE_START;




};

}


#endif /* ECATSYSTEM_ECAT_NRMK_Indy_Tool_H_ */
