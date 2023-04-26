/*
 * Ecat_Elmo.h
 *
 *  Created on: 2018. 11. 15.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_ELMO_H_
#define ECATSYSTEM_ECAT_ELMO_H_

#include "Ecat_Slave.h"
#include "PDOConfig.h"



#define INDEX_HOMING_METHOD 		0x6098U
#define INDEX_HOMING_SPEED 			0x6099U		//subindex 0x01(high speed), subindex 0x02(low speed)
#define INDEX_HOMING_OFFSET 		0x607CU
#define INDEX_HOMING_CURRENT_LIMIT 	0x2020U 	//subindex 0x01 / OV[55]

namespace hyuEcat{

class EcatElmo : public Slave
{
public:
public:
	EcatElmo() : Slave(Elmo_VendorID, Elmo_ProductCode) {}
    virtual ~EcatElmo() {}

    /** Returns true if Elmo has reached "operation enabled" state.
     *  The transition through the state machine is handled automatically. */
    bool initialized() const {return initialized_;}
    bool isHoming() {return homing_;}
    void reHoming() {this->homing_ = 0;}

    int Elmo_DeviceState(void) const {return state_;};

    /** Write the torque in Nm.
     *  User must first set max_torque_Nm_.
     *  Provided as a convenience to calculate target_torque_
     *  NOTE: function assumes [max_torque_ = 1000]  */
    void writeTorqueNm(const double torque)
    {
        target_torque_ = 1000.0*torque/max_torque_;
    }
    void writeTorque(int16_t torque)
    {
 	   target_torque_ = torque;
    }

    int32_t HomingOffset(int position)
    {
    	return (unsigned int)m_HomingOffset[position];
    }

    int8_t HomingMethod(int position)
    {
    	return (unsigned char)m_HomingMethod[position];
    }

    uint32_t HomingSpeed(int position)
    {
    	return (unsigned int)m_HomingSpeed[position];
    }

    uint16_t HomingCurrentLimit(int position)
    {
    	return (unsigned short)m_HomingCurrentLimit[position];
    }



    /** Maximum torque in Nm,
     *  corresponding to the max current set in Elmo Motion Studio */
    double max_torque_Nm_ = 1800;


	virtual void processData(size_t index, uint8_t* domain_address) //read and write PDO and if index is 8,
	{                                                               //check the state and change the flag of state
		// DATA READ WRITE
		switch(index)
		{
		//RxPDO
		case 0:
			EC_WRITE_S32(domain_address, target_position_);
			break;
		case 1:
			EC_WRITE_S32(domain_address, target_velocity_);
			break;
		case 2:
			EC_WRITE_S16(domain_address, target_torque_);
			break;
		case 3:
			EC_WRITE_U16(domain_address, max_torque_);
			break;
		case 4:
			control_word_ = EC_READ_U16(domain_address);

			//initialization sequence
			control_word_ = transition(state_, control_word_);
			EC_WRITE_U16(domain_address, control_word_);
			break;
		case 5:
			EC_WRITE_S8(domain_address, mode_of_operation_);
			break;
		//TxPDO
		case 6:
			position_ = EC_READ_S32(domain_address);
			break;
		case 7:
			torque_ = EC_READ_S16(domain_address);
			break;
		case 8:
			status_word_ = EC_READ_U16(domain_address);
			state_ = deviceState(status_word_);
			break;
		case 9:
			mode_of_operation_display_= EC_READ_S8(domain_address);
			break;
		case 10:
			velocity_ = EC_READ_S32(domain_address);
			break;
		default:
			std::cout << "WARNING. Elmo Gold Whistle pdo index out of range." << std::endl;
		}

		// CHECK FOR STATE CHANGE
		if (index==8) //if last entry  in domain
		{
			if (status_word_ != last_status_word_){
				state_ = deviceState(status_word_);
				if (state_ != last_state_){
					#if 1
						std::cout << "ElMO_POS: " << slave_position << " " << ", ELMO_STATE: " << device_state_str_[state_] << std::endl;
					#endif
				}
			}
			if ((state_ == STATE_OPERATION_ENABLED) && (last_state_ == STATE_OPERATION_ENABLED)){
				initialized_ = true;
			}
			else {
				initialized_ = false;
			}
			if ((state_ == STATE_HOMING_NOT_START) && (last_state_ == STATE_HOMING_NOT_START)){
				homing_ready_ = true;
			}
			else {
				homing_ready_ = false;
			}
			last_status_word_ = status_word_;
			last_state_ = state_;
		}
    }

    virtual const ec_sync_info_t* syncs() { return &Elmo_syncs[0]; }

    virtual size_t syncSize() {
        return sizeof(Elmo_syncs)/sizeof(ec_sync_info_t);
    }

    virtual const ec_pdo_entry_info_t* channels() {
        return Elmo_pdo_entries;
    }

    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }



    int32_t  target_position_           = 0; 		// write
    int32_t  target_velocity_           = 0; 		// write
    int16_t  target_torque_             = 0; 		// write (max torque (max current) = 1000)
    uint16_t max_torque_                = 1800; 	// write (max current = 1000, as set in Elmo Motion Studio)
    uint16_t control_word_              = 0; 		// write
    int8_t   mode_of_operation_         = 0; 		// write (use enum ModeOfOperation for convenience)

    int32_t  position_                  = 0; 		// read
    int32_t  velocity_					= 0;		// read
    int16_t  torque_                    = 0; 		// read
    uint16_t status_word_               = 0; 		// read
    int8_t   mode_of_operation_display_ = 0; 		// read
    int32_t  absolute_position_         = 0; 		// read
    int16_t  analog_input_              = 0; 		// read

    bool     digital_outputs_[6]        = {false}; // write
    bool     digital_inputs_[6]         = {false}; // read


    enum ModeOfOperation
    {
        MODE_NO_MODE                = 0,
        MODE_PROFILED_POSITION      = 1,
        MODE_PROFILED_VELOCITY      = 3,
        MODE_PROFILED_TORQUE        = 4,
        MODE_HOMING                 = 6,
        MODE_INTERPOLATED_POSITION  = 7,
        MODE_CYCLIC_SYNC_POSITION   = 8,
        MODE_CYCLIC_SYNC_VELEOCITY  = 9,
        MODE_CYCLIC_SYNC_TORQUE     = 10
    };

private:

    uint32_t digital_output_            = 0; // write
    uint32_t digital_input_             = 0; // read (must be enabled in Elmo Motion Studio)

    DomainMap domains_ = {
        //{0, {0,1,2,3,4,5,6,7,8,9}}
    	{0, {0,1,2,3,4,5,6,7,8,9,10}}
    };


//========================================================
// ELMO SPECIFIC
//========================================================

    enum DeviceState
    {
        STATE_UNDEFINED 					= 0,
        STATE_START 						= 1,
        STATE_NOT_READY_TO_SWITCH_ON		= 3,
        STATE_SWITCH_ON_DISABLED			= 4,
        STATE_READY_TO_SWITCH_ON			= 5,
        STATE_SWITCH_ON						= 6,
        STATE_OPERATION_ENABLED				= 7,
        STATE_QUICK_STOP_ACTIVE				= 8,
        STATE_FAULT_REACTION_ACTIVE			= 9,
        STATE_FAULT							= 10,
		STATE_HOMING_PROGRESS				= 11,
		STATE_HOMING_NOT_START				= 12,
		STATE_HOMING_ATTAINED_NOT_REACHED 	= 13,
		STATE_HOMING_COMPLITE				= 14,
		STATE_HOMING_ERROR					= 15,
		STATE_HOMING_UNDIFINED
    };

    std::map<DeviceState,std::string> device_state_str_ = {
         {STATE_START,                  	"Start"},
         {STATE_NOT_READY_TO_SWITCH_ON, 	"Not Ready to Switch On"},
         {STATE_SWITCH_ON_DISABLED,     	"Switch on Disabled"},
         {STATE_READY_TO_SWITCH_ON,     	"Ready to Switch On"},
         {STATE_SWITCH_ON,              	"Switch On"},
         {STATE_OPERATION_ENABLED,      	"Operation Enabled"},
         {STATE_QUICK_STOP_ACTIVE,      	"Quick Stop Active"},
         {STATE_FAULT_REACTION_ACTIVE,  	"Fault Reaction Active"},
         {STATE_FAULT,                  	"Fault"},
		 {STATE_HOMING_PROGRESS,        	"Homing Progress"},
		 {STATE_HOMING_NOT_START,       	"Homing Not Start"},
		 {STATE_HOMING_ATTAINED_NOT_REACHED, "Homing Attained not reached"},
		 {STATE_HOMING_COMPLITE,        	"Homing Finished"},
		 {STATE_HOMING_ERROR,           	"Homing Error"},
		 {STATE_HOMING_UNDIFINED,       	"Homing Undefined"}
    };

    /** returns device state based upon the status_word */
    DeviceState deviceState(uint16_t status_word)
    {
        if      ((status_word & 0b01001111) == 0b00000000){
            return STATE_NOT_READY_TO_SWITCH_ON;
        }
        else if ((status_word & 0b01001111) == 0b01000000){
            return STATE_SWITCH_ON_DISABLED;
        }
        else if ((status_word & 0b01101111) == 0b00100001){
            return STATE_READY_TO_SWITCH_ON;
        }
        else if ((status_word & 0b01101111) == 0b00100011){
        	return STATE_SWITCH_ON;
        }
        else if ((status_word & 0b01101111) == 0b00100111){
        	if(mode_of_operation_display_ == MODE_HOMING){
        		if     ((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00000000)
        			return STATE_HOMING_PROGRESS;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00000100)
        			return STATE_HOMING_NOT_START;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00010000)
        			return STATE_HOMING_ATTAINED_NOT_REACHED;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00010100)
        		{
        			homing_ = true;
        			return STATE_HOMING_COMPLITE;
        		}
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00100000)
        		    return STATE_HOMING_ERROR;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00100100)
        		    return STATE_HOMING_ERROR;
        		else
        			return STATE_HOMING_UNDIFINED;
        	}
        	else{
        		return STATE_OPERATION_ENABLED;
        	}
        }
        else if ((status_word & 0b01101111) == 0b00000111){
            return STATE_QUICK_STOP_ACTIVE;
        }
        else if ((status_word & 0b01001111) == 0b00001111){
            return STATE_FAULT_REACTION_ACTIVE;
        }
        else if ((status_word & 0b01001111) == 0b00001000){
            return STATE_FAULT;
        }
        return STATE_UNDEFINED;
    }

    uint16_t transition(DeviceState state, uint16_t control_word)
    {
        switch(state)
        {
        case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
            return control_word;
        case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
        	return control_word;
        case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
            return ((control_word & 0b01111110) | 0b00000110);
        case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
            return ((control_word & 0b01110111) | 0b00000111);
        case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
            return ((control_word & 0b01110000) | 0b00001111);
        case STATE_OPERATION_ENABLED:       // -> GOOD
        	return ((control_word & 0b00000000) | 0b00001111);
        case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
            return ((control_word & 0b01111111) | 0b00001111);
        case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
            return control_word;
        case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
            return ((control_word & 0b11111111) | 0b10000000);
        case STATE_HOMING_PROGRESS:
        	return ((control_word & 0b00011111) | 0b00011111);
        case STATE_HOMING_NOT_START:
        	if(homing_ready_)
        		return ((control_word & 0b00011111) | 0b00011111);
        	else
        		return control_word;
        case STATE_HOMING_ATTAINED_NOT_REACHED:
        	return ((control_word & 0b00011111) | 0b00011111);
        case STATE_HOMING_COMPLITE:
        	return ((control_word & 0b00001111) | 0b00001111);
        case STATE_HOMING_ERROR:
        	return ((control_word & 0b00001111) | 0b00001111);
        case STATE_HOMING_UNDIFINED:
        	return ((control_word & 0b00011111) | 0b00011111);
        default:
            break;
        }
        return control_word;
    }

    int last_status_word_ = -1;
    DeviceState last_state_ = STATE_START;
    DeviceState state_ = STATE_START;

    bool initialized_ = false;
    bool homing_ = false;
    bool homing_ready_ = false;

    // 1axis :
    // 2axis : 30.07+1    160*2*2000
    // 3axis : 36.17    160*2000
    // 4axis : 62.29    100*2*2048
    // 5axis : 35.63    160*2*2000
    // 6axis : 20.75    160*2000

    /* 3 joint
	int32_t 	m_HomingOffset[3] = { -136490, 75727, 120023 };	// cnt
	int8_t 		m_HomingMethod[3] = { -4, -3, -3};										//
	uint32_t 	m_HomingSpeed[3] = { 17000, 17000, 17000 };					// RPM
	uint16_t	m_HomingCurrentLimit[3] = {700, 700, 700}; 			// Per Thoussand of reted torque
*/
    // 5 joint
	int32_t 	m_HomingOffset[6] = {-4416600, 8979600, 4354530, -858850, -5566400, -6264300};				// cnt
	int8_t 		m_HomingMethod[6] = { -2, -1, -1, -2, -2, -2};							//
	uint32_t 	m_HomingSpeed[6] = {1500000, 1500000, 1500000, 1500000, 1500000, 1500000};					// RPM
	uint16_t	m_HomingCurrentLimit[6] = {270, 500, 150, 150, 150, 150}; 					// Per Thoussand of rated torque

	/*
 	int32_t 	m_HomingOffset[5] = { -125776, 268289, 63561, 19518, -110000};				// cnt
	int8_t 		m_HomingMethod[5] = { -4, -3, -3, -3, -4};							//
	uint32_t 	m_HomingSpeed[5] = { 25000, 22000, 22000, 22000, 22000};					// RPM
	uint16_t	m_HomingCurrentLimit[5] = {1000, 400, 700, 700, 800};
	 */

};

}


#endif /* ECATSYSTEM_ECAT_ELMO_H_ */
