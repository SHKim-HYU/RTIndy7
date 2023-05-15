#include "PDOConfig.h"

ec_pdo_entry_info_t RobotusFT_pdo_entries[] =
{
		{0x7000, 0x01, 32}, //ConfigParam_1
		{0x7000, 0x02, 32}, //ConfigParam_2
		{0x6000, 0x01, 8}, 	//DF_1
		{0x6000, 0x02, 8}, 	//DF_2
		{0x6000, 0x03, 8}, 	//DF_3
		{0x6000, 0x04, 8}, 	//DF_4
		{0x6000, 0x05, 8}, 	//DF_5
		{0x6000, 0x06, 8}, 	//DF_6
		{0x6000, 0x07, 8}, 	//DF_7
		{0x6000, 0x08, 8}, 	//DF_8
		{0x6000, 0x09, 8}, 	//DF_9
		{0x6000, 0x0a, 8}, 	//DF_10
		{0x6000, 0x0b, 8}, 	//DF_11
		{0x6000, 0x0c, 8}, 	//DF_12
		{0x6000, 0x0d, 8}, 	//DF_13
		{0x6000, 0x0e, 8}, 	//DF_14
		{0x6000, 0x0f, 8}, 	//DF_15
		{0x6000, 0x10, 8}, 	//DF_16
		{0x6000, 0x11, 16}, //Raw_Fx
		{0x6000, 0x12, 16}, //Raw_Fy
		{0x6000, 0x13, 16}, //Raw_Fz
		{0x6000, 0x14, 16}, //Raw_Tx
		{0x6000, 0x15, 16}, //Raw_Ty
		{0x6000, 0x16, 16}, //Raw_Tz
		{0x6000, 0x17, 8}, 	//OverloadStatus
		{0x6000, 0x18, 8}, 	//ErrorFlag

};

ec_pdo_info_t RobotusFT_pdos[] = {
		{0x1600, 2, 	RobotusFT_pdo_entries + 0}, //OUT_GENERIC process data mapping
		{0x1a00, 24, 	RobotusFT_pdo_entries + 2}, //IN_GENERIC process data mapping
};

ec_sync_info_t RobotusFT_syncs[5] = {
		{0, EC_DIR_OUTPUT, 	0, NULL, 				EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 	0, NULL, 				EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 	1, RobotusFT_pdos + 0, 	EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 	1, RobotusFT_pdos + 1, 	EC_WD_DISABLE},
		{0xff}
};
// Index, Subindex, DataType
ec_pdo_entry_info_t Elmo_pdo_entries[] = {
    {0x607a, 0x00, 32}, 	/* Target position */
    {0x60ff, 0x00, 32}, 	/* Target velocity */
    {0x6071, 0x00, 16}, 	/* Target torque */
    {0x6072, 0x00, 16}, 	/* Maximal torque */
    {0x6040, 0x00, 16}, 	/* Controlword */
    {0x6060, 0x00, 8}, 		/* Modes of operation */
    {0x6064, 0x00, 32}, 	/* Position actual value */
    {0x6077, 0x00, 16}, 	/* Torque value */
    {0x6041, 0x00, 16}, 	/* Statusword */
    {0x6061, 0x00, 8}, 		/* Modes of operation display */
	{0x6069, 0x00, 32},		/* Velocity actual value*/
};


ec_pdo_info_t Elmo_pdos[] = {
    {0x1605, 6, Elmo_pdo_entries + 0}, /* RPDO6 Mapping */
    {0x1a02, 4, Elmo_pdo_entries + 6}, /* TPDO3 Mapping */
	{0x1A0f, 1, Elmo_pdo_entries + 10},
};


ec_sync_info_t Elmo_syncs[5] = {  //does sync mean what mode is?
    {0, EC_DIR_OUTPUT, 	0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 	0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 	1, Elmo_pdos + 0, EC_WD_ENABLE},
	{3, EC_DIR_INPUT, 	2, Elmo_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


// Index, Subindex, DataType
ec_pdo_entry_info_t NRMK_IO_Module_pdo_entries[] = {
    {0x7100, 0x01, 8}, /* ControlCode */
    {0x7100, 0x02, 8}, /* DO_5V */
    {0x7100, 0x03, 8}, /* TO */
    {0x7100, 0x04, 8}, /* DO */
    {0x7100, 0x05, 16}, /* AO1 */
    {0x7100, 0x06, 16}, /* AO2 */
    {0x7100, 0x07, 32}, /* FT_ConfigParam */
    {0x7100, 0x08, 8}, /* RS485_ConfigParam */
    {0x7100, 0x09, 8}, /* RS485_CMD */
    {0x7100, 0x0a, 8}, /* RS485_Tx_Cnt */
    {0x7100, 0x0b, 8}, /* RS485_Tx_D0 */
    {0x7100, 0x0c, 8}, /* RS485_Tx_D1 */
    {0x7100, 0x0d, 8}, /* RS485_Tx_D2 */
    {0x7100, 0x0e, 8}, /* RS485_Tx_D3 */
    {0x7100, 0x0f, 8}, /* RS485_Tx_D4 */
    {0x7100, 0x10, 8}, /* RS485_Tx_D5 */
    {0x7100, 0x11, 8}, /* RS485_Tx_D6 */
    {0x7100, 0x12, 8}, /* RS485_Tx_D7 */
    {0x7100, 0x13, 8}, /* RS485_Tx_D8 */
    {0x7100, 0x14, 8}, /* RS485_Tx_D9 */

    {0x6100, 0x01, 8}, /* StatusCode */
    {0x6100, 0x02, 8}, /* DI_5V */
    {0x6100, 0x03, 8}, /* DI1 */
    {0x6100, 0x04, 8}, /* DI2 */
    {0x6100, 0x05, 16}, /* AI1 */
    {0x6100, 0x06, 16}, /* AI2 */
    {0x6100, 0x07, 16}, /* FT_Raw_Fx */
    {0x6100, 0x08, 16}, /* FT_Raw_Fy */
    {0x6100, 0x09, 16}, /* FT_Raw_Fz */
    {0x6100, 0x0a, 16}, /* FT_Raw_Tx */
    {0x6100, 0x0b, 16}, /* FT_Raw_Ty */
    {0x6100, 0x0c, 16}, /* FT_Raw_Tz */
    {0x6100, 0x0d, 8}, /* FT_OverloadStatus */
    {0x6100, 0x0e, 8}, /* FT_ErrorFlag */
    {0x6100, 0x0f, 8}, /* RS485_Rx_Cnt */
    {0x6100, 0x10, 8}, /* RS485_Rx_D0 */
    {0x6100, 0x11, 8}, /* RS485_Rx_D1 */
    {0x6100, 0x12, 8}, /* RS485_Rx_D2 */
    {0x6100, 0x13, 8}, /* RS485_Rx_D3 */
    {0x6100, 0x14, 8}, /* RS485_Rx_D4 */
    {0x6100, 0x15, 8}, /* RS485_Rx_D5 */
    {0x6100, 0x16, 8}, /* RS485_Rx_D6 */
    {0x6100, 0x17, 8}, /* RS485_Rx_D7 */
    {0x6100, 0x18, 8}, /* RS485_Rx_D8 */
    {0x6100, 0x19, 8}, /* RS485_Rx_D9 */
};



// Index, Subindex, DataType
ec_pdo_entry_info_t NRMK_Drive_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* controlword */
    {0x607a, 0x00, 32}, /* target_position */
    {0x60ff, 0x00, 32}, /* target_velocity */
    {0x6071, 0x00, 16}, /* target_torque */
    {0x6060, 0x00, 8}, /* modes_of_operation */

    {0x6041, 0x00, 16}, /* statusword */
    {0x6064, 0x00, 32}, /* position_actual_value */
    {0x606c, 0x00, 32}, /* velocity_actual_value */
    {0x6077, 0x00, 16}, /* torque_actual_value */
    {0x6061, 0x00, 8}, /* modes_of_operation_display */
};




// Index, Subindex, DataType
ec_pdo_entry_info_t NRMK_Indy_Tool_pdo_entries[] = {
    {0x7000, 0x01, 8}, /* iLed */
    {0x7000, 0x02, 8}, /* iGripper */
    {0x7000, 0x03, 32},/* FT_ConfigParam */
    {0x7000, 0x04, 8}, /* LED_Mode */
    {0x7000, 0x05, 8}, /* LED_G */
    {0x7000, 0x06, 8}, /* LED_R */
    {0x7000, 0x07, 8}, /* LED_B */

    {0x6000, 0x01, 8}, /* iStatus */
    {0x6000, 0x02, 8}, /* iButton */
    {0x6000, 0x03, 16}, /* FT_Raw_Fx */
    {0x6000, 0x04, 16}, /* FT_Raw_Fy */
    {0x6000, 0x05, 16}, /* FT_Raw_Fz */
    {0x6000, 0x06, 16}, /* FT_Raw_Tx */
    {0x6000, 0x07, 16}, /* FT_Raw_Ty */
    {0x6000, 0x08, 16}, /* FT_Raw_Tz */
    {0x6000, 0x09, 8}, /* FT_OverloadStatus */
    {0x6000, 0x0a, 8}, /* FT_ErrorFlag */
};

