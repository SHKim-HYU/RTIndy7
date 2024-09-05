#include "PDOConfig.h"

// Index, Subindex, DataType
ec_pdo_entry_info_t iServo_pdo_entries[] = {
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

ec_pdo_info_t iServo_pdos[] = {
    {0x1600, 5, iServo_pdo_entries + 0}, /* Drive RxPDO */
    {0x1a00, 5, iServo_pdo_entries + 5}, /* Drive TxPDO */
};

ec_sync_info_t iServo_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, iServo_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, iServo_pdos + 1, EC_WD_DISABLE},
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


ec_sync_info_t Elmo_syncs[5] = { 
    {0, EC_DIR_OUTPUT, 	0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 	0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 	1, Elmo_pdos + 0, EC_WD_ENABLE},
	{3, EC_DIR_INPUT, 	2, Elmo_pdos + 1, EC_WD_DISABLE},
    {0xff}
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

ec_pdo_info_t NRMK_Drive_pdos[] = {
    {0x1600, 5, NRMK_Drive_pdo_entries + 0}, /* Drive RxPDO */
    {0x1a00, 5, NRMK_Drive_pdo_entries + 5}, /* Drive TxPDO */
};

ec_sync_info_t NRMK_Drive_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, NRMK_Drive_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, NRMK_Drive_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// Index, Subindex, DataType
ec_pdo_entry_info_t NRMK_Tool_pdo_entries[] = {
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

ec_pdo_info_t NRMK_Tool_pdos[] = {
    {0x1600, 7, NRMK_Tool_pdo_entries + 0}, /* HRI_OUT process data mapping */
    {0x1a00, 10, NRMK_Tool_pdo_entries + 7}, /* HRI_IN process data mapping */
};

ec_sync_info_t NRMK_Tool_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, NRMK_Tool_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, NRMK_Tool_pdos + 1, EC_WD_DISABLE},
    {0xff}
};