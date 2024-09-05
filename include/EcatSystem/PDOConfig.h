/*! 
 *  @file PDOConfig.h
 *  @brief PDOConfig
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */

#ifndef PDOCONFIG_H_
#define PDOCONFIG_H_

#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define iServo_VendorID  0x00000000
#define iServo_ProductCode 0x00020001

#define Elmo_VendorID  0x0000009a
#define Elmo_ProductCode 0x00030924

#define NRMK_Drive_VendorID  0x0000089a
#define NRMK_Drive_ProductCode 0x30000000

#define NRMK_Tool_VendorID  0x0000089a
#define NRMK_Tool_ProductCode 0x10000007

extern ec_pdo_entry_info_t 	iServo_pdo_entries[];
extern ec_pdo_info_t		iServo_pdos[];
extern ec_sync_info_t		iServo_syncs[5];

extern ec_pdo_entry_info_t 	Elmo_pdo_entries[];
extern ec_pdo_info_t		Elmo_pdos[];
extern ec_sync_info_t		Elmo_syncs[5];

extern ec_pdo_entry_info_t 	NRMK_Drive_pdo_entries[];
extern ec_pdo_info_t		NRMK_Drive_pdos[];
extern ec_sync_info_t		NRMK_Drive_syncs[5];

extern ec_pdo_entry_info_t 	NRMK_Tool_pdo_entries[];
extern ec_pdo_info_t		NRMK_Tool_pdos[];
extern ec_sync_info_t		NRMK_Tool_syncs[5];

#endif /* PDOCONFIG_H_ */