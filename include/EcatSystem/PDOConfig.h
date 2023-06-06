#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define RobotusFT_VendorID 	0x000008ee
#define Robotus_ProductCode 0x00000002

#define Elmo_VendorID  0x0000009a
#define Elmo_ProductCode 0x00030924

#define NRMK_VendorID  0x0000089a
#define NRMK_IO_Module_ProductCode 0x40000000
#define NRMK_Drive_ProductCode 0x30000000
#define NRMK_Indy_Tool_ProductCode 0x10000007

extern ec_pdo_entry_info_t 	RobotusFT_pdo_entries[];
extern ec_pdo_info_t 		RobotusFT_pdos[];
extern ec_sync_info_t 		RobotusFT_syncs[5];

extern ec_pdo_entry_info_t 	Elmo_pdo_entries[];
extern ec_pdo_info_t		Elmo_pdos[];
extern ec_sync_info_t		Elmo_syncs[5];

extern ec_pdo_entry_info_t 	NRMK_IO_Module_pdo_entries[];
extern ec_pdo_info_t 		NRMK_IO_Module_pdos[];
extern ec_sync_info_t 		NRMK_IO_Module_syncs[5];

extern ec_pdo_entry_info_t 	NRMK_Drive_pdo_entries[];
extern ec_pdo_info_t		NRMK_Drive_pdos[];
extern ec_sync_info_t		NRMK_Drive_syncs[5];

extern ec_pdo_entry_info_t 	NRMK_Indy_Tool_pdo_entries[];
extern ec_pdo_info_t		NRMK_Indy_Tool_pdos[];
extern ec_sync_info_t		NRMK_Indy_Tool_syncs[5];

