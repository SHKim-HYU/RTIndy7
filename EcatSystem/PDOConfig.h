#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define RobotusFT_VendorID 	0x000008ee
#define Robotus_ProductCode 0x00000002

#define Elmo_VendorID  0x0000009a
#define Elmo_ProductCode 0x00030924

extern ec_pdo_entry_info_t 	RobotusFT_pdo_entries[];
extern ec_pdo_info_t 		RobotusFT_pdos[];
extern ec_sync_info_t 		RobotusFT_syncs[5];

extern ec_pdo_entry_info_t 	Elmo_pdo_entries[];
extern ec_pdo_info_t		Elmo_pdos[];
extern ec_sync_info_t		Elmo_syncs[5];
