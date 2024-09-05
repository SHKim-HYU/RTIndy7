/*
 * Ecat_Slave.h
 *
 *  Created on: 2024. 04. 07.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_SLAVE_H_
#define ECATSYSTEM_ECAT_SLAVE_H_

#include "ecrt.h"
#include <map>
#include <vector>
#include <iostream>

class Slave
{
public:
	Slave(uint32_t vendor_id, uint32_t product_id): m_vendor_id(vendor_id), m_product_id(product_id){}
	virtual ~Slave(){}
	virtual void processData(size_t index, uint8_t* domain_address){}

	virtual const ec_sync_info_t* syncs(){
		return NULL;
	}

	virtual size_t syncSize(){
		return 0;
	}

	virtual const ec_pdo_entry_info_t* channels(){
		return NULL;
	}

	typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;

	virtual void domains(DomainMap& domains) const{}

	const uint32_t m_vendor_id;
	const uint32_t m_product_id;

	void setSlaveAlias(int alias){slave_alias = alias; return;}
	void setSlavePosition(int position){slave_position = position; return;}

	int getSlaveAlias(void){return slave_alias;}
	int getSlavePosition(void){return slave_position;}
protected:
	int slave_alias=0;
	int slave_position=0;

};


#endif /* ECATSYSTEM_ECAT_SLAVE_H_ */
