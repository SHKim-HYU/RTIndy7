/*
 * Ecat_Master.cpp
 *
 *  Created on: 2024. 03. 04.
 *      Author: Sunhong Kim
 */

#include "Ecat_Master.h"
#include "Ecat_Slave.h"

#include <unistd.h>
#include <iostream>
#include <sstream>
#include <sys/resource.h>
#include <sys/mman.h>

Master::DomainInfo::DomainInfo(ec_master_t* master)
{
	domain = ecrt_master_create_domain(master);
	if(domain == NULL){
		printWarning("Err: Failed to create domain!");
		return;
	}
	const ec_pdo_entry_reg_t empty = {0};
	domain_regs.push_back(empty);
}

Master::DomainInfo::~DomainInfo(void)
{
	for(Entry& entry : entries){
		delete [] entry.offset;
		delete [] entry.bit_position;
	}
}


Master::Master(const int master) {
	p_master = ecrt_request_master(master);
	if(p_master == NULL){
		printWarning("Err: Failed to obtain master!");
		//ecrt_release_master(p_master);
		return;
	}
}

Master::~Master() {
	//for(SlaveInfo& slave : m_slave_info){

	//}
	for(auto& domain : m_domain_info){
		delete domain.second;
	}

}

void Master::SDOread(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data)
{
	size_t result_size;
	uint32_t abort_data;

	if(ecrt_master_sdo_upload(p_master, position, index, subindex, data, sizeof(data), &result_size, &abort_data) < 0){
		printf("WARNING. Master. [SDOread], index:0x%04X, subindex:0x%02X, unable to process\n", index, subindex);
	}
	return;
}

void Master::SDOwrite(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data)
{
	uint32_t abort_data;

	if(ecrt_master_sdo_download(p_master, position, index, subindex, data, sizeof(data), &abort_data) < 0){
		printf("WARNING. Master. [SDOwrite], index:0x%04X, subindex:0x%02X, unable to process\n", index, subindex);
	}
	return;
}

void Master::addSlave(uint16_t alias, uint16_t position, Slave* slave)
{
	slave->setSlaveAlias(alias);
	slave->setSlavePosition(position);

	SlaveInfo slave_info;
	slave_info.slave = slave;
	slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

	if(slave_info.config == NULL){
		printWarning("Err: Add slave, Failed to get slave configuration.");
		return;
	}
	m_slave_info.push_back(slave_info);

	size_t num_syncs = slave->syncSize();
	const ec_sync_info_t* syncs = slave->syncs();
	if(num_syncs > 0){
		int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
		if(pdos_status){
			printWarning("Err: Add slave Failed to configure PDOs");

			return;
		}
	}
	else{
		printWarning("Add slave. Sync size is zero for "
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
				+ ":"
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
	}

	Slave::DomainMap domain_map;
	slave->domains(domain_map);
	for(auto& iter : domain_map){
		unsigned int domain_index = iter.first;
		DomainInfo* domain_info = m_domain_info[domain_index];
		if(domain_info == NULL){
			domain_info = new DomainInfo(p_master);
			m_domain_info[domain_index] = domain_info;
		}
		registerPDOInDomain(alias, position, iter.second, domain_info, slave);
	}
}

void Master::addSlaveiServo(uint16_t alias, uint16_t position, Ecat_iServo* slave)
{
    slave->setSlaveAlias(alias);
    slave->setSlavePosition(position);

    SlaveInfo slave_info;  //instance of struct
    slave_info.slave = slave;
    slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

    if(slave_info.config == NULL){
        printWarning("Err: Add slave, Failed to get slave configuration.");
        return;
    }
    m_slave_info.push_back(slave_info);

    
    size_t num_syncs = slave->syncSize();
    const ec_sync_info_t* syncs = slave->syncs();
    if(num_syncs > 0){
        int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
        if(pdos_status){
            printWarning("Err: Add slave Failed to configure PDOs");

            return;
        }
    }
    else{
        printWarning("Add slave. Sync size is zero for "
                + static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
                + ":"
                + static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
    }

    Slave::DomainMap domain_map;
    slave->domains(domain_map);
    for(auto& iter : domain_map){
        unsigned int domain_index = iter.first;
        DomainInfo* domain_info = m_domain_info[domain_index];
        if(domain_info == NULL){
            domain_info = new DomainInfo(p_master);
            m_domain_info[domain_index] = domain_info;
        }
        registerPDOInDomain(alias, position, iter.second, domain_info, slave);
    }
}

void Master::addSlaveNRMKdrive(uint16_t alias, uint16_t position, EcatNRMK_Drive* slave)
{
    slave->setSlaveAlias(alias);
    slave->setSlavePosition(position);

    SlaveInfo slave_info;  //instance of struct
    slave_info.slave = slave;
    slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

    if(slave_info.config == NULL){
        printWarning("Err: Add slave, Failed to get slave configuration.");
        return;
    }
    m_slave_info.push_back(slave_info);

    
    size_t num_syncs = slave->syncSize();
    const ec_sync_info_t* syncs = slave->syncs();
    if(num_syncs > 0){
        int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
        if(pdos_status){
            printWarning("Err: Add slave Failed to configure PDOs");

            return;
        }
    }
    else{
        printWarning("Add slave. Sync size is zero for "
                + static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
                + ":"
                + static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
    }

    Slave::DomainMap domain_map;
    slave->domains(domain_map);
    for(auto& iter : domain_map){
        unsigned int domain_index = iter.first;
        DomainInfo* domain_info = m_domain_info[domain_index];
        if(domain_info == NULL){
            domain_info = new DomainInfo(p_master);
            m_domain_info[domain_index] = domain_info;
        }
        registerPDOInDomain(alias, position, iter.second, domain_info, slave);
    }
}

void Master::addSlaveNRMKtool(uint16_t alias, uint16_t position, EcatNRMK_Tool* slave)
{
    slave->setSlaveAlias(alias);
    slave->setSlavePosition(position);

    SlaveInfo slave_info;  //instance of struct
    slave_info.slave = slave;
    slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

    if(slave_info.config == NULL){
        printWarning("Err: Add slave, Failed to get slave configuration.");
        return;
    }
    m_slave_info.push_back(slave_info);

    
    size_t num_syncs = slave->syncSize();
    const ec_sync_info_t* syncs = slave->syncs();
    if(num_syncs > 0){
        int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
        if(pdos_status){
            printWarning("Err: Add slave Failed to configure PDOs");

            return;
        }
    }
    else{
        printWarning("Add slave. Sync size is zero for "
                + static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
                + ":"
                + static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
    }

    Slave::DomainMap domain_map;
    slave->domains(domain_map);
    for(auto& iter : domain_map){
        unsigned int domain_index = iter.first;
        DomainInfo* domain_info = m_domain_info[domain_index];
        if(domain_info == NULL){
            domain_info = new DomainInfo(p_master);
            m_domain_info[domain_index] = domain_info;
        }
        registerPDOInDomain(alias, position, iter.second, domain_info, slave);
    }
}

void Master::registerPDOInDomain(uint16_t alias, uint16_t position, std::vector<unsigned int>& channel_indices, DomainInfo* domain_info, Slave* slave)
{
    // expand the size of the domain
    unsigned int num_pdo_regs = channel_indices.size();
    size_t start_index = domain_info->domain_regs.size()-1; //empty element at end
    domain_info->domain_regs.resize(domain_info->domain_regs.size()+num_pdo_regs);

    // create a new entry in the domain
    DomainInfo::Entry domain_entry;
    domain_entry.slave        = slave;
    domain_entry.num_pdos     = num_pdo_regs;
    domain_entry.offset       = new unsigned int[num_pdo_regs];
    domain_entry.bit_position = new unsigned int[num_pdo_regs];
    domain_info->entries.push_back(domain_entry);

    Slave::DomainMap domain_map;
    slave->domains(domain_map);

    // add to array of pdos registrations
    const ec_pdo_entry_info_t* pdo_regs = slave->channels();
    for (size_t i=0; i<num_pdo_regs; ++i)
    {
        // create pdo entry in the domain
        ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index+i];
        pdo_reg.alias       = alias;
        pdo_reg.position    = position;
        pdo_reg.vendor_id   = slave->m_vendor_id;
        pdo_reg.product_code= slave->m_product_id;
        pdo_reg.index       = pdo_regs[channel_indices[i]].index;
        pdo_reg.subindex    = pdo_regs[channel_indices[i]].subindex;
        pdo_reg.offset      = &(domain_entry.offset[i]);
        pdo_reg.bit_position= &(domain_entry.bit_position[i]);


        // print the domain pdo entry
#if defined(_DEBUG)
        std::cout << "{" << pdo_reg.alias <<", "<< pdo_reg.position;
        std::cout << ", 0x" << std::hex << pdo_reg.vendor_id;
        std::cout << ", 0x" << std::hex << pdo_reg.product_code;
        std::cout << ", 0x" << std::hex << pdo_reg.index;
        std::cout << ", 0x" << std::hex << (int)pdo_reg.subindex;
        std::cout << "}" << std::dec << std::endl;
#endif
    }

    // set the last element to null
    ec_pdo_entry_reg_t empty = {0};
    domain_info->domain_regs.back() = empty;
}

void Master::activate()
{
    // register domain
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        bool domain_status = ecrt_domain_reg_pdo_entry_list(domain_info->domain, &(domain_info->domain_regs[0]));
        if (domain_status){
            printWarning("Activate. Failed to register domain PDO entries.");
            return;
        }
    }

    // activate master
    bool activate_status = ecrt_master_activate(p_master);
    if (activate_status){
        printWarning("Activate. Failed to activate master.");
        deactivate();
        return;
    }

    // retrieve domain data
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
        if (domain_info->domain_pd==NULL){
            printWarning("Activate. Failed to retrieve domain process data.");
            return;
        }
    }
}

void Master::activateWithDC(uint8_t RefPosition, uint32_t _SyncCycleNano)
{
    SyncCycleNano = _SyncCycleNano;
    // register domain
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        bool domain_status = ecrt_domain_reg_pdo_entry_list(domain_info->domain, &(domain_info->domain_regs[0]));
        if (domain_status){
            printWarning("Activate. Failed to register domain PDO entries.");
            return;
        }
    }

    // register sync manager
    for (SlaveInfo& slave : m_slave_info)
    {
    	// ecrt_slave_config_dc(slave.config, 0x0300, SyncCycleNano, 0, 0, 0 );
        ecrt_slave_config_dc(slave.config, 0x0000, SyncCycleNano, 0, 0, 0 );
    }
    printf("activeWithDC: ecrt_slave config dc is done\n");
    int res = ecrt_master_select_reference_clock(p_master, m_slave_info.at(RefPosition).config );  //error point
    if(res < 0) {
    	printf("ActiveWithDC: Failed to select reference clock:%d\n", res);
    }
    printf("activeWithDC: ecrt_slave reference clock is chosen\n");

    // activate master
    bool activate_status = ecrt_master_activate(p_master);
    if (activate_status){
        printWarning("Activate. Failed to activate master.");
        deactivate();
        return;
    }

    // retrieve domain data
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
        if (domain_info->domain_pd==NULL){
            printWarning("Activate. Failed to retrieve domain process data.");
            return;
        }
    }


}

void Master::SyncEcatMaster(uint64_t RefTime)
{
    uint32_t master_time, slave_time;
	ecrt_master_application_time(p_master, RefTime);
    // ecrt_master_64bit_reference_clock_time_queue(p_master);
    ecrt_master_reference_clock_time(p_master, &master_time);

	ecrt_master_sync_reference_clock(p_master);
	ecrt_master_sync_slave_clocks(p_master);
}

void Master::deactivate(void)
{
    TxUpdate();
    usleep(1000);
	printf("Release Master!\n");
	ecrt_release_master(p_master);
    
	// p_master = NULL;
	return;
}

void Master::update(unsigned int domain)
{
    // receive process data
    ecrt_master_receive(p_master);

    DomainInfo* domain_info = m_domain_info[domain];

    ecrt_domain_process(domain_info->domain);

    // check process data state (optional)
    checkDomainState(domain);

    // check for master and slave state change
    if (update_counter_ % check_state_frequency_ == 0){
        checkMasterState();
        checkSlaveStates();
    }

    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }

    // send process data
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(p_master);

    ++update_counter_;
}

void Master::TxUpdate(unsigned int domain)
{
    ecrt_master_receive(p_master);

    DomainInfo* domain_info = m_domain_info[domain];

    ecrt_domain_process(domain_info->domain);

#if defined(_DEBUG)// 1//
    // check process data state (optional)
    checkDomainState();
    checkMasterState();
    // check for master and slave state change
    if (update_counter_ % check_state_frequency_ == 0){
        checkSlaveStates();
    }
#endif
    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(p_master);

}

void Master::RxUpdate(unsigned int domain)
{
    ecrt_master_receive(p_master);

    DomainInfo* domain_info = m_domain_info[domain];

    ecrt_domain_process(domain_info->domain);

    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }

    // send process data
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(p_master);
}


void Master::checkDomainState(unsigned int domain)
{
    DomainInfo* domain_info = m_domain_info[domain];

    ec_domain_state_t ds;
    ecrt_domain_state(domain_info->domain, &ds);

    if (ds.working_counter != domain_info->domain_state.working_counter){
        printf("Domain: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain_info->domain_state.wc_state){
        printf("Domain: State %u.\n", ds.wc_state);
    }
    domain_info->domain_state = ds;
}

void Master::checkMasterState()
{
    ec_master_state_t ms;
    ecrt_master_state(p_master, &ms);

    if (ms.slaves_responding != m_master_state.slaves_responding){
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != m_master_state.al_states){
        printf("Master AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != m_master_state.link_up){
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    m_master_state = ms;
}


void Master::checkSlaveStates()
{
	int id_count = 0;
    for (SlaveInfo& slave : m_slave_info)
    {
        ec_slave_config_state_t s;
        ecrt_slave_config_state(slave.config, &s);

        if (s.al_state != slave.config_state.al_state){
            //this spams the terminal at initialization.
        	printf( "Slave %d:  ", id_count );
            printf(" State 0x%02X.\n", s.al_state);
        }
        if (s.online != slave.config_state.online){
        	printf( "Slave %d:  ", id_count );
            printf(" %s.\n", s.online ? "online" : "offline");
        }
        if (s.operational != slave.config_state.operational){
        	printf( "Slave %d:  ", id_count );
            printf(" %s operational.\n", s.operational ? "" : "Not ");
        }
        slave.config_state = s;
        ++id_count;
    }
}

int Master::SDO_ENCODER_RESOLUTION(int position)
{
    uint8_t *u32_data = new uint8_t[4];
    SDOread(position, OBJ_ENCODER_RESOLUTION, 1, u32_data);
    return (int)(u32_data[0] + (u32_data[1]<<8) + (u32_data[2]<<16) + (u32_data[3]<<24));
}

int Master::SDO_RATE_CURRENT(int position)
{
    uint8_t *u32_data = new uint8_t[4];
    SDOread(position, OBJ_RATE_CURRENT, 0, u32_data);
    return (int)(u32_data[0] + (u32_data[1]<<8) + (u32_data[2]<<16) + (u32_data[3]<<24));
}

int Master::SDO_TORQUE_CONSTANT(int position)
{
    uint8_t *u32_data = new uint8_t[4];
    SDOread(position, OBJ_TORQUE_CONSTANT, 0, u32_data);
    return (int)(u32_data[0] + (u32_data[1]<<8) + (u32_data[2]<<16) + (u32_data[3]<<24));
}

int Master::SDO_MOTOR_DIRECTION(int position)
{
    uint8_t *u8_data = new uint8_t[1];
    SDOread(position, OBJ_MOTOR_DIRECTION, 0, u8_data);
    
    int res = (int)(u8_data[0]);
	if (res == 0) res=1;	// 0 - nomal  -->  1 - nomal
	else res=-1;			// 1 - invert --> -1 - invert
	return res;
}

void Master::printWarning(const std::string& message)
{
    std::cout << "WARNING. Master. " << message << std::endl;
}

void Master::printWarning(const std::string& message, int position)
{
    std::cout << "WARNING. Master. " << "ID: " <<  position << message << std::endl;
}

