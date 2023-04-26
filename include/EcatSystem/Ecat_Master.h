/*
 * Master.h
 *
 *  Created on: 2018. 11. 1.
 *      Author: Administrator
 */

#ifndef ECATSYSTEM_ECAT_MASTER_H_
#define ECATSYSTEM_ECAT_MASTER_H_

#include "ecrt.h"
#include <string>
#include <vector>
#include <map>
#include <stdio.h>
#include <stdint.h>

#include "Ecat_Elmo.h"
#include "Ecat_NRMK_Drive.h"

namespace hyuEcat {
class Slave;
class Master {
public:
	Master(const int master = 0);
	virtual ~Master();
	void SDOread(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);
	void SDOwrite(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);
	void addSlave(uint16_t alias, uint16_t position, Slave* slave);
	void addSlaveElmo(uint16_t alias, uint16_t position, EcatElmo* slave);
	void addSlaveNRMK_Drive(uint16_t alias, uint16_t position, EcatNRMK_Drive* slave);
	void activate();
	void activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano);
	void SyncEcatMaster(uint64_t RefTime);
	void deactivate();
	void update(unsigned int domain = 0);
	void TxUpdate(unsigned int domain = 0);
	void RxUpdate(unsigned int domain = 0);

	void checkDomainState(unsigned int domain = 0);
	void checkMasterState();
	void checkSlaveStates();

private:
	volatile bool isRunning = false;

	struct DomainInfo;
	void registerPDOInDomain(uint16_t alias, uint16_t position, std::vector<unsigned int>& channel_indices,	DomainInfo* domain_info, Slave* slave);


	static void printWarning(const std::string& message);
	static void printWarning(const std::string& message, int position);

	ec_master_t *p_master;
	ec_master_state_t m_master_state = {};

	struct DomainInfo{
		DomainInfo(ec_master_t* master);
		~DomainInfo();

		ec_domain_t *domain = NULL;
		ec_domain_state_t domain_state = {};
		uint8_t *domain_pd = NULL;

		std::vector<ec_pdo_entry_reg_t> domain_regs;

		struct Entry{
			Slave* slave = NULL;
			int num_pdos = 0;
			unsigned int* offset = NULL;
			unsigned int* bit_position = NULL;
		};

		std::vector<Entry> entries;
	};

	std::map<unsigned int, DomainInfo*> m_domain_info;

	struct SlaveInfo{
		Slave* slave = NULL;
		ec_slave_config_t* config = NULL;
		ec_slave_config_state_t config_state = {0};
	};

	std::vector<SlaveInfo> m_slave_info;

    /** counter of control loops */
    unsigned long long update_counter_ = 0;

    /** frequency to check for master or slave state change.
     *  state checked every frequency_ control loops */
    unsigned int check_state_frequency_ = 100;


};

} /* namespace hyuEcat */

#endif /* ECATSYSTEM_ECAT_MASTER_H_ */
