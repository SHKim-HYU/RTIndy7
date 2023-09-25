//! \file Ecat_Master.h
//!
//! \brief Automatically generated header file for the EtherCAT system interface
//!
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP)
//
// Copyright (C) 2013-2016 Neuromeka <http://www.neuromeka.com>
#if !defined(_USE_LIB_)
#include "Ecat_Master.h"

#include <unistd.h>
#include <native/timer.h>

// for license mac----
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <ifaddrs.h>

#define MASK        0x85
#define hlen        0x19
#define packetsize  6
#define flen        0x82
#define packetlen   (hlen+packetsize+flen)

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

#ifdef __CB__
ec_pdo_info_t NRMK_IO_Module_pdos[] = {
    {0x1610, 20, NRMK_IO_Module_pdo_entries + 0}, /* IO_OUT process data mapping */
    {0x1a10, 25, NRMK_IO_Module_pdo_entries + 20}, /* IO_IN process data mapping */
};

ec_sync_info_t NRMK_IO_Module_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, NRMK_IO_Module_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, NRMK_IO_Module_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
#endif

ec_pdo_info_t NRMK_Indy_Tool_pdos[] = {
    {0x1600, 7, NRMK_Indy_Tool_pdo_entries + 0}, /* HRI_OUT process data mapping */
    {0x1a00, 10, NRMK_Indy_Tool_pdo_entries + 7}, /* HRI_IN process data mapping */
};

ec_sync_info_t NRMK_Indy_Tool_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, NRMK_Indy_Tool_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, NRMK_Indy_Tool_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


struct ecat_variables
{
    enum
    {
#ifdef __CB__
#ifdef __RP__
        NUM_RXDOMAIN_ENTRIES = 62,
        NUM_TXDOMAIN_ENTRIES = 70,      
#else
        NUM_RXDOMAIN_ENTRIES = 57,
        NUM_TXDOMAIN_ENTRIES = 65,      
#endif
#else
#ifdef __RP__
        NUM_RXDOMAIN_ENTRIES = 42,
        NUM_TXDOMAIN_ENTRIES = 45,         
#else
        NUM_RXDOMAIN_ENTRIES = 37,
        NUM_TXDOMAIN_ENTRIES = 40,         
#endif
#endif
    };


    void _registerRxDomainEntry(UINT32 Vendor, UINT32 Product, UINT32 SlaveAlias, UINT32 SlavePosition, UINT16 EntryIndex, UINT8 EntrySubIndex, UINT32 * Offset, UINT32 * BitOffset);
    void _registerTxDomainEntry(UINT32 Vendor, UINT32 Product, UINT32 SlaveAlias, UINT32 SlavePosition, UINT16 EntryIndex, UINT8 EntrySubIndex, UINT32 * Offset, UINT32 * BitOffset);

    uint64_t _getSysTimeDC(void);
    int _computeCtrlWrd(int Idx, UINT16 StatWrd, UINT16 & ControlWrd);
    int _initSlaves();
    int _initDomains();

    int _getMyMac(UINT8 myMac[], char aname[]);
    int _readLicenseFile(UINT8 allMac[]);
    bool _checkLicense();

    ec_master_t *_master;               /* EtherCAT Master */
    
    /* EtherCAT RxDomain for sending data */
    ec_pdo_entry_reg_t _rxDomain_regs[NUM_RXDOMAIN_ENTRIES + 1];
    ec_domain_t *_rxDomain;             //working domain for all working process data
    UINT8 *_rxDomain_pd;                // process data in working domain
    int _rxDomainIndex;

            /* EtherCAT TxDomain for sending data */
    ec_pdo_entry_reg_t _txDomain_regs[NUM_TXDOMAIN_ENTRIES + 1];
    ec_domain_t *_txDomain;             //working domain for all working process data
    UINT8 *_txDomain_pd;                // process data in working domain
    int _txDomainIndex;
            

    UINT32  _cycle_ns;
    INT64   _system_time_base;
    bool    _license;
};


NRMK_Master::NRMK_Master() : _systemVars(new ecat_variables())
{
} 

NRMK_Master::~NRMK_Master()
{
    _systemVars.reset();
}

int NRMK_Master::deinit()
{
    printf("Release Master!\n");
    ecrt_release_master(_systemVars->_master);
    _systemVars->_master = NULL;
    return 0;
}


void NRMK_Master::syncEcatMaster()
{
    // set master time in nano-seconds
    ecrt_master_application_time(_systemVars->_master, _systemVars->_getSysTimeDC());

    // sync reference clock to master
    ecrt_master_sync_reference_clock(_systemVars->_master);

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(_systemVars->_master);
}

int NRMK_Master::getRxDomainStatus()
{
    ec_domain_state_t ds = {};
    ecrt_domain_state(_systemVars->_rxDomain, &ds);

    return (int) ds.wc_state;
}

int NRMK_Master::getTxDomainStatus()
{
    ec_domain_state_t ds = {};
    ecrt_domain_state(_systemVars->_txDomain, &ds);

    return (int) ds.wc_state;
}

int NRMK_Master::getMasterStatus(unsigned int & NumSlaves, unsigned int & State)
{
    ec_master_state_t ms;
    ecrt_master_state(_systemVars->_master, &ms);

    if (ms.link_up)
    {
        NumSlaves = ms.slaves_responding;

        if (ms.al_states & OP)
            State = OP;
        else if (ms.al_states & SAFE_OP)
            State = SAFE_OP;
        else if (ms.al_states & PRE_OP)
            State = PRE_OP;
        else
            State = INIT;

        return 1;
    }
    else
    {
        NumSlaves = 0;
        State = 0x00;

        return 0;
    }
}

int NRMK_Master::getAxisEcatStatus(unsigned int AxisIdx, unsigned int & State)
{
    for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
        if (_NRMK_Drive[i].Index == AxisIdx)
        {
            ec_slave_config_state_t s;

            ecrt_slave_config_state(_NRMK_Drive[i].Config, &s);

            if (s.online)
            {
                if (s.al_state & OP)
                    State = OP;
                else if (s.al_state & SAFE_OP)
                    State = SAFE_OP;
                else if (s.al_state & PRE_OP)
                    State = PRE_OP;
                else
                    State = INIT;

                return 1;
            }
            else
            {
                return 0;
            }
        }
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
        if (_NRMK_IO_Module[i].Index == AxisIdx)
        {
            ec_slave_config_state_t s;

            ecrt_slave_config_state(_NRMK_IO_Module[i].Config, &s);

            if (s.online)
            {
                if (s.al_state & OP)
                    State = OP;
                else if (s.al_state & SAFE_OP)
                    State = SAFE_OP;
                else if (s.al_state & PRE_OP)
                    State = PRE_OP;
                else
                    State = INIT;

                return 1;
            }
            else
            {
                return 0;
            }
        }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
        if (_NRMK_Indy_Tool[i].Index == AxisIdx)
        {
            ec_slave_config_state_t s;

            ecrt_slave_config_state(_NRMK_Indy_Tool[i].Config, &s);

            if (s.online)
            {
                if (s.al_state & OP)
                    State = OP;
                else if (s.al_state & SAFE_OP)
                    State = SAFE_OP;
                else if (s.al_state & PRE_OP)
                    State = PRE_OP;
                else
                    State = INIT;

                return 1;
            }
            else
            {
                return 0;
            }
        }
                                
    return 0;
}

void NRMK_Master::_setMasterCycle(UINT32 DCCycle)
{
    _systemVars->_cycle_ns = DCCycle;
}

int NRMK_Master::_initMaster()
{
    // if (!_systemVars->_checkLicense())
    // {
    //     fprintf(stderr, "License failed!\n");
    //     return -1;
    // }
    
    unsigned int id = 0;
    _systemVars->_master = ecrt_request_master(id);
    if (!_systemVars->_master)
    {
        fprintf(stderr, "Unable to get requested master.\n");
        return -1;
    }

    //checkState();
    return 0;
}

int NRMK_Master::_activateMaster()
{
    if (_systemVars->_master != NULL)
    {
        printf("Activating master...\n");
        if (ecrt_master_activate(_systemVars->_master))
        {
            fprintf(stderr,"Master Activation failed.\n");\
            return -1;
        }
        
        usleep(1000);


        if (!(_systemVars->_rxDomain_pd = ecrt_domain_data(_systemVars->_rxDomain)))
        {
            printf("Failed to initialize RxDomain data pointer.\n");
            return -1;
        }

        if (!(_systemVars->_txDomain_pd = ecrt_domain_data(_systemVars->_txDomain)))
        {
            printf("Failed to initialize TxDomain data pointer.\n");
            return -1;
        }
        
        
        return 0;
    }

    return -1;
}

int ecat_variables::_getMyMac(UINT8 myMac[], char aname[])
{
    struct ifreq s;
    int fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);

    strcpy(s.ifr_name, aname);
    if (0 == ioctl(fd, SIOCGIFHWADDR, &s)) {
        int i;
        for (i = 0; i < packetsize; ++i)
            //printf("0x%02x, ", (unsigned char) s.ifr_addr.sa_data[i]);
            myMac[i]=(unsigned char) s.ifr_addr.sa_data[i];

        return 1;

    }
    else
        return 0;
}

int ecat_variables::_readLicenseFile(UINT8 allMac[])
{
    FILE * fp;
    long lSize;

    size_t result;
    int i, num, numofpacket;

    UINT8 mac[packetsize];
    fp = fopen ("/etc/devb.conf", "rb");
    if (fp==NULL){
        return -1;
    }
    // obtain file size:
    fseek (fp , 0 , SEEK_END);
    lSize = ftell (fp);
    rewind (fp);

    UINT8 buffer[lSize];
    if (buffer == NULL) return -2;

    // copy the file into the buffer:
    result = fread (buffer,1,lSize,fp);
    if (result != lSize){
        return -3;
    }
    numofpacket=lSize/packetlen;
    for (num=0; num<numofpacket; ++num)
    {
        memcpy(mac, buffer+packetlen*num+hlen, packetsize);
        int i;
        for (i=0; i<packetsize; ++i){
            //mac[i] ^=MASK;
            allMac[num*packetsize+i]=mac[i]^MASK;
        }
        //tracebuf(mac, packetsize, MASK); //FIXME: comment out this line
    }

    //printf("\n"); //FIXME: delete this line

    // terminate
    fclose (fp);
    return numofpacket;
}

bool ecat_variables::_checkLicense()
{
    if (!_license)
    {
        UINT8 myMac[6];
        UINT8 allMac[60000];
        UINT8 blankMac[6]={0,0,0,0,0,0};

        struct ifaddrs *ifaddr, *ifa;
        int family, n, i;

        if (getifaddrs(&ifaddr) == -1)
        {
           //perror("getifaddrs");
           //printf("License failed #1"); //network adapter does not exist
           return false;
        }

        int res=_readLicenseFile(allMac);
        if(res<0)
        {
            //printf("License failed #%i", res); //-1 file not found
            return false;
        }

        for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) { //scan all existing adapters
           if (ifa->ifa_addr == NULL)
               continue;

           family = ifa->ifa_addr->sa_family;
            if (family == AF_PACKET)
            {
                if(_getMyMac(myMac, ifa->ifa_name)==0){
                    //printf("License failed #1"); //network adapter does not exist
                    return false;
                }

                if (memcmp(myMac, blankMac, packetsize)==0) //loop back adapter, skip
                    continue;

                int i;
                //tracebuf(myMac, packetsize, 0); //FIXME: comment out this line
                //tracebuf(allMac, res*packetsize, 0);
                for(i=0; i<res; ++i){
                    if (memcmp(myMac, allMac+packetsize*i, packetsize)==0){
                        _license = true;
                        //break;
                        goto complete;
                    }
                }

            } // end of if (family == AF_PACKET)

        }//end of for ifa

complete:
        freeifaddrs(ifaddr);

        return _license;
    }
    else
        return _license;
}

void ecat_variables::_registerRxDomainEntry(UINT32 Vendor, UINT32 Product, UINT32 SlaveAlias, UINT32 SlavePosition, UINT16 EntryIndex, UINT8 EntrySubIndex, UINT32 * Offset, UINT32 * BitOffset)
{
    _rxDomain_regs[_rxDomainIndex].alias = SlaveAlias;
    _rxDomain_regs[_rxDomainIndex].position = SlavePosition;
    _rxDomain_regs[_rxDomainIndex].vendor_id = Vendor;
    _rxDomain_regs[_rxDomainIndex].product_code = Product;

    _rxDomain_regs[_rxDomainIndex].index = EntryIndex;
    _rxDomain_regs[_rxDomainIndex].subindex = EntrySubIndex;
    _rxDomain_regs[_rxDomainIndex].offset = Offset;
    _rxDomain_regs[_rxDomainIndex].bit_position = BitOffset;    // PDO entries are byte-aligned.

    _rxDomainIndex++;
}

void ecat_variables::_registerTxDomainEntry(UINT32 Vendor, UINT32 Product, UINT32 SlaveAlias, UINT32 SlavePosition, UINT16 EntryIndex, UINT8 EntrySubIndex, UINT32 * Offset, UINT32 * BitOffset)
{
    _txDomain_regs[_txDomainIndex].alias = SlaveAlias;
    _txDomain_regs[_txDomainIndex].position = SlavePosition;
    _txDomain_regs[_txDomainIndex].vendor_id = Vendor;
    _txDomain_regs[_txDomainIndex].product_code = Product;

    _txDomain_regs[_txDomainIndex].index = EntryIndex;
    _txDomain_regs[_txDomainIndex].subindex = EntrySubIndex;
    _txDomain_regs[_txDomainIndex].offset = Offset;
    _txDomain_regs[_txDomainIndex].bit_position = BitOffset;    // PDO entries are byte-aligned.

    _txDomainIndex++;
}


int ecat_variables::_computeCtrlWrd(int Idx, UINT16 StatWrd, UINT16 & ControlWrd)
{
    if (!(StatWrd & (1<<STATUSWORD_OPERATION_ENABLE_BIT)))
    {
        if (!(StatWrd & (1<<STATUSWORD_SWITCHED_ON_BIT))) {
            if (!(StatWrd & (1<<STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
                if ((StatWrd & (1<<STATUSWORD_FAULT_BIT))) {
                    ControlWrd = 0x80; //fault reset
                    return 0;
                }
                else
                {
                    ControlWrd = 0x06; //shutdown
                    return 0;
                }
            }
            else
            {
                ControlWrd = 0x07; //switch on
                return 1;
            }
        }
        else
        {
            ControlWrd = 0x0F; //switch on
            return 1;
        }
    }
    else
    {
        ControlWrd = 0x0F; //switch on
        return 1;
    }

    ControlWrd = 0;
    return 0;
}

uint64_t ecat_variables::_getSysTimeDC(void)
{
    RTIME time = rt_timer_read();

    if (0 > time) {
        //rt_printf("system_time_base: %lld, time: %llu\n", system_time_base, time);
        return time;
    }
    else {
        return time - 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void NRMK_Master::processTxDomain()
{
    ecrt_master_receive(_systemVars->_master); //RECEIVE A FRAME
    ecrt_domain_process(_systemVars->_txDomain);
    
    // TODO: Read feedback data from servos to axes via master      
    
    for (int i=0; i<NUM_NRMK_DRIVE_AXES; ++i)
    {
        _NRMK_Drive[i].OutParam.Statusword = EC_READ_U16(_systemVars->_txDomain_pd + _NRMK_Drive[i].offStatusword);
        _NRMK_Drive[i].OutParam.Positionactualvalue = EC_READ_S32(_systemVars->_txDomain_pd + _NRMK_Drive[i].offPositionactualvalue);
        _NRMK_Drive[i].OutParam.Velocityactualvalue = EC_READ_S32(_systemVars->_txDomain_pd + _NRMK_Drive[i].offVelocityactualvalue);
        _NRMK_Drive[i].OutParam.Torqueactualvalue = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Drive[i].offTorqueactualvalue);
        _NRMK_Drive[i].OutParam.Modesofoperationdisplay = EC_READ_S8(_systemVars->_txDomain_pd + _NRMK_Drive[i].offModesofoperationdisplay);
        
    }
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; ++i)
    {
        _NRMK_IO_Module[i].OutParam.StatusCode = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offStatusCode);
        _NRMK_IO_Module[i].OutParam.DI5V = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offDI5V);
        _NRMK_IO_Module[i].OutParam.DI1 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offDI1);
        _NRMK_IO_Module[i].OutParam.DI2 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offDI2);
        _NRMK_IO_Module[i].OutParam.AI1 = EC_READ_U16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offAI1);
        _NRMK_IO_Module[i].OutParam.AI2 = EC_READ_U16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offAI2);
        _NRMK_IO_Module[i].OutParam.FTRawFx = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawFx);
        _NRMK_IO_Module[i].OutParam.FTRawFy = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawFy);
        _NRMK_IO_Module[i].OutParam.FTRawFz = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawFz);
        _NRMK_IO_Module[i].OutParam.FTRawTx = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawTx);
        _NRMK_IO_Module[i].OutParam.FTRawTy = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawTy);
        _NRMK_IO_Module[i].OutParam.FTRawTz = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTRawTz);
        _NRMK_IO_Module[i].OutParam.FTOverloadStatus = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTOverloadStatus);
        _NRMK_IO_Module[i].OutParam.FTErrorFlag = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offFTErrorFlag);
        _NRMK_IO_Module[i].OutParam.RS485RxCnt = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxCnt);
        _NRMK_IO_Module[i].OutParam.RS485RxD0 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD0);
        _NRMK_IO_Module[i].OutParam.RS485RxD1 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD1);
        _NRMK_IO_Module[i].OutParam.RS485RxD2 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD2);
        _NRMK_IO_Module[i].OutParam.RS485RxD3 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD3);
        _NRMK_IO_Module[i].OutParam.RS485RxD4 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD4);
        _NRMK_IO_Module[i].OutParam.RS485RxD5 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD5);
        _NRMK_IO_Module[i].OutParam.RS485RxD6 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD6);
        _NRMK_IO_Module[i].OutParam.RS485RxD7 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD7);
        _NRMK_IO_Module[i].OutParam.RS485RxD8 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD8);
        _NRMK_IO_Module[i].OutParam.RS485RxD9 = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_IO_Module[i].offRS485RxD9);
        
    }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; ++i)
    {
        _NRMK_Indy_Tool[i].OutParam.IStatus = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offIStatus);
        _NRMK_Indy_Tool[i].OutParam.IButton = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offIButton);
        _NRMK_Indy_Tool[i].OutParam.FTRawFx = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawFx);
        _NRMK_Indy_Tool[i].OutParam.FTRawFy = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawFy);
        _NRMK_Indy_Tool[i].OutParam.FTRawFz = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawFz);
        _NRMK_Indy_Tool[i].OutParam.FTRawTx = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawTx);
        _NRMK_Indy_Tool[i].OutParam.FTRawTy = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawTy);
        _NRMK_Indy_Tool[i].OutParam.FTRawTz = EC_READ_S16(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTRawTz);
        _NRMK_Indy_Tool[i].OutParam.FTOverloadStatus = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTOverloadStatus);
        _NRMK_Indy_Tool[i].OutParam.FTErrorFlag = EC_READ_U8(_systemVars->_txDomain_pd + _NRMK_Indy_Tool[i].offFTErrorFlag);
        
    }
                
                    
    ecrt_domain_queue(_systemVars->_txDomain);
    
    syncEcatMaster();
    
    ecrt_master_send(_systemVars->_master); //SEND ALL QUEUED DATAGRAMS             
}

// [ToDo] Seperate Slaves to each modules
void NRMK_Master::processRxDomain()
{
    ecrt_master_receive(_systemVars->_master); //RECEIVE A FRAME
    ecrt_domain_process(_systemVars->_rxDomain);
    
    // TODO: Write control data from axes to servos via master

    for (int i=0; i<NUM_NRMK_DRIVE_AXES; ++i)
    {
        if (_servoOn[_NRMK_Drive[i].Index])
        {
            if (_systemVars->_computeCtrlWrd(_NRMK_Drive[i].Index, _NRMK_Drive[i].OutParam.Statusword, _NRMK_Drive[i].InParam.Controlword))
                _systemReady[_NRMK_Drive[i].Index] = 1;
        }
        else
        {
            _NRMK_Drive[i].InParam.Controlword = 0;
        }
                                        
        EC_WRITE_U16(_systemVars->_rxDomain_pd +  _NRMK_Drive[i].offControlword, _NRMK_Drive[i].InParam.Controlword);
        EC_WRITE_S32(_systemVars->_rxDomain_pd +  _NRMK_Drive[i].offTargetposition, _NRMK_Drive[i].InParam.Targetposition);
        EC_WRITE_S32(_systemVars->_rxDomain_pd +  _NRMK_Drive[i].offTargetvelocity, _NRMK_Drive[i].InParam.Targetvelocity);
        EC_WRITE_S16(_systemVars->_rxDomain_pd +  _NRMK_Drive[i].offTargettorque, _NRMK_Drive[i].InParam.Targettorque);
        EC_WRITE_S8(_systemVars->_rxDomain_pd +  _NRMK_Drive[i].offModesofoperation, _NRMK_Drive[i].InParam.Modesofoperation);
        
    }

#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; ++i)
    {
        _systemReady[_NRMK_IO_Module[i].Index] = 1;
                            
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offControlCode, _NRMK_IO_Module[i].InParam.ControlCode);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offDO5V, _NRMK_IO_Module[i].InParam.DO5V);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offTO, _NRMK_IO_Module[i].InParam.TO);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offDO, _NRMK_IO_Module[i].InParam.DO);
        EC_WRITE_U16(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offAO1, _NRMK_IO_Module[i].InParam.AO1);
        EC_WRITE_U16(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offAO2, _NRMK_IO_Module[i].InParam.AO2);
        EC_WRITE_U32(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offFTConfigParam, _NRMK_IO_Module[i].InParam.FTConfigParam);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485ConfigParam, _NRMK_IO_Module[i].InParam.RS485ConfigParam);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485CMD, _NRMK_IO_Module[i].InParam.RS485CMD);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxCnt, _NRMK_IO_Module[i].InParam.RS485TxCnt);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD0, _NRMK_IO_Module[i].InParam.RS485TxD0);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD1, _NRMK_IO_Module[i].InParam.RS485TxD1);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD2, _NRMK_IO_Module[i].InParam.RS485TxD2);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD3, _NRMK_IO_Module[i].InParam.RS485TxD3);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD4, _NRMK_IO_Module[i].InParam.RS485TxD4);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD5, _NRMK_IO_Module[i].InParam.RS485TxD5);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD6, _NRMK_IO_Module[i].InParam.RS485TxD6);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD7, _NRMK_IO_Module[i].InParam.RS485TxD7);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD8, _NRMK_IO_Module[i].InParam.RS485TxD8);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_IO_Module[i].offRS485TxD9, _NRMK_IO_Module[i].InParam.RS485TxD9);
        
    }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; ++i)
    {
        _systemReady[_NRMK_Indy_Tool[i].Index] = 1;
                            
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offILed, _NRMK_Indy_Tool[i].InParam.ILed);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offIGripper, _NRMK_Indy_Tool[i].InParam.IGripper);
        EC_WRITE_U32(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offFTConfigParam, _NRMK_Indy_Tool[i].InParam.FTConfigParam);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offLEDMode, _NRMK_Indy_Tool[i].InParam.LEDMode);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offLEDG, _NRMK_Indy_Tool[i].InParam.LEDG);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offLEDR, _NRMK_Indy_Tool[i].InParam.LEDR);
        EC_WRITE_U8(_systemVars->_rxDomain_pd +  _NRMK_Indy_Tool[i].offLEDB, _NRMK_Indy_Tool[i].InParam.LEDB);
        
    }
                
                    
    ecrt_domain_queue(_systemVars->_rxDomain);
    
    ecrt_master_send(_systemVars->_master); //SEND ALL QUEUED DATAGRAMS     
}

void NRMK_Master::readBuffer(int EntryID, void * const data)
{       
    switch (EntryID)
    {       
        case 0x60410:
        {
            UINT16 * const _statusword = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++){
                            _statusword[_NRMK_Drive[i].Index] = _NRMK_Drive[i].OutParam.Statusword;
                            if(_statusword[_NRMK_Drive[i].Index]!=_statusword_buff[i])
                                std::cout<<"index: "<<i<<", stwrd: "<<_NRMK_Drive[i].OutParam.Statusword<<std::endl;
                            _statusword_buff[i] = _statusword[_NRMK_Drive[i].Index];
                        }

        }
            break;          

        case 0x60640:
        {
            INT32 * const _positionactualvalue = static_cast<INT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++){
                            _positionactualvalue[_NRMK_Drive[i].Index] = _NRMK_Drive[i].OutParam.Positionactualvalue;
                        }

        }
            break;          

        case 0x606c0:
        {
            INT32 * const _velocityactualvalue = static_cast<INT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++){
                            _velocityactualvalue[_NRMK_Drive[i].Index] = _NRMK_Drive[i].OutParam.Velocityactualvalue;
                        }

        }
            break;          

        case 0x60770:
        {
            INT16 * const _torqueactualvalue = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++){
                            _torqueactualvalue[_NRMK_Drive[i].Index] = _NRMK_Drive[i].OutParam.Torqueactualvalue;
                        }

        }
            break;          

        case 0x60610:
        {
            INT8 * const _modesofoperationdisplay = static_cast<INT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++){
                            _modesofoperationdisplay[_NRMK_Drive[i].Index] = _NRMK_Drive[i].OutParam.Modesofoperationdisplay;
                        }

        }
            break;          

#ifdef __CB__
        case 0x61001:
        {
            UINT8 * const _statusCode = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _statusCode[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.StatusCode;
                        }

        }
            break;          

        case 0x61002:
        {
            UINT8 * const _dI5V = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _dI5V[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.DI5V;
                        }

        }
            break;          

        case 0x61003:
        {
            UINT8 * const _dI1 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _dI1[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.DI1;
                        }

        }
            break;          

        case 0x61004:
        {
            UINT8 * const _dI2 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _dI2[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.DI2;
                        }

        }
            break;          

        case 0x61005:
        {
            UINT16 * const _aI1 = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _aI1[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.AI1;
                        }

        }
            break;          

        case 0x61006:
        {
            UINT16 * const _aI2 = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _aI2[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.AI2;
                        }

        }
            break;          

        case 0x61007:
        {
            INT16 * const _fTRawFx = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawFx[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawFx;
                        }

        }
            break;          

        case 0x61008:
        {
            INT16 * const _fTRawFy = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawFy[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawFy;
                        }

        }
            break;          

        case 0x61009:
        {
            INT16 * const _fTRawFz = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawFz[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawFz;
                        }

        }
            break;          

        case 0x610010:
        {
            INT16 * const _fTRawTx = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawTx[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawTx;
                        }

        }
            break;          

        case 0x610011:
        {
            INT16 * const _fTRawTy = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawTy[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawTy;
                        }

        }
            break;          

        case 0x610012:
        {
            INT16 * const _fTRawTz = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTRawTz[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTRawTz;
                        }

        }
            break;          

        case 0x610013:
        {
            UINT8 * const _fTOverloadStatus = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTOverloadStatus[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTOverloadStatus;
                        }

        }
            break;          

        case 0x610014:
        {
            UINT8 * const _fTErrorFlag = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _fTErrorFlag[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.FTErrorFlag;
                        }

        }
            break;          

        case 0x610015:
        {
            UINT8 * const _rS485RxCnt = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxCnt[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxCnt;
                        }

        }
            break;          

        case 0x610016:
        {
            UINT8 * const _rS485RxD0 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD0[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD0;
                        }

        }
            break;          

        case 0x610017:
        {
            UINT8 * const _rS485RxD1 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD1[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD1;
                        }

        }
            break;          

        case 0x610018:
        {
            UINT8 * const _rS485RxD2 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD2[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD2;
                        }

        }
            break;          

        case 0x610019:
        {
            UINT8 * const _rS485RxD3 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD3[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD3;
                        }

        }
            break;          

        case 0x610020:
        {
            UINT8 * const _rS485RxD4 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD4[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD4;
                        }

        }
            break;          

        case 0x610021:
        {
            UINT8 * const _rS485RxD5 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD5[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD5;
                        }

        }
            break;          

        case 0x610022:
        {
            UINT8 * const _rS485RxD6 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD6[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD6;
                        }

        }
            break;          

        case 0x610023:
        {
            UINT8 * const _rS485RxD7 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD7[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD7;
                        }

        }
            break;          

        case 0x610024:
        {
            UINT8 * const _rS485RxD8 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD8[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD8;
                        }

        }
            break;          

        case 0x610025:
        {
            UINT8 * const _rS485RxD9 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++){
                            _rS485RxD9[_NRMK_IO_Module[i].Index] = _NRMK_IO_Module[i].OutParam.RS485RxD9;
                        }

        }
            break;          
#endif
        case 0x60001:
        {
            UINT8 * const _iStatus = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _iStatus[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.IStatus;
                        }

        }
            break;          

        case 0x60002:
        {
            UINT8 * const _iButton = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _iButton[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.IButton;
                        }

        }
            break;          

        case 0x60003:
        {
            INT16 * const _fTRawFx = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawFx[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawFx;
                        }

        }
            break;          

        case 0x60004:
        {
            INT16 * const _fTRawFy = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawFy[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawFy;
                        }

        }
            break;          

        case 0x60005:
        {
            INT16 * const _fTRawFz = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawFz[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawFz;
                        }

        }
            break;          

        case 0x60006:
        {
            INT16 * const _fTRawTx = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawTx[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawTx;
                        }

        }
            break;          

        case 0x60007:
        {
            INT16 * const _fTRawTy = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawTy[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawTy;
                        }

        }
            break;          

        case 0x60008:
        {
            INT16 * const _fTRawTz = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTRawTz[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTRawTz;
                        }

        }
            break;          

        case 0x60009:
        {
            UINT8 * const _fTOverloadStatus = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTOverloadStatus[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTOverloadStatus;
                        }

        }
            break;          

        case 0x600010:
        {
            UINT8 * const _fTErrorFlag = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++){
                            _fTErrorFlag[_NRMK_Indy_Tool[i].Index] = _NRMK_Indy_Tool[i].OutParam.FTErrorFlag;
                        }

        }
            break;          
                            
        default:    // Undefined Entry ID
            break;
    }
}

void NRMK_Master::writeBuffer(int EntryID, void * const data)
{
    switch (EntryID)
    {
        case 0x60400:
        {
            UINT16 * const _controlword = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
            {
                _NRMK_Drive[i].InParam.Controlword = _controlword[_NRMK_Drive[i].Index];
            }

        }
            break;                      

        case 0x607a0:
        {
            INT32 * const _targetposition = static_cast<INT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
            {
                _NRMK_Drive[i].InParam.Targetposition = _targetposition[_NRMK_Drive[i].Index];
            }

        }
            break;                      

        case 0x60ff0:
        {
            INT32 * const _targetvelocity = static_cast<INT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
            {
                _NRMK_Drive[i].InParam.Targetvelocity = _targetvelocity[_NRMK_Drive[i].Index];
            }

        }
            break;                      

        case 0x60710:
        {
            INT16 * const _targettorque = static_cast<INT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
            {
                _NRMK_Drive[i].InParam.Targettorque = _targettorque[_NRMK_Drive[i].Index];
            }

        }
            break;                      

        case 0x60600:
        {
            INT8 * const _modesofoperation = static_cast<INT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
            {
                _NRMK_Drive[i].InParam.Modesofoperation = _modesofoperation[_NRMK_Drive[i].Index];
            }

        }
            break;                      
#ifdef __CB__
        case 0x71001:
        {
            UINT8 * const _controlCode = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.ControlCode = _controlCode[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71002:
        {
            UINT8 * const _dO5V = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.DO5V = _dO5V[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71003:
        {
            UINT8 * const _tO = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.TO = _tO[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71004:
        {
            UINT8 * const _dO = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.DO = _dO[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71005:
        {
            UINT16 * const _aO1 = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.AO1 = _aO1[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71006:
        {
            UINT16 * const _aO2 = static_cast<UINT16 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.AO2 = _aO2[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71007:
        {
            UINT32 * const _fTConfigParam = static_cast<UINT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.FTConfigParam = _fTConfigParam[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71008:
        {
            UINT8 * const _rS485ConfigParam = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485ConfigParam = _rS485ConfigParam[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x71009:
        {
            UINT8 * const _rS485CMD = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485CMD = _rS485CMD[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710010:
        {
            UINT8 * const _rS485TxCnt = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxCnt = _rS485TxCnt[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710011:
        {
            UINT8 * const _rS485TxD0 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD0 = _rS485TxD0[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710012:
        {
            UINT8 * const _rS485TxD1 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD1 = _rS485TxD1[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710013:
        {
            UINT8 * const _rS485TxD2 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD2 = _rS485TxD2[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710014:
        {
            UINT8 * const _rS485TxD3 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD3 = _rS485TxD3[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710015:
        {
            UINT8 * const _rS485TxD4 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD4 = _rS485TxD4[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710016:
        {
            UINT8 * const _rS485TxD5 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD5 = _rS485TxD5[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710017:
        {
            UINT8 * const _rS485TxD6 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD6 = _rS485TxD6[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710018:
        {
            UINT8 * const _rS485TxD7 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD7 = _rS485TxD7[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710019:
        {
            UINT8 * const _rS485TxD8 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD8 = _rS485TxD8[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      

        case 0x710020:
        {
            UINT8 * const _rS485TxD9 = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
            {
                _NRMK_IO_Module[i].InParam.RS485TxD9 = _rS485TxD9[_NRMK_IO_Module[i].Index];
            }

        }
            break;                      
#endif
        case 0x70001:
        {
            UINT8 * const _iLed = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.ILed = _iLed[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      

        case 0x70002:
        {
            UINT8 * const _iGripper = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.IGripper = _iGripper[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      

        case 0x70003:
        {
            UINT32 * const _fTConfigParam = static_cast<UINT32 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.FTConfigParam = _fTConfigParam[_NRMK_Indy_Tool[i].Index];
            }
            // std::cout<<"FT Initialization done"<<_fTConfigParam[_NRMK_Indy_Tool[0].Index]<<std::endl;

        }
            break;                      

        case 0x70004:
        {
            UINT8 * const _lEDMode = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.LEDMode = _lEDMode[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      

        case 0x70005:
        {
            UINT8 * const _lEDG = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.LEDG = _lEDG[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      

        case 0x70006:
        {
            UINT8 * const _lEDR = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.LEDR = _lEDR[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      

        case 0x70007:
        {
            UINT8 * const _lEDB = static_cast<UINT8 * const>(data);
            
            for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
            {
                _NRMK_Indy_Tool[i].InParam.LEDB = _lEDB[_NRMK_Indy_Tool[i].Index];
            }

        }
            break;                      
                            
        default:    // Undefined Entry ID                   
            break;
    }
}

void NRMK_Master::readSDO(int EntryID, void * const data)
{
    switch (EntryID)
    {
            
        default:    // Undefined Entry ID
            break;
    }
}

void NRMK_Master::writeSDO(int EntryID, void * const data)
{
    switch (EntryID)
    {
            
        default:    // Undefined Entry ID
            break;
    }
}

int NRMK_Master::_initSlaves()
{
#ifdef __CB__
#ifdef __RP__
    _NRMK_Drive[0].Index = 1; _NRMK_Drive[0].Alias = 0; _NRMK_Drive[0].Position = 1;

    _NRMK_Drive[1].Index = 2; _NRMK_Drive[1].Alias = 0; _NRMK_Drive[1].Position = 2;

    _NRMK_Drive[2].Index = 3; _NRMK_Drive[2].Alias = 0; _NRMK_Drive[2].Position = 3;

    _NRMK_Drive[3].Index = 4; _NRMK_Drive[3].Alias = 0; _NRMK_Drive[3].Position = 4;

    _NRMK_Drive[4].Index = 5; _NRMK_Drive[4].Alias = 0; _NRMK_Drive[4].Position = 5;

    _NRMK_Drive[5].Index = 6; _NRMK_Drive[5].Alias = 0; _NRMK_Drive[5].Position = 6;

    _NRMK_Drive[6].Index = 7; _NRMK_Drive[6].Alias = 0; _NRMK_Drive[6].Position = 7;

    _NRMK_IO_Module[0].Index = 0; _NRMK_IO_Module[0].Alias = 0; _NRMK_IO_Module[0].Position = 0;

    _NRMK_Indy_Tool[0].Index = 8; _NRMK_Indy_Tool[0].Alias = 0; _NRMK_Indy_Tool[0].Position = 8;
#else
    _NRMK_Drive[0].Index = 1; _NRMK_Drive[0].Alias = 0; _NRMK_Drive[0].Position = 1;

    _NRMK_Drive[1].Index = 2; _NRMK_Drive[1].Alias = 0; _NRMK_Drive[1].Position = 2;

    _NRMK_Drive[2].Index = 3; _NRMK_Drive[2].Alias = 0; _NRMK_Drive[2].Position = 3;

    _NRMK_Drive[3].Index = 4; _NRMK_Drive[3].Alias = 0; _NRMK_Drive[3].Position = 4;

    _NRMK_Drive[4].Index = 5; _NRMK_Drive[4].Alias = 0; _NRMK_Drive[4].Position = 5;

    _NRMK_Drive[5].Index = 6; _NRMK_Drive[5].Alias = 0; _NRMK_Drive[5].Position = 6;

    _NRMK_IO_Module[0].Index = 0; _NRMK_IO_Module[0].Alias = 0; _NRMK_IO_Module[0].Position = 0;

    _NRMK_Indy_Tool[0].Index = 7; _NRMK_Indy_Tool[0].Alias = 0; _NRMK_Indy_Tool[0].Position = 7;
#endif
#else
#ifdef __RP__
    _NRMK_Drive[0].Index = 0; _NRMK_Drive[0].Alias = 0; _NRMK_Drive[0].Position = 0;

    _NRMK_Drive[1].Index = 1; _NRMK_Drive[1].Alias = 0; _NRMK_Drive[1].Position = 1;

    _NRMK_Drive[2].Index = 2; _NRMK_Drive[2].Alias = 0; _NRMK_Drive[2].Position = 2;

    _NRMK_Drive[3].Index = 3; _NRMK_Drive[3].Alias = 0; _NRMK_Drive[3].Position = 3;

    _NRMK_Drive[4].Index = 4; _NRMK_Drive[4].Alias = 0; _NRMK_Drive[4].Position = 4;

    _NRMK_Drive[5].Index = 5; _NRMK_Drive[5].Alias = 0; _NRMK_Drive[5].Position = 5;

    _NRMK_Drive[6].Index = 6; _NRMK_Drive[6].Alias = 0; _NRMK_Drive[6].Position = 6;

    _NRMK_Indy_Tool[0].Index = 7; _NRMK_Indy_Tool[0].Alias = 0; _NRMK_Indy_Tool[0].Position = 7;
#else
    _NRMK_Drive[0].Index = 0; _NRMK_Drive[0].Alias = 0; _NRMK_Drive[0].Position = 0;

    _NRMK_Drive[1].Index = 1; _NRMK_Drive[1].Alias = 0; _NRMK_Drive[1].Position = 1;

    _NRMK_Drive[2].Index = 2; _NRMK_Drive[2].Alias = 0; _NRMK_Drive[2].Position = 2;

    _NRMK_Drive[3].Index = 3; _NRMK_Drive[3].Alias = 0; _NRMK_Drive[3].Position = 3;

    _NRMK_Drive[4].Index = 4; _NRMK_Drive[4].Alias = 0; _NRMK_Drive[4].Position = 4;

    _NRMK_Drive[5].Index = 5; _NRMK_Drive[5].Alias = 0; _NRMK_Drive[5].Position = 5;

    _NRMK_Indy_Tool[0].Index = 6; _NRMK_Indy_Tool[0].Alias = 0; _NRMK_Indy_Tool[0].Position = 6;
#endif
#endif
    
    for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
    {
        _NRMK_Drive[i].Config = ecrt_master_slave_config(_systemVars->_master, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, NRMK_VendorID, NRMK_Drive_ProductCode);                   
        if (_NRMK_Drive[i].Config == NULL)
        {
            printf("There is no configuration on slave NRMK_DRIVE!\n");
            return -1;
        }

                            
        if (ecrt_slave_config_pdos(_NRMK_Drive[i].Config, EC_END, NRMK_Drive_syncs))
        {
            printf("Error in configuring PDOs for slave!\n");
            return -1;
        }
    }
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
    {
        _NRMK_IO_Module[i].Config = ecrt_master_slave_config(_systemVars->_master, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, NRMK_VendorID, NRMK_IO_Module_ProductCode);                   
        if (_NRMK_IO_Module[i].Config == NULL)
        {
            printf("There is no configuration on slave NRMK_IO_MODULE!\n");
            return -1;
        }

                            
        if (ecrt_slave_config_pdos(_NRMK_IO_Module[i].Config, EC_END, NRMK_IO_Module_syncs))
        {
            printf("Error in configuring PDOs for slave!\n");
            return -1;
        }
    }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
    {
        _NRMK_Indy_Tool[i].Config = ecrt_master_slave_config(_systemVars->_master, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, NRMK_VendorID, NRMK_Indy_Tool_ProductCode);                   
        if (_NRMK_Indy_Tool[i].Config == NULL)
        {
            printf("There is no configuration on slave NRMK_INDY_TOOL!\n");
            return -1;
        }

                            
        if (ecrt_slave_config_pdos(_NRMK_Indy_Tool[i].Config, EC_END, NRMK_Indy_Tool_syncs))
        {
            printf("Error in configuring PDOs for slave!\n");
            return -1;
        }
    }
    

    return 0;
}

int NRMK_Master::_initDomains()
{
    // Create Domain
    _systemVars->_rxDomain = ecrt_master_create_domain(_systemVars->_master);
    if (_systemVars->_rxDomain == NULL)
    {
        printf("Creating RxDomain failed!\n");
        return -1;
    }

    _systemVars->_txDomain = ecrt_master_create_domain(_systemVars->_master);
    if (_systemVars->_txDomain == NULL)
    {
        printf("Creating TxDomain failed!\n");
        return -1;
    }
    
                    
    // Register Entries
    for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
                {
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6040, 0, &(_NRMK_Drive[i].offControlword), &(_NRMK_Drive[i].bitoffControlword));  // Controlword
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x607a, 0, &(_NRMK_Drive[i].offTargetposition), &(_NRMK_Drive[i].bitoffTargetposition));    // Targetposition
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x60ff, 0, &(_NRMK_Drive[i].offTargetvelocity), &(_NRMK_Drive[i].bitoffTargetvelocity));    // Targetvelocity
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6071, 0, &(_NRMK_Drive[i].offTargettorque), &(_NRMK_Drive[i].bitoffTargettorque));    // Targettorque
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6060, 0, &(_NRMK_Drive[i].offModesofoperation), &(_NRMK_Drive[i].bitoffModesofoperation));    // Modesofoperation
                    
                }
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
                {
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 1, &(_NRMK_IO_Module[i].offControlCode), &(_NRMK_IO_Module[i].bitoffControlCode));  // ControlCode
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 2, &(_NRMK_IO_Module[i].offDO5V), &(_NRMK_IO_Module[i].bitoffDO5V));    // DO5V
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 3, &(_NRMK_IO_Module[i].offTO), &(_NRMK_IO_Module[i].bitoffTO));    // TO
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 4, &(_NRMK_IO_Module[i].offDO), &(_NRMK_IO_Module[i].bitoffDO));    // DO
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 5, &(_NRMK_IO_Module[i].offAO1), &(_NRMK_IO_Module[i].bitoffAO1));  // AO1
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 6, &(_NRMK_IO_Module[i].offAO2), &(_NRMK_IO_Module[i].bitoffAO2));  // AO2
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 7, &(_NRMK_IO_Module[i].offFTConfigParam), &(_NRMK_IO_Module[i].bitoffFTConfigParam));  // FTConfigParam
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 8, &(_NRMK_IO_Module[i].offRS485ConfigParam), &(_NRMK_IO_Module[i].bitoffRS485ConfigParam));    // RS485ConfigParam
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 9, &(_NRMK_IO_Module[i].offRS485CMD), &(_NRMK_IO_Module[i].bitoffRS485CMD));    // RS485CMD
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 10, &(_NRMK_IO_Module[i].offRS485TxCnt), &(_NRMK_IO_Module[i].bitoffRS485TxCnt));   // RS485TxCnt
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 11, &(_NRMK_IO_Module[i].offRS485TxD0), &(_NRMK_IO_Module[i].bitoffRS485TxD0)); // RS485TxD0
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 12, &(_NRMK_IO_Module[i].offRS485TxD1), &(_NRMK_IO_Module[i].bitoffRS485TxD1)); // RS485TxD1
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 13, &(_NRMK_IO_Module[i].offRS485TxD2), &(_NRMK_IO_Module[i].bitoffRS485TxD2)); // RS485TxD2
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 14, &(_NRMK_IO_Module[i].offRS485TxD3), &(_NRMK_IO_Module[i].bitoffRS485TxD3)); // RS485TxD3
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 15, &(_NRMK_IO_Module[i].offRS485TxD4), &(_NRMK_IO_Module[i].bitoffRS485TxD4)); // RS485TxD4
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 16, &(_NRMK_IO_Module[i].offRS485TxD5), &(_NRMK_IO_Module[i].bitoffRS485TxD5)); // RS485TxD5
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 17, &(_NRMK_IO_Module[i].offRS485TxD6), &(_NRMK_IO_Module[i].bitoffRS485TxD6)); // RS485TxD6
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 18, &(_NRMK_IO_Module[i].offRS485TxD7), &(_NRMK_IO_Module[i].bitoffRS485TxD7)); // RS485TxD7
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 19, &(_NRMK_IO_Module[i].offRS485TxD8), &(_NRMK_IO_Module[i].bitoffRS485TxD8)); // RS485TxD8
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x7100, 20, &(_NRMK_IO_Module[i].offRS485TxD9), &(_NRMK_IO_Module[i].bitoffRS485TxD9)); // RS485TxD9
                    
                }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
                {
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 1, &(_NRMK_Indy_Tool[i].offILed), &(_NRMK_Indy_Tool[i].bitoffILed));    // ILed
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 2, &(_NRMK_Indy_Tool[i].offIGripper), &(_NRMK_Indy_Tool[i].bitoffIGripper));    // IGripper
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 3, &(_NRMK_Indy_Tool[i].offFTConfigParam), &(_NRMK_Indy_Tool[i].bitoffFTConfigParam));  // FTConfigParam
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 4, &(_NRMK_Indy_Tool[i].offLEDMode), &(_NRMK_Indy_Tool[i].bitoffLEDMode));  // LEDMode
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 5, &(_NRMK_Indy_Tool[i].offLEDG), &(_NRMK_Indy_Tool[i].bitoffLEDG));    // LEDG
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 6, &(_NRMK_Indy_Tool[i].offLEDR), &(_NRMK_Indy_Tool[i].bitoffLEDR));    // LEDR
                    _systemVars->_registerRxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x7000, 7, &(_NRMK_Indy_Tool[i].offLEDB), &(_NRMK_Indy_Tool[i].bitoffLEDB));    // LEDB
                    
                }

    for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
                {
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6041, 0, &(_NRMK_Drive[i].offStatusword), &(_NRMK_Drive[i].bitoffStatusword));    // Statusword
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6064, 0, &(_NRMK_Drive[i].offPositionactualvalue), &(_NRMK_Drive[i].bitoffPositionactualvalue));  // Positionactualvalue
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x606c, 0, &(_NRMK_Drive[i].offVelocityactualvalue), &(_NRMK_Drive[i].bitoffVelocityactualvalue));  // Velocityactualvalue
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6077, 0, &(_NRMK_Drive[i].offTorqueactualvalue), &(_NRMK_Drive[i].bitoffTorqueactualvalue));  // Torqueactualvalue
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Drive_ProductCode, _NRMK_Drive[i].Alias, _NRMK_Drive[i].Position, 0x6061, 0, &(_NRMK_Drive[i].offModesofoperationdisplay), &(_NRMK_Drive[i].bitoffModesofoperationdisplay));  // Modesofoperationdisplay
                    
                }
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
                {
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 1, &(_NRMK_IO_Module[i].offStatusCode), &(_NRMK_IO_Module[i].bitoffStatusCode));    // StatusCode
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 2, &(_NRMK_IO_Module[i].offDI5V), &(_NRMK_IO_Module[i].bitoffDI5V));    // DI5V
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 3, &(_NRMK_IO_Module[i].offDI1), &(_NRMK_IO_Module[i].bitoffDI1));  // DI1
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 4, &(_NRMK_IO_Module[i].offDI2), &(_NRMK_IO_Module[i].bitoffDI2));  // DI2
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 5, &(_NRMK_IO_Module[i].offAI1), &(_NRMK_IO_Module[i].bitoffAI1));  // AI1
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 6, &(_NRMK_IO_Module[i].offAI2), &(_NRMK_IO_Module[i].bitoffAI2));  // AI2
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 7, &(_NRMK_IO_Module[i].offFTRawFx), &(_NRMK_IO_Module[i].bitoffFTRawFx));  // FTRawFx
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 8, &(_NRMK_IO_Module[i].offFTRawFy), &(_NRMK_IO_Module[i].bitoffFTRawFy));  // FTRawFy
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 9, &(_NRMK_IO_Module[i].offFTRawFz), &(_NRMK_IO_Module[i].bitoffFTRawFz));  // FTRawFz
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 10, &(_NRMK_IO_Module[i].offFTRawTx), &(_NRMK_IO_Module[i].bitoffFTRawTx)); // FTRawTx
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 11, &(_NRMK_IO_Module[i].offFTRawTy), &(_NRMK_IO_Module[i].bitoffFTRawTy)); // FTRawTy
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 12, &(_NRMK_IO_Module[i].offFTRawTz), &(_NRMK_IO_Module[i].bitoffFTRawTz)); // FTRawTz
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 13, &(_NRMK_IO_Module[i].offFTOverloadStatus), &(_NRMK_IO_Module[i].bitoffFTOverloadStatus));   // FTOverloadStatus
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 14, &(_NRMK_IO_Module[i].offFTErrorFlag), &(_NRMK_IO_Module[i].bitoffFTErrorFlag)); // FTErrorFlag
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 15, &(_NRMK_IO_Module[i].offRS485RxCnt), &(_NRMK_IO_Module[i].bitoffRS485RxCnt));   // RS485RxCnt
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 16, &(_NRMK_IO_Module[i].offRS485RxD0), &(_NRMK_IO_Module[i].bitoffRS485RxD0)); // RS485RxD0
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 17, &(_NRMK_IO_Module[i].offRS485RxD1), &(_NRMK_IO_Module[i].bitoffRS485RxD1)); // RS485RxD1
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 18, &(_NRMK_IO_Module[i].offRS485RxD2), &(_NRMK_IO_Module[i].bitoffRS485RxD2)); // RS485RxD2
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 19, &(_NRMK_IO_Module[i].offRS485RxD3), &(_NRMK_IO_Module[i].bitoffRS485RxD3)); // RS485RxD3
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 20, &(_NRMK_IO_Module[i].offRS485RxD4), &(_NRMK_IO_Module[i].bitoffRS485RxD4)); // RS485RxD4
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 21, &(_NRMK_IO_Module[i].offRS485RxD5), &(_NRMK_IO_Module[i].bitoffRS485RxD5)); // RS485RxD5
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 22, &(_NRMK_IO_Module[i].offRS485RxD6), &(_NRMK_IO_Module[i].bitoffRS485RxD6)); // RS485RxD6
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 23, &(_NRMK_IO_Module[i].offRS485RxD7), &(_NRMK_IO_Module[i].bitoffRS485RxD7)); // RS485RxD7
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 24, &(_NRMK_IO_Module[i].offRS485RxD8), &(_NRMK_IO_Module[i].bitoffRS485RxD8)); // RS485RxD8
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_IO_Module_ProductCode, _NRMK_IO_Module[i].Alias, _NRMK_IO_Module[i].Position, 0x6100, 25, &(_NRMK_IO_Module[i].offRS485RxD9), &(_NRMK_IO_Module[i].bitoffRS485RxD9)); // RS485RxD9
                    
                }
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
                {
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 1, &(_NRMK_Indy_Tool[i].offIStatus), &(_NRMK_Indy_Tool[i].bitoffIStatus));  // IStatus
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 2, &(_NRMK_Indy_Tool[i].offIButton), &(_NRMK_Indy_Tool[i].bitoffIButton));  // IButton
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 3, &(_NRMK_Indy_Tool[i].offFTRawFx), &(_NRMK_Indy_Tool[i].bitoffFTRawFx));  // FTRawFx
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 4, &(_NRMK_Indy_Tool[i].offFTRawFy), &(_NRMK_Indy_Tool[i].bitoffFTRawFy));  // FTRawFy
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 5, &(_NRMK_Indy_Tool[i].offFTRawFz), &(_NRMK_Indy_Tool[i].bitoffFTRawFz));  // FTRawFz
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 6, &(_NRMK_Indy_Tool[i].offFTRawTx), &(_NRMK_Indy_Tool[i].bitoffFTRawTx));  // FTRawTx
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 7, &(_NRMK_Indy_Tool[i].offFTRawTy), &(_NRMK_Indy_Tool[i].bitoffFTRawTy));  // FTRawTy
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 8, &(_NRMK_Indy_Tool[i].offFTRawTz), &(_NRMK_Indy_Tool[i].bitoffFTRawTz));  // FTRawTz
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 9, &(_NRMK_Indy_Tool[i].offFTOverloadStatus), &(_NRMK_Indy_Tool[i].bitoffFTOverloadStatus));    // FTOverloadStatus
                    _systemVars->_registerTxDomainEntry(NRMK_VendorID, NRMK_Indy_Tool_ProductCode, _NRMK_Indy_Tool[i].Alias, _NRMK_Indy_Tool[i].Position, 0x6000, 10, &(_NRMK_Indy_Tool[i].offFTErrorFlag), &(_NRMK_Indy_Tool[i].bitoffFTErrorFlag)); // FTErrorFlag
                    
                }
    
    
    // Init Domain
    if (ecrt_domain_reg_pdo_entry_list(_systemVars->_rxDomain, _systemVars->_rxDomain_regs) != 0)
    {
        printf("Failed to register RxDomain entry!\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(_systemVars->_txDomain, _systemVars->_txDomain_regs) != 0)
    {
        printf("Failed to register TxDomain entry!\n");
        return -1;
    }
    
        
    // Config DC Mode
    for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
        // 0x0700(sync0,1), 0x0300(sync0), 0x0000(none)
        ecrt_slave_config_dc(_NRMK_Drive[i].Config, 0x0000, _systemVars->_cycle_ns, 0, 0, 0);
        //ecrt_slave_config_dc(_NRMK_Drive[i].Config, 0x0300, _systemVars->_cycle_ns, 0, 0, 0);
#ifdef __CB__
    for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
        ecrt_slave_config_dc(_NRMK_IO_Module[i].Config, 0x0000, _systemVars->_cycle_ns, 0, 0, 0);
        //ecrt_slave_config_dc(_NRMK_IO_Module[i].Config, 0x0300, _systemVars->_cycle_ns, 0, 0, 0);
#endif
    for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
        ecrt_slave_config_dc(_NRMK_Indy_Tool[i].Config, 0x0000, _systemVars->_cycle_ns, 0, 0, 0);
        //ecrt_slave_config_dc(_NRMK_Indy_Tool[i].Config, 0x0300, _systemVars->_cycle_ns, 0, 0, 0);
    
#ifdef __CB__   
    int ret = ecrt_master_select_reference_clock(_systemVars->_master, _NRMK_IO_Module[0].Config);
#endif
#ifndef __CB__
    int ret = ecrt_master_select_reference_clock(_systemVars->_master, _NRMK_Drive[0].Config);
#endif
    if (ret < 0)
    {
        fprintf(stderr, "Failed to select reference clock: %s\n", strerror(-ret));
        return ret;
    }

    
    return 0;
}
#endif // _USE_LIB_