/*
 * DirectVariable.h
 *
 *  Created on: 2018. 03. 14.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SHAREDDATA_DIRECTVARIABLE_H_
#define NRMKINDY_SHAREDDATA_DIRECTVARIABLE_H_

//#if __cplusplus < 201103L		//before gcc 4.9.x (step2 version is 4.8.x)
//#define _USE_POCO_REGEX
//#endif
#define _USE_POCO_REGEX

#define INDY_DIRECTVAR_REGEX "[BWILFD][0-9]{3}"

#include <new>
#include <stdexcept>
#include <string>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>

#ifdef _USE_POCO_REGEX
#include <Poco/RegularExpression.h>
#else
#include <regex>
#endif

#include <NRMKFramework/shmem/ShmemManager.hpp>
#include "SharedMemoryAddress.h"

#define INDY_DIRECT_VAR_MAX_NUM 1000

namespace NRMKIndy
{
namespace SharedData
{

struct DirectVariablesByte { uint8_t addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesWord { int16_t addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesDWord { int32_t addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesLWord { int64_t addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesFloat { float addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesDFloat { double addr[INDY_DIRECT_VAR_MAX_NUM]; };
struct DirectVariablesModbusReg { uint16_t addr[INDY_DIRECT_VAR_MAX_NUM]; };

union DirectVariableVal
{
	uint8_t byteVal;
	int16_t wordVal;
	int32_t dwordVal;
	int64_t lwordVal;
	float floatVal;
	double dfloatVal;
	double doubleVal;	//DEPRECATED -> use dfloatVal
	uint16_t modbusRegVal;
};

enum DirectVariableType : int
{
	DIRECT_VAR_TYPE_ERROR=-1,
	DIRECT_VAR_TYPE_BYTE=0,
	DIRECT_VAR_TYPE_WORD=1,
	DIRECT_VAR_TYPE_DWORD=2,
	DIRECT_VAR_TYPE_LWORD=3,
	DIRECT_VAR_TYPE_FLOAT=4,
	DIRECT_VAR_TYPE_DFLOAT=5,
	DIRECT_VAR_TYPE_MODBUS_REG=10,
};

struct DirectVariable
{
	DirectVariableVal val;
	DirectVariableType type;
	DirectVariable(const DirectVariableVal & val, DirectVariableType type)
	{
		this->val = val;
		this->type = type;
	}
};

class DirectVariableManager
{
public:
	DirectVariableManager(NRMKFramework::ShmemManager & indyShmem);
	~DirectVariableManager();

	uint8_t & byteVar(int addr);
	int16_t & wordVar(int addr);
	int32_t & dwordVar(int addr);
	int32_t & intVar(int addr);
	int64_t & lwordVar(int addr);
	int64_t & longVar(int addr);
	float & floatVar(int addr);
	double & dfloatVar(int addr);
	double & doubleVar(int addr);
	void * varPtr(const std::string & strAddr);
	DirectVariable getVar(const std::string & strAddr);
	std::string getVarAsStringValue(const std::string & strAddr);

	void setVar(DirectVariable var, int addr);
	void setVar(const std::string & strAddr, uint8_t val);
	void setVar(const std::string & strAddr, int16_t val);
	void setVar(const std::string & strAddr, int32_t val);
	void setVar(const std::string & strAddr, int64_t val);
	void setVar(const std::string & strAddr, float val);
	void setVar(const std::string & strAddr, double val);
	void setVar(const std::string & strAddr, const std::string & val);
	void setVarByPtr(const std::string & strAddr, void * valPtr);

private:
	bool _checkStrAddrValid(const std::string & strAddr);

private:
	NRMKFramework::ShmemManager & _indyShmem;
#ifdef _USE_POCO_REGEX
	Poco::RegularExpression _regexpAddr;
#else
	std::regex _regexpAddr;
#endif

	DirectVariablesByte * _byteVars;
	DirectVariablesWord * _wordVars;
	DirectVariablesDWord * _dwordVars;
	DirectVariablesLWord * _lwordVars;
	DirectVariablesFloat * _floatVars;
	DirectVariablesDFloat * _doubleVars;

public:
	static DirectVariableType checkStrAddrValid(const std::string & strAddr);
};

} /* namespace SharedData */
} /* namespace NRMKIndy */
#endif /* NRMKINDY_SHAREDDATA_DIRECTVARIABLE_H_ */
