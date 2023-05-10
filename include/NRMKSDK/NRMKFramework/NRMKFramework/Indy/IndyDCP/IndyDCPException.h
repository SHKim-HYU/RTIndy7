/*
 * IndyDCPException.h
 *
 *  Created on: 2019. 1. 11.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SERVICE_INDYDCPEXCEPTION_H_
#define NRMKINDY_SERVICE_INDYDCPEXCEPTION_H_

#include "IndyDCP.h"
#include "IndyDCPUtility.h"

#include <string>
#include <stdexcept>

namespace NRMKIndy
{
namespace Service
{
namespace DCP
{

class IndyDCPException : public std::exception
{
private:
	std::string _msg;
	int _code;

public:
	IndyDCPException(int code=0);
	IndyDCPException(const std::string & msg, int code=0);
	virtual ~IndyDCPException();
    virtual const char* what() const throw();


	template<typename... Args>
	static std::string buildErrorMessage(Args const&... args)
	{
		return buildStringFromParts(args...);
	}
};



//
// Macros for quickly declaring and implementing exception classes.
// Unfortunately, we cannot use a template here because character
// pointers (which we need for specifying the exception name)
// are not allowed as template arguments.
//
#define INDYDCP_DECLARE_EXCEPTION_CODE(CLS, BASE, CODE) \
class CLS: public BASE															\
{																				\
public:																			\
	CLS(int code = CODE);														\
	CLS(const std::string& msg, int code = CODE);								\
	~CLS();																		\
};

#define INDYDCP_DECLARE_EXCEPTION(CLS, BASE) \
INDYDCP_DECLARE_EXCEPTION_CODE(CLS, BASE, 0)

#define INDYDCP_IMPLEMENT_EXCEPTION(CLS, BASE)											\
CLS::CLS(int code): BASE(code)																	\
{																								\
}																								\
CLS::CLS(const std::string& msg, int code): BASE(msg, code)										\
{																								\
}																								\
CLS::~CLS()																						\
{																								\
}																								\

//
// Standard exception classes
//
INDYDCP_DECLARE_EXCEPTION(DomainException, IndyDCPException)
INDYDCP_DECLARE_EXCEPTION(RobotDeclareException, DomainException)

INDYDCP_DECLARE_EXCEPTION(LogicException, IndyDCPException)

INDYDCP_DECLARE_EXCEPTION(RunTimeException, IndyDCPException)
INDYDCP_DECLARE_EXCEPTION(ConnectionException, RunTimeException)
INDYDCP_DECLARE_EXCEPTION(TimeoutException, RunTimeException)


} /* namespace DCP */
} /* namespace Service */
} /* namespace NRMKIndy */

#endif /* NRMKINDY_SERVICE_INDYDCPEXCEPTION_H_ */
