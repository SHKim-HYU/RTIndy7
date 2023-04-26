/*
 * IndyDCPUtility.h
 *
 *  Created on: 2019. 1. 11.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SERVICE_INDYDCPUtility_H_
#define NRMKINDY_SERVICE_INDYDCPUtility_H_
#pragma once

#if defined (__CYGWIN__)
#ifndef WINDOWS_CYGWIN
#define WINDOWS_CYGWIN
#endif
#elif defined (_WIN32) || defined (_WIN32) || defined (WIN64) || defined (_WIN64)
#ifndef WINDOWS
#define WINDOWS
#endif
#elif defined (__linux) || defined (__linux__) || defined (linux)
#ifndef LINUX
#define LINUX
#endif
#elif defined (__unix) || defined (__unix__)
#ifndef UNIX
#define UNIX
#endif
#endif //defined _WIN32 ...
#if defined (__posix) || defined (_POSIX_VERSION)
#ifndef POSIX
#define POSIX
#endif
#endif	//__posix


#include <sstream>
#include <utility>
#include <cstddef>

#if defined (WINDOWS)
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <Windows.h>
#include <WinSock2.h>
#pragma comment(lib,"ws2_32")	//Must Link Winsock Library "ws2_32.dll"
#include <vector>
#else
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#endif	//WINDOWS

#include <string.h>


//STRING FUNCTIONS SECURE for WINDOWS COMPABILITY
#if defined (WINDOWS)
#define STRS_STRNCPY(DEST, SRC, SIZE) strncpy_s(DEST, SRC, SIZE);
#define STRS_STRCPY(DEST, SRC) strcpy_s(DEST, SRC);
#else
#define STRS_STRNCPY(DEST, SRC, SIZE) strncpy(DEST, SRC, SIZE);
#define STRS_STRCPY(DEST, SRC) strcpy(DEST, SRC);
#endif


namespace NRMKIndy
{
namespace Service
{
namespace DCP
{

template<typename... Args>
int print(std::ostream& s, Args&... args)
{
	using Expander = int[];
	return Expander{ 0, ((s << std::forward<Args>(args)), 0)...}[0];
}

template<typename... Args>
std::string buildStringFromParts(Args const&... args)
{
	std::stringstream msg;
	print(msg, args...);
	return msg.str();
}

#if defined (WINDOWS)
inline std::string toStringWCHAR(const WCHAR * str)
{
#ifdef UNICODE
	char buffer[4096] = { '\0' };
	int size = WideCharToMultiByte(CP_UTF8, 0, str, -1, NULL, 0, NULL, NULL);
	if (size > 0) {
		WideCharToMultiByte(CP_UTF8, 0, str, -1, buffer, 4095, NULL, NULL);
	}
	else {
		// Error handling
	}
	return std::string(buffer);
#else
	return std::string(str);
#endif	//UNICODE
}
#endif	// defined (WINDOWS)

template <typename T>
inline std::string toString(T value) {
#if 0 //defined (WINDOWS)
	if (std::is_same<T, const WCHAR *>::value || std::is_same<T, WCHAR *>::value)
	{
		return toStringWCHAR(reinterpret_cast<const WCHAR *>value);
	}
#endif
#if !defined(WINDOWS_CYGWIN) && !defined(WINDOWS)
	return std::to_string(value);
#else
    std::stringstream ss;
    ss << value;
    return ss.str();
#endif
}

inline void uSleep(unsigned long useconds)
{
#if defined (WINDOWS)
#if 0		//CAUTION: Busy Waiting!!! - use WINAPU
	LARGE_INTEGER perfCnt, start, now;

	QueryPerformanceFrequency((LARGE_INTEGER *)&perfCnt);
	QueryPerformanceCounter((LARGE_INTEGER *)&start);

	do {
		QueryPerformanceCounter((LARGE_INTEGER *)&now);
	} while ((now.QuadPart - start.QuadPart) / float(perfCnt.QuadPart) * 1000 * 1000 < useconds);
#else		//Spinlock Waiting - Use WIN32API
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * (__int64)useconds);

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
#endif

#elif defined (WINDOWS_CYGWIN)
	struct timeval tv;
	tv.tv_sec = (useconds / 1000000/*1e6*/);
	tv.tv_usec = (useconds % 1000000);

	select(0, NULL, NULL, NULL, &tv);

#else
	::usleep(useconds);
#endif
}

#if defined (WINDOWS)
inline std::string WSAGetLastErrorString(int err)
{
	TCHAR* message = nullptr;
	FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER,
		nullptr,
		err,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(TCHAR *)&message,
		0,
		nullptr);
	return toStringWCHAR(message);
}

inline const char * WSAGetLastErrorCString(int err)
{
	static char tempLastMsg[4096];
	std::string msg = WSAGetLastErrorString(err);
	STRS_STRNCPY(tempLastMsg, msg.c_str(), 4095);
	return tempLastMsg;
}

inline void initWinSock()
{
	WORD		wVersionRequested;
	WSADATA		wsaData;

	wVersionRequested = MAKEWORD(2, 2);
	int err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		int lastError = WSAGetLastError();
		WSACleanup();
		throw std::exception(std::string(std::string("WSAStartup Failed: ") + WSAGetLastErrorString(lastError)).c_str());
	}
}

inline void cleanupWinSock()
{
	WSACleanup();
}
#endif	//WINDOWS

inline std::string toErrString(int error)
{
#if defined (WINDOWS)
	return WSAGetLastErrorString(error);
#else
	return std::string(::strerror(error));
#endif
}

} /* namespace DCP */
} /* namespace Service */
} /* namespace NRMKIndy */



#endif /* NRMKINDY_SERVICE_INDYDCPUtility_H_ */
