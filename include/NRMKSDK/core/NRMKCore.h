/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

//
// Ensure that NRMKCORE_DLL is default unless NRMKCORE_STATIC is defined
//
#if defined(NRMKCORE_STATIC)

#define NRMKCore_API

#else


#if defined(_WIN32) && defined(_DLL)
	#if !defined(NRMKCORE_DLL) // && !defined(NRMKCORE_STATIC)
		#define NRMKCORE_DLL
	#endif
#endif


//
// The following block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the Foundation_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// Foundation_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
//
#if (defined(_WIN32) || defined(_WIN32_WCE)) && defined(NRMKCORE_DLL)
	#if defined(NRMKCORE_EXPORTS)
		#define NRMKCore_API __declspec(dllexport)
	#else
		#define NRMKCore_API __declspec(dllimport)	
	#endif
#endif

#if !defined(NRMKCore_API)
	#define NRMKCore_API
#endif

//
// Automatically link Foundation library.
//
#if defined(_MSC_VER)
	#if defined(NRMKCORE_DLL)
		#if defined(_DEBUG)
			#define NRMKCORE_LIB_SUFFIX "d.lib"
		#else
			#define NRMKCORE_LIB_SUFFIX ".lib"
		#endif
	#else
		#if defined(_DEBUG)
			#define NRMKCORE_LIB_SUFFIX "d.lib"
		#else
			#define NRMKCORE_LIB_SUFFIX ".lib"
		#endif
	#endif

	#if !defined(NRMKCORE_NO_AUTOMATIC_LIBS) && !defined(NRMKCORE_EXPORTS)
		#pragma comment(lib, "NRMKCore" NRMKCORE_LIB_SUFFIX)
	#endif
#endif

#endif 