/* NRMKFoundation, Copyright 2014- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKSocketBase.h"
#include "NRMKPendantSocket.h"
//#include "Eigen/Eigen"

namespace NRMKHelper
{
	template<int _CMD_NUM, int _DATA_NUM>
	class NRMKPendantGuiSocket : public NRMKPendantSocketBase<_CMD_NUM, 0, _DATA_NUM, 0>
	{

	public:
		inline int getData(float & time, int  const * & q)
		{
			int fCount, iCount;
			float const * fdata = NULL;

			return getMessage(time, iCount, fCount, q, fdata);
		}

		inline int setData(float & time, int* q)
		{
			return sendMessage(time, _DATA_NUM, 0 , q , NULL);
		}


		inline int getCmd(int & cmd, int const * & q)
		{
			int iCount, fCount;
			float const * fdata = NULL;

			return getCommand(cmd, iCount, fCount, q, fdata);

		}

		inline int setCmd(int & cmd, int* q)
		{
			return sendCommand(cmd, _CMD_NUM, 0, q , NULL);

		}




	public:
		using NRMKPendantSocketBase<_CMD_NUM, 0, _DATA_NUM, 0>::sendMessage;
		using NRMKPendantSocketBase<_CMD_NUM, 0, _DATA_NUM, 0>::sendCommand;
		using NRMKPendantSocketBase<_CMD_NUM, 0, _DATA_NUM, 0>::getMessage;
		using NRMKPendantSocketBase<_CMD_NUM, 0, _DATA_NUM, 0>::getCommand;


		
	};

} // namespace NRMKHelper
