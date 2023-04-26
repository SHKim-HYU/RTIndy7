/*
 * AbstractToolTask.h
 *
 *  Created on: Feb 28, 2018
 *      Author: ThachDo
 */

#ifndef ABSTRACTTOOLTASK_H_
#define ABSTRACTTOOLTASK_H_

namespace NRMKFramework
{

class AbstractToolTask
{
public:
	typedef int DataType;
	enum
	{
		MAX_TOOL_NUM = 10,
		DATA_BUFF_SIZE = 10*sizeof(DataType),
		TOTAL_BUFF_SIZE = MAX_TOOL_NUM * DATA_BUFF_SIZE
	};
public:
	AbstractToolTask(unsigned int id, void * toolShmData, void * toolShmCmd);
	virtual ~AbstractToolTask();

	bool writeToolData(DataType const * const data);
	bool readToolCommand(DataType * command);

private:
	unsigned int _id;		//FIXME tool index (0 to 9) is different with tool ID
	DataType * _toolShmData;
	DataType * _toolShmCmd;
};

} /* namespace NRMKFramework */

#endif /* ABSTRACTTOOLTASK_H_ */
