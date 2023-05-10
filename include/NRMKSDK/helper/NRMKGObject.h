/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "LieGroup/LieGroup.h"

namespace NRMKHelper 
{

// Graphics object
enum NRMK_GOBJ_TYPE
{	
	NRMK_GOBJ_TYPE_ARROW = 0, 	
	NRMK_GOBJ_TYPE_FRAME = 1,
	NRMK_GOBJ_TYPE_PLANE = 2,
	NRMK_GOBJ_TYPE_POINT = 3,
	NRMK_GOBJ_TYPE_LINE = 4,
	NRMK_GOBJ_TYPE_VECTORFIELD = 5,

// 	NRMK_GOBJ_TYPE_ARC_ARROW = 11,
// 	NRMK_GOBJ_TYPE_CIRCLE = 12,
// 	NRMK_GOBJ_TYPE_ELLIPSE = 14,
// 	NRMK_GOBJ_TYPE_TRIANGLE = 15,
// 	NRMK_GOBJ_TYPE_RECTANGLE = 16,
// 
// 	NRMK_GOBJ_TYPE_SPHERE = 21,
// 	NRMK_GOBJ_TYPE_CUBE = 22,
// 	NRMK_GOBJ_TYPE_ELLIPSOID = 23,
// 	NRMK_GOBJ_TYPE_BOX = 24,

	NRMK_GOBJ_TYPE_NONE = 9999, 
};

class NRMKGObject
{
public:
	enum
	{
		PACKET_HEADER_BYTE = 3*sizeof(int) + 1*sizeof(unsigned char),
	};

	// X | X | X | X | SET_TRACE | SET_SIZE | SET_COLOR | SET_VISIBLE 
	enum 
	{
		CMD_SET_VISIBLE	= 0x01,
		CMD_SET_COLOR	= 0x02,
		CMD_SET_SIZE	= 0x04,
		CMD_SET_TRACE	= 0x08,
	};

public:
	inline NRMKGObject(NRMK_GOBJ_TYPE type = NRMK_GOBJ_TYPE_NONE) 
		: _type( type), _bid(-1), _gid(_GenerateID()), _command(CMD_SET_VISIBLE)
	{
	}

	void setId(int gid) 
	{
		_gid = gid;
	}

	void belong(int bid)
	{
		_bid = bid;
	}

	void resetCommand()
	{
		_command = 0x00;
	}

	void setCommand(unsigned char command)
	{
		_command = command;
	}

	void addCommand(unsigned char command)
	{
		_command |= command;
	}

protected:
	inline int _makeSocketDataHeader(unsigned char * const socket_data) const
	{
		int offset = 0;
				
		memcpy(socket_data, &_type, sizeof(int));
		offset += sizeof(int);

		memcpy(socket_data + offset, &_command, sizeof(unsigned char));
		offset += sizeof(unsigned char);

		memcpy(socket_data + offset, &_bid, sizeof(int));
		offset += sizeof(int);

		memcpy(socket_data + offset, &_gid, sizeof(int));
		offset += sizeof(int);

		return offset;
	}

private:
	int _GenerateID() 
	{
		static int id = 0;

		return id++;
	}

protected:
	int _type;
	int _bid;
	int _gid;
	unsigned char _command;
};

template <typename ObjectType, int DATA_SIZE>
class NRMKGObjectBase : public NRMKGObject
{
public:
	/** \returns a reference to the derived object */
	inline ObjectType& derived() { return *static_cast<ObjectType*>(this); }
	/** \returns a const reference to the derived object */
	inline const ObjectType& derived() const { return *static_cast<const ObjectType*>(this); }

	NRMKGObjectBase(NRMK_GOBJ_TYPE type) : NRMKGObject(type) { }

	int makeSocketData(unsigned char * const socket_data) const
	{
		return derived().makeSocketData(socket_data);
	}

	void setData(float const * const data, int dim = DATA_SIZE, int begin = 0)
	{
		memcpy(_data + begin, data, dim*sizeof(float));
	}

	void setDataAt(float const * const data, int dim = DATA_SIZE, int begin = 0)
	{
		setData(data, dim, begin);
	}

protected:
	float _data[DATA_SIZE];
};

template <typename ObjectType>
class NRMKGObjectBase<ObjectType, 0> : public NRMKGObject
{
public:
	/** \returns a reference to the derived object */
	inline ObjectType& derived() { return *static_cast<ObjectType*>(this); }
	/** \returns a const reference to the derived object */
	inline const ObjectType& derived() const { return *static_cast<const ObjectType*>(this); }

	NRMKGObjectBase(NRMK_GOBJ_TYPE type) : NRMKGObject(type) { }

	int makeSocketData(unsigned char * const socket_data) const
	{
		return derived().makeSocketData(socket_data);
	}

	void setData(float const * const data, int dim = 0, int begin = 0)
	{
	}
};

class NRMKGObjectArrow : public NRMKGObjectBase<NRMKGObjectArrow, 10>
{
public:
	// X | X | SET_BEGIN_POINT | SET_END_POINT | X | SET_SIZE | SET_COLOR | SET_VISIBLE 
	enum 
	{
		CMD_SET_END_POINT = 0x10,
		CMD_SET_BEGIN_POINT = 0x20,
	};

	enum
	{
		MAX_PACKET_BYTE = 10*sizeof(float),
	};

public:
	inline NRMKGObjectArrow() 
		: NRMKGObjectBase<NRMKGObjectArrow, 10>(NRMK_GOBJ_TYPE_ARROW)
		, _scale(1), _hasScale(false)
	{
	}	

	int makeSocketData(unsigned char * const socket_data) const 
	{
		int offset = _makeSocketDataHeader(socket_data);
		int cur = 0;

		if (_command & CMD_SET_BEGIN_POINT)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_END_POINT)
		{
			// FIXME @20130901 It does not work when CMD_SET_BEGIN_POINT is defined
			if (_hasScale)
			{
				float arrow[3] = { _data[cur]*_scale, _data[cur + 1]*_scale, _data[cur + 2]*_scale };
				memcpy(socket_data + offset, arrow, 3*sizeof(float));
			}
			else
			{
				memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			}

			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_SIZE)
		{
			memcpy(socket_data + offset, _data + cur, sizeof(float));
			cur += 1;
			offset += sizeof(float);
		}

		if (_command & CMD_SET_COLOR)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		return offset;
	}

	void setScale(float scale)
	{
		_scale = scale;
		_hasScale = true;
	}

// 	void setBegin(float const * const begin)
// 	{
// 		memcpy(_begin, begin, 3*sizeof(float));
// 		_hasBegin = true;
// 	}

private:
	using NRMKGObjectBase<NRMKGObjectArrow, 10>::_data;
	float _scale;
	bool _hasScale;

	float _begin[3];
	bool _hasBegin;
};

class NRMKGObjectFrame : public  NRMKGObjectBase<NRMKGObjectFrame, 14>
{
public:
	// X | X | SET_ORIENTATION | SET_ORIGIN | X | SET_SIZE | SET_COLOR (X) | SET_VISIBLE 
	enum 
	{
		CMD_SET_ORIGIN = 0x10,
		CMD_SET_ORIENTATION = 0x20,
	};

	enum
	{
		MAX_PACKET_BYTE = 14*sizeof(float),
	};

public:
	inline NRMKGObjectFrame() 
		: NRMKGObjectBase<NRMKGObjectFrame, 14>(NRMK_GOBJ_TYPE_FRAME)
		, _hasOffset(false) 
	{

	}	

	inline int makeSocketData(unsigned char * const socket_data) const 
	{
		int offset = _makeSocketDataHeader(socket_data);
		int cur = 0;

		if (_command & CMD_SET_ORIENTATION)
		{
			memcpy(socket_data + offset, _data + cur, 9*sizeof(float));
			cur += 9;
			offset += 9*sizeof(float);
		}

		if (_command & CMD_SET_ORIGIN)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_SIZE)
		{
			memcpy(socket_data + offset, _data + cur, 2*sizeof(float));
			cur += 2;
			offset += 2*sizeof(float);
		}

		return offset;
	}

	inline void makeTransformData(LieGroup::HTransform const & T, float * const socket_data) const
	{
		double const * data;
		if (_hasOffset)
		{
			LieGroup::HTransform T_ = _Toffset.cascade(T);
			data = T_.data();
		}
		else
		{
			data = T.data();
		}

		for (int dest = 0, src = 0; dest < 12; src++)
		{
			socket_data[dest++] = (float) data[src++];
			socket_data[dest++] = (float) data[src++];
			socket_data[dest++] = (float) data[src++];
		}	
	}

	inline void makeRotationData(LieGroup::Rotation const & R, float * const socket_data) const
	{
		double const * data;
		if (_hasOffset)
		{
			LieGroup::Rotation R_ = _Toffset.R()*R;
			data = R_.data();
		}
		else
		{
			data = R.data();
		}

		for (int dest = 0, src = 0; dest < 9; )
		{
			socket_data[dest++] = (float) data[src++];
			socket_data[dest++] = (float) data[src++];
			socket_data[dest++] = (float) data[src++];
		}	
	}

	inline void makePositionData(LieGroup::Displacement const & r, float * const socket_data) const
	{
		double const * data;
		if (_hasOffset)
		{
			LieGroup::Displacement r_ = _Toffset.transform(r);
			data = r_.data();
		}
		else
		{
			data = r.data();
		}
		
		socket_data[0] = (float) data[0];
		socket_data[1] = (float) data[1];
		socket_data[2] = (float) data[2];
	}

	inline void setOffset(LieGroup::HTransform const & T)
	{
		_Toffset = T;
		_hasOffset = true;
	}

private:
	using NRMKGObjectBase<NRMKGObjectFrame, 14>::_data;
	// float _data[14];	// in the order of orientation (9) - origin (3) - length (1) - radius(1)

	LieGroup::HTransform _Toffset;
	bool _hasOffset;
};

class NRMKGObjectPoint : public NRMKGObjectBase<NRMKGObjectPoint, 6>
{
public:
	// X | X | X | SET_POSITION | X | SET_SIZE (X) | SET_COLOR | SET_VISIBLE 
	enum 
	{
		CMD_SET_POSITION = 0x10,
	};

	enum
	{
		MAX_PACKET_BYTE = 6*sizeof(float),
	};

public:
	inline NRMKGObjectPoint() : NRMKGObjectBase<NRMKGObjectPoint, 6>(NRMK_GOBJ_TYPE_POINT) { }	

	int makeSocketData(unsigned char * const socket_data) const 
	{
		int offset = _makeSocketDataHeader(socket_data);
		int cur = 0;

		if (_command & CMD_SET_POSITION)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_COLOR)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		return offset;
	}

private:
	using NRMKGObjectBase<NRMKGObjectPoint, 6>::_data;
};

class NRMKGObjectVectorField : public NRMKGObjectBase<NRMKGObjectVectorField, 6>
{
public:
	// X | X | X | SET_END_POINT | X | SET_SIZE (X) | SET_COLOR | SET_VISIBLE 
	enum 
	{
		CMD_SET_END_POINT = 0x10,
	};

	enum
	{
		MAX_PACKET_BYTE = 6*sizeof(float),
	};

public:
	inline NRMKGObjectVectorField() 
		: NRMKGObjectBase<NRMKGObjectVectorField, 6>(NRMK_GOBJ_TYPE_VECTORFIELD)
		, _scale(1), _hasScale(false)
	{
	}	

	int makeSocketData(unsigned char * const socket_data) const 
	{
		int offset = _makeSocketDataHeader(socket_data);
		int cur = 0;

		if (_command & CMD_SET_END_POINT)
		{
			if (_hasScale)
			{
				float arrow[3] = { _data[cur]*_scale, _data[cur + 1]*_scale, _data[cur + 2]*_scale };
				memcpy(socket_data + offset, arrow, 3*sizeof(float));
			}
			else
			{
				memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			}

			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_COLOR)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		return offset;
	}

	void setScale(float scale)
	{
		_scale = scale;
		_hasScale = true;
	}

private:
	using NRMKGObjectBase<NRMKGObjectVectorField, 6>::_data;
	float _scale;
	bool _hasScale;
};


class NRMKGObjectLine : public NRMKGObjectBase<NRMKGObjectLine, 9>
{
public:
	// X | X | SET_BEGIN_POINT | SET_END_POINT | X | X | SET_COLOR | SET_VISIBLE 
	enum 
	{
		CMD_SET_END_POINT = 0x10,
		CMD_SET_BEGIN_POINT = 0x20,
	};

	enum
	{
		MAX_PACKET_BYTE = 9*sizeof(float),
	};

public:
	inline NRMKGObjectLine() 
		: NRMKGObjectBase<NRMKGObjectLine, 9>(NRMK_GOBJ_TYPE_LINE)
	{
	}	

	int makeSocketData(unsigned char * const socket_data) const 
	{
		int offset = _makeSocketDataHeader(socket_data);
		int cur = 0;

		if (_command & CMD_SET_BEGIN_POINT)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_END_POINT)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));

			cur += 3;
			offset += 3*sizeof(float);
		}

		if (_command & CMD_SET_COLOR)
		{
			memcpy(socket_data + offset, _data + cur, 3*sizeof(float));
			cur += 3;
			offset += 3*sizeof(float);
		}

		return offset;
	}

private:
	using NRMKGObjectBase<NRMKGObjectLine, 9>::_data;
};


} // namespace NRMKHelper