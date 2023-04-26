/*
 * Version.h
 *
 *  Created on: 2018. 9. 20.
 *      Author: Hanter Jung
 */

#ifndef NRMKFRAMEWORK_NRMKHELPER_VERSION_H_
#define NRMKFRAMEWORK_NRMKHELPER_VERSION_H_

#pragma once

#include <stdlib.h>
#include <string.h>

#include <string>
#include <iostream>
#include <sstream>

namespace NRMKHelper
{

struct Version
{
public:
	Version(const std::string & versionStr)
	: versionMain {0, 0, 0}
	, versionSupple ("")
	{
		std::string versionStrMain;

		//parse versionStr format
		size_t hyphenPos = versionStr.find_first_of('-');
		if (hyphenPos == std::string::npos)	//no supplement version
		{
			versionStrMain = versionStr;
		}
		else
		{
			versionStrMain = versionStr.substr(0, hyphenPos);
			versionSupple = versionStr.substr(hyphenPos+1);
		}

		std::istringstream is(versionStrMain);
		int versionN = 0;
		std::string versionPart;
		while (getline(is, versionPart, '.'))
		{
			versionMain[versionN++] = atoi(versionPart.c_str());
			if (versionN == 3) break;
		}
	}

	Version(const char * versionStr)
	: Version(std::string(versionStr))
	{}

	Version(int major=0, int minor=0, int patch=0, const std::string & supplement = "")
//	: Version (std::string().append(std::to_string(major)).append(".")
//							.append(std::to_string(minor)).append(".")
//							.append(std::to_string(patch))
//							.append(supplement.empty()? "" : std::string("-").append(supplement)) )
	: versionMain {major, minor, patch}
	, versionSupple (supplement)
	{
	}

	Version(Version const & other)
	{
		memcpy(versionMain, other.versionMain, sizeof(int)*3);
		versionSupple = other.versionSupple;
	}

	void set(const std::string & versionStr)
	{
		Version version (versionStr);
		*this = version;
	}

	void set(int major=0, int minor=0, int patch=0, const std::string & supplement = "")
	{
		Version version(major, minor, patch, supplement);
		*this = version;
	}

	Version & operator =(const Version & other)
	{
		memcpy(versionMain, other.versionMain, sizeof(int)*3);
		versionSupple = other.versionSupple;

		return (*this);
	}

	Version & operator =(const std::string & versionString)
	{
		Version version (versionString);
		*this = version;
		return (*this);
	}

	Version & operator =(const char * versionString)
	{
		Version version (versionString);
		*this = version;
		return (*this);
	}

	//compare operator functions don't compare supplement version
	bool operator ==(const Version& other) const
	{
		if ((versionMain[0]==other.versionMain[0]) && (versionMain[1]==other.versionMain[1]) &&
				(versionMain[2]==other.versionMain[2])) return true;
		else return false;
	}

	bool operator >(const Version& other) const
	{
		if (versionMain[0] > other.versionMain[0]) return true;
		else if (versionMain[0] == other.versionMain[0]) {
			if (versionMain[1] > other.versionMain[1]) return true;
			else if (versionMain[1] == other.versionMain[1]) {
				if (versionMain[2] > other.versionMain[2]) return true;
			}
		}
		return false;
	}

	bool operator <(const Version& other) const
	{
		const Version & thisObj = (*this);
		//return other.operator >(thisObj);
		return other > thisObj;
	}

	bool operator >=(const Version& other) const
	{
		return (operator >(other) || operator ==(other));
	}

	bool operator <=(const Version& other) const
	{
		return (operator <(other) || operator ==(other));
	}

	int getVersion(int idxNum) const
	{
		if (idxNum >= 0 && idxNum < 3)
			return versionMain[idxNum];
		else return -2;
	}

	const std::string getVersionSupple() const
	{
		return versionSupple;
	}

	std::string toString()
	{
		std::string versionStr;

		versionStr.append(std::to_string(versionMain[0])).append(".")
				.append(std::to_string(versionMain[1])).append(".")
				.append(std::to_string(versionMain[2]));
		if (!versionSupple.empty())
			versionStr.append("-").append(versionSupple);

		return versionStr;
	}

private:
	//std::string versionStr;	//version string must be "x.x.x-s" style. ex) "2.1.5-rc.2"

	int versionMain[3];
	std::string versionSupple;	//not use to compare version
};


}


#endif /* NRMKFRAMEWORK_NRMKHELPER_VERSION_H_ */
