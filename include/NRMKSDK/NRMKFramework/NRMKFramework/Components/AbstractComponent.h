/*
 * AbstractComponent.h
 *
 *  Created on: Aug 29, 2019
 *      Author: Thach Do
 */

#ifndef ABSTRACTCOMPONENT_H_
#define ABSTRACTCOMPONENT_H_

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <json/value.h>

namespace NRMKLeanFramework
{

class AbstractComponent
{
public:
    AbstractComponent(std::string userName, std::string email, std::string serialNo);

    Json::Value getLicenseObj() const;

private:
    Json::Value _licenseInfo;
};

}
#endif /* ABSTRACTCOMPONENT_H_ */
