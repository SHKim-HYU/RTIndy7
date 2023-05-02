//
// Template File for Cmake Build
//

#include "JointController.h"

typedef JointController JointController6D;

POCO_BEGIN_MANIFEST(AbstractJointController<AbstractRobot6D>)
	POCO_EXPORT_CLASS(JointController6D)
POCO_END_MANIFEST
