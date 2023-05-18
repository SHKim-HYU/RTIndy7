/*
 * robotIndy7.h
 */

#ifndef ROBOTINDY7_H_
#define ROBOTINDY7_H_

#include <AbstractRobotIndy7.h>

class robotIndy7 : public AbstractRobotIndy7
{
public:
    robotIndy7(): AbstractRobotIndy7()
    {};
    ~robotIndy7(){};

    // Overridden for pual functions in an abstract class
    void initToolProperties() {}
    bool setJointHome() {}

    void computeFK(TaskPosition & pos, TaskVelocity & vel)
    {

    }
    void computeFK(TaskPosition & pos)
    {

    }

    void computeJacobian(TaskPosition & pos, TaskVelocity & vel, TaskJacobian & J, TaskJacobian & Jdot)
    {

    }

    void computeDynamicsParams(LieGroup::Vector3D const & gravDir, JointMat & M, JointMat & C, JointVec & G)
    {

    }

    int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, TaskVec & velRef, TaskVec & accRef, JointVec & qdotRef, JointVec & qddotRef)
    {

    }

    int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, TaskVec & accRef, JointVec & qddotRef)
    {

    }

    int computeJointVel(TaskJacobian const & J, TaskVec & velRef, JointVec & qdotRef)
    {

    }

private:

};

#endif /* ROBOTINDY7_H_ */
