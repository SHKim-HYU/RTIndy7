/*
 * robotIndy7.h
 */

#ifndef ROBOTINDY7_H_
#define ROBOTINDY7_H_

// #include <AbstractRobotIndy7.h>

#include <NRMKFramework/Components/AbstractRobot6D.h>

class robotIndy7 : public AbstractRobot6D
{
public:
    robotIndy7(std::string userName, std::string email, std::string serialNo)
    : AbstractRobot6D(userName, email, serialNo)
    {};
    ~robotIndy7(){};

    // Overridden for pure abstract functions in an abstract class
    void initToolProperties() {}
    bool setJointHome() {}

    // wylee
    void initHinfController(double delt)
    {}
    void resetHinfController()
    {}
    void setHinfControlGain(const JointVec &kp, const JointVec &kv, const JointVec &ki)
    {}
    //void HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd, JointVec & torque){}
    //int HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd, JointVec & torque){}
    //int HinfController(LieGroup::Vector3D const & grav, TaskPosition const & pd, TaskVelocity const & pdotd, TaskAcceleration const & pddotd, JointVec & torque){}
    int HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd)
    {}
    int HinfController(LieGroup::Vector3D const & grav, TaskPosition const & pd, TaskVelocity const & pdotd, TaskAcceleration const & pddotd)
    {}

    //wylee-admittance-control
    void resetAdmittanceTraj(double delT)
    {}
    void updateAdmittanceTraj(TaskPosition const & posDes, TaskVelocity const & velDes, TaskAcceleration const & accDes, TaskPosition & tposProxy, TaskVelocity & tvelProxy, TaskAcceleration & taccProxy, LieGroup::Wrench f_ext, LieGroup::Wrench f_ext_filtered, int mode)
    {}
    void ForwardDynController(AbstractRobot6D & robotNom, JointMat & Mn, JointMat & Cn, JointVec & Gn, TaskPosition const & posDes, TaskVelocity const & velDes, LieGroup::Vector3D const & gravDir)
    {}
    void setForwardDynControlGain(JointVec const & kp, JointVec const & kv, JointVec const & ki)
    {}
    void initForwardDynController(double delt)
    {}
    void resetForwardDynController()
    {}

    void computeFK(TaskPosition & pos, TaskVelocity & vel)
    {

    }
    void computeFK(TaskPosition & pos)
    {

    }

    //wylee
    void initTaskErr(double delt)
    {}
    void computeTaskErr(TaskPosition const & pos, TaskVelocity const & vel, TaskPosition const & posDes, TaskVelocity const & velDes, TaskVec & e, TaskVec & edot)
    {}

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
