#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContactNode.h"

#include "Utils/Error.h"
#include "Utils/SpatialVector.h"

#include "RigidBody/Joints.h"
#include "RigidBody/Segment.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContactNode::SoftContactNode() :
    rigidbody::NodeSegment(),
    m_contactPlane(std::make_shared<std::pair<utils::Vector3d, utils::Vector3d>>(utils::Vector3d(0, 0, 0), utils::Vector3d(0, 0, 1)))
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z) :
    rigidbody::NodeSegment(x, y, z),
    m_contactPlane(std::make_shared<std::pair<utils::Vector3d, utils::Vector3d>>(utils::Vector3d(0, 0, 0), utils::Vector3d(0, 0, 1)))
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Vector3d &other) :
    rigidbody::NodeSegment(other),
    m_contactPlane(std::make_shared<std::pair<utils::Vector3d, utils::Vector3d>>(utils::Vector3d(0, 0, 0), utils::Vector3d(0, 0, 1)))
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z,
        const utils::String &name,
        const utils::String &parentName,
        int parentID) :
    rigidbody::NodeSegment(x, y, z, name, parentName, true, true, "", parentID),
    m_contactPlane(std::make_shared<std::pair<utils::Vector3d, utils::Vector3d>>(utils::Vector3d(0, 0, 0), utils::Vector3d(0, 0, 1)))
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Vector3d &node,
        const utils::String &name,
        const utils::String &parentName,
        int parentID) :
    rigidbody::NodeSegment(node, name, parentName, true, true, "", parentID),
    m_contactPlane(std::make_shared<std::pair<utils::Vector3d, utils::Vector3d>>(utils::Vector3d(0, 0, 0), utils::Vector3d(0, 0, 1)))
{

}

void rigidbody::SoftContactNode::DeepCopy(
        const rigidbody::SoftContactNode &other)
{
    rigidbody::NodeSegment::DeepCopy(other);
    *m_contactPlane = *other.m_contactPlane;
}

utils::SpatialVector rigidbody::SoftContactNode::computeForceAtOrigin(
        Joints &model,
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &QDot,
        bool updateKin)
{

#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#else
    if (updateKin){
        model.UpdateKinematicsCustom(&Q, &QDot);
    }
    updateKin = false;
#endif

    unsigned int id = model.GetBodyId(parent().c_str());
    utils::Vector3d x(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, *this, updateKin));
    utils::Vector3d dx(rigidbody::NodeSegment(RigidBodyDynamics::CalcPointVelocity(model, Q, QDot, id, *this, updateKin)));
    utils::Vector3d angularVelocity(RigidBodyDynamics::CalcPointVelocity6D(model, Q, QDot, id, utils::Vector3d(0, 0, 0), updateKin).block(0, 0, 3, 1));

    utils::Vector3d force(computeForce(x, dx, angularVelocity));

    // Transport to CoM (Bour's formula)
    const utils::Vector3d& CoM(model.segment(parent()).characteristics().CoM());
    utils::Vector3d CoMinGlobal(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, id, CoM, updateKin));

    // Find the application point of the force
    utils::SpatialVector out(0., 0., 0., 0., 0., 0.);
    out.block(0, 0, 3, 1) = force.cross(- applicationPoint(x));
    out.block(3, 0, 3, 1) = force;
    return out;
}

void rigidbody::SoftContactNode::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::SOFT_CONTACT;
}
