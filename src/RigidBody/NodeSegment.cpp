#define BIORBD_API_EXPORTS
#include "RigidBody/NodeSegment.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

rigidbody::NodeSegment::NodeSegment() :
    utils::Vector3d(0, 0, 0),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(const utils::Vector3d
        &other) :
    utils::Vector3d(other),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    const utils::String &axesToRemove,
    int parentID) :
    utils::Vector3d(x, y, z, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Vector3d &node,
    const utils::String &name,
    const utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    const utils::String& axesToRemove, // Axis to remove
    int parentID) :
    utils::Vector3d(node, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
    //ctor
}

rigidbody::NodeSegment::NodeSegment(
        const utils::Vector3d &node,
        const utils::String &name,
        const utils::String &parentName,
        bool isTechnical,
        bool isAnatomical,
        const std::vector<bool> &axesToRemove,
        int parentID) :
    utils::Vector3d(node, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(axesToRemove)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{

}

rigidbody::NodeSegment rigidbody::NodeSegment::DeepCopy() const
{
    rigidbody::NodeSegment copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::NodeSegment::DeepCopy(const
        rigidbody::NodeSegment&other)
{
    utils::Node::DeepCopy(other);
    *m_axesRemoved = *other.m_axesRemoved;
    *m_nbAxesToRemove = *other.m_nbAxesToRemove;
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
    *m_id = *other.m_id;
}


bool rigidbody::NodeSegment::isAnatomical() const
{
    return *m_anatomical;
}



bool rigidbody::NodeSegment::isTechnical() const
{
    return *m_technical;
}


int rigidbody::NodeSegment::parentId() const
{
    return *m_id;
}

const std::vector<bool> &rigidbody::NodeSegment::axes() const
{
    return *m_axesRemoved;
}

rigidbody::NodeSegment rigidbody::NodeSegment::removeAxes()
const
{
    rigidbody::NodeSegment pos(*this);
    for (unsigned int i=0; i<m_axesRemoved->size(); ++i)
        if (isAxisRemoved(i)) {
            pos(i) = 0;
        }
    return pos;
}

bool rigidbody::NodeSegment::isAxisRemoved(unsigned int i) const
{
    return (*m_axesRemoved)[i];
}

bool rigidbody::NodeSegment::isAxisKept(unsigned int i) const
{
    return !isAxisRemoved(i);
}

int rigidbody::NodeSegment::nbAxesToRemove() const
{
    return *m_nbAxesToRemove;
}

void rigidbody::NodeSegment::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::BONE_POINT;
}

void rigidbody::NodeSegment::addAxesToRemove(unsigned int axisNumber)
{
    if (axisNumber>2) {
        utils::Error::raise("Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    }
    (*m_axesRemoved)[axisNumber] = true;
    ++*m_nbAxesToRemove;
}

void rigidbody::NodeSegment::addAxesToRemove(const
        utils::String& s)
{
    for (unsigned int i=0; i<s.length(); ++i)
        if (!s(i).compare("x")) {
            addAxesToRemove(0);
        } else if (!s(i).compare("y")) {
            addAxesToRemove(1);
        } else if (!s(i).compare("z")) {
            addAxesToRemove(2);
        } else {
            utils::Error::raise("Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
        }
}

void rigidbody::NodeSegment::addAxesToRemove(const
        std::vector<unsigned int>& axes)
{
    for (unsigned int i=0; i<axes.size(); ++i) {
        addAxesToRemove(axes[i]);
    }
}

void rigidbody::NodeSegment::addAxesToRemove(const
        std::vector<utils::String>& axes)
{
    for (unsigned int i=0; i<axes.size(); ++i) {
        addAxesToRemove(axes[i]);
    }
}

utils::String rigidbody::NodeSegment::axesToRemove()
{
    utils::String axes;
    if (isAxisRemoved(0)) {
        axes += "x";
    }
    if (isAxisRemoved(1)) {
        axes += "y";
    }
    if (isAxisRemoved(2)) {
        axes += "z";
    }
    return axes;
}
