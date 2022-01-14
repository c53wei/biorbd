#ifndef BIORBD_RIGIDBODY_MARKERS_H
#define BIORBD_RIGIDBODY_MARKERS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
class Matrix;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedAcceleration;
class NodeSegment;

///
/// \brief Holder for the marker set
///
class BIORBD_API Markers
{
public:
    ///
    /// \brief Construct a marker set
    ///
    Markers();

    ///
    /// \brief Construct markers from another marker set
    /// \param other The other marker set
    ///
    Markers(const Markers& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Markers();

    ///
    /// \brief Deep copy of the markers
    /// \return Deep copy of the markers
    ///
    Markers DeepCopy() const;

    ///
    /// \brief Deep copy of the markers
    /// \param other The markers to copy from
    ///
    void DeepCopy(const Markers& other);

    ///
    /// \brief Add a marker to the set
    /// \param pos The position of the marker
    /// \param name The name of the marker
    /// \param parentName The name of the segment the marker is attached on
    /// \param technical If the marker is technical
    /// \param anatomical If the marker is anatomical
    /// \param axesToRemove Axes to remove while projecting the marker
    /// \param id The index of the parent segment
    ///
    void addMarker(
        const NodeSegment &pos,
        const utils::String &name,
        const utils::String &parentName,
        bool technical,
        bool anatomical,
        const utils::String& axesToRemove,
        int id = -1
    );

    ///
    /// \brief Return the marker of index idx
    /// \param idx The marker we want to return
    /// \return The marker
    ///
    const NodeSegment& marker(
        unsigned int idx) const;

    ///
    /// \brief Return the markers on a segment
    /// \param name Name of the segment
    /// \return The markers on the segment
    ///
    std::vector<NodeSegment> marker(
        const utils::String &name) const;

    ///
    /// \brief Return the names of all the markers
    /// \return The names of the markers
    ///
    std::vector<utils::String> markerNames() const;

    ///
    /// \brief Return the names of all the technical markers
    /// \return The names of the technical markers
    ///
    std::vector<utils::String> technicalMarkerNames() const;

    ///
    /// \brief Return the names of all the anatomical markers
    /// \return The names of the anatomical markers
    ///
    std::vector<utils::String> anatomicalMarkerNames() const;

    ///
    /// \brief Compute and return the position of a marker at given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param node The position of the marker in its parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The marker in the global reference frame
    ///
    NodeSegment marker(
        const GeneralizedCoordinates& Q,
        const NodeSegment& node,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Compute and return the position of the marker of index idx at given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The marker idx in the global reference frame
    ///
    NodeSegment marker(
        const GeneralizedCoordinates& Q,
        unsigned int  idx,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return a marker of index idx in the marker set
    /// \param idx The index of the marker
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return The marker of index idx
    ///
    NodeSegment marker(
        unsigned int  idx,
        bool removeAxis);

    ///
    /// \brief Return all the markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return All the markers in the global reference frame
    ///
    std::vector<NodeSegment> markers(
        const GeneralizedCoordinates &Q,
        bool removeAxis = true,
        bool updateKin = true);

    ///
    /// \brief Return all the markers in their respective parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return All the markers
    ///
    std::vector<NodeSegment> markers(
        bool removeAxis = true);

    ///
    /// \brief Return the linear velocity of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The linear velocity of a marker
    ///
    NodeSegment markerVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        unsigned int idx,
        bool removeAxis = true,
        bool updateKin = true);

    ///
    /// \brief Return the angular velocity of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The angular velocity of a marker
    ///
    NodeSegment markerAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        unsigned int idx,
        bool removeAxis = true,
        bool updateKin = true);

    ///
    /// \brief Return the linear velocity of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The linear velocity of all the markers
    ///
    std::vector<NodeSegment> markersVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the angular velocity of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The angular velocity of all the markers
    ///
    std::vector<NodeSegment> markersAngularVelocity(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the acceleration of a marker
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param idx The index of the marker in the marker set
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The acceleration of a marker
    ///
    NodeSegment markerAcceleration(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &Qddot,
        unsigned int idx,
        bool removeAxis = true,
        bool updateKin = true);


    ///
    /// \brief Return the acceleration of all the markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The acceleration of all the markers
    ///
    std::vector<NodeSegment> markerAcceleration(
        const GeneralizedCoordinates &Q,
        const GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &dQdot,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return all the technical markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return A vector of all the technical markers
    ///
    std::vector<NodeSegment> technicalMarkers(
        const GeneralizedCoordinates &Q,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the technical markers in their respective parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the technical markers in their parent reference frame
    ///
    std::vector<NodeSegment> technicalMarkers(
        bool removeAxis=true);

    ///
    /// \brief Return all the anatomical markers at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return A vector of all the anatomical markers
    ///
    std::vector<NodeSegment> anatomicalMarkers(
        const GeneralizedCoordinates &Q,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the anatomical markers in their respective parent reference frame
    /// \param removeAxis If there are axis to remove from the position variables
    /// \return A vector of all the anatomical markers in their parent reference frame
    ///
    std::vector<NodeSegment> anatomicalMarkers(
        bool removeAxis=true);

    ///
    /// \brief Return all the markers of the segment idx at a given Q in the global reference frame
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return All the markers of the segment idx
    ///
    std::vector<NodeSegment> segmentMarkers(
        const GeneralizedCoordinates &Q,
        unsigned int  idx,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the number of markers
    /// \return The number of markers
    ///
    unsigned int nbMarkers() const;

    ///
    /// \brief Return the number of markers on the segment idxSegment
    /// \param idxSegment The index of the segment
    /// \return The number of markers of segment idxSegment
    ///
    unsigned int nbMarkers(
        unsigned int idxSegment) const;

    ///
    /// \brief Return the number of technical markers
    /// \return The number of technical markers
    ///
    unsigned int nbTechnicalMarkers();

    ///
    /// \brief Return the number of technical markers on the segment idxSegment
    /// \param idxSegment The index of the segment
    /// \return The number of technical markers of segment idxSegment
    ///
    unsigned int nbTechnicalMarkers(unsigned int idxSegment);

    ///
    /// \brief Return the number of anatomical markers
    /// \return The number of anatomical markers
    ///
    unsigned int nbAnatomicalMarkers();

    ///
    /// \brief Return the jacobian of the markers
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The jacobian of the markers
    ///
    std::vector<utils::Matrix> markersJacobian(
        const GeneralizedCoordinates &Q,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the jacobian of the technical markers
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \return The jacobian of the technical markers
    ///
    std::vector<utils::Matrix> technicalMarkersJacobian(
        const GeneralizedCoordinates &Q,
        bool removeAxis=true,
        bool updateKin = true);

    ///
    /// \brief Return the jacobian of a chosen marker
    /// \param Q The generalized coordinates of the model
    /// \param parentName The name of the segment the marker lies on
    /// \param p The position of the point in the parent reference frame
    /// \param updateKin If the model should be updated
    /// \return The jacobian of the chosen marker
    ///
    utils::Matrix markersJacobian(
        const GeneralizedCoordinates &Q,
        const utils::String& parentName,
        const NodeSegment& p,
        bool updateKin);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Performs an inverse kinematics
    /// \param markers The markers to track
    /// \param Qinit The initial guess for the generalized coordinates
    /// \param Q The generalized coordinates that tracks the markers
    /// \param removeAxes If the markers should be projected on the axes
    ///
    bool inverseKinematics(
        const std::vector<NodeSegment>& markers,
        const GeneralizedCoordinates& Qinit,
        GeneralizedCoordinates &Q,
        bool removeAxes=true);
#endif

protected:
    ///
    /// \brief Compute the jacobian of the markers
    /// \param Q The generalized coordinates
    /// \param removeAxis If there are axis to remove from the position variables
    /// \param updateKin If the model should be updated
    /// \param lookForTechnical Check if only technical markers are to be computed
    /// \return The jacobian of the markers
    ///
    std::vector<utils::Matrix> markersJacobian(
        const GeneralizedCoordinates &Q,
        bool removeAxis,
        bool updateKin,
        bool lookForTechnical); // Retourne la jacobienne des markers

    std::shared_ptr<std::vector<NodeSegment>>
            m_marks; ///< The markers

};

}
}

#endif // BIORBD_RIGIDBODY_MARKERS_H
