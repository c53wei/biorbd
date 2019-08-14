#ifndef S2M_IMUS_H
#define S2M_IMUS_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/Attitude.h"
#include "Utils/String.h"
#include "Utils/Node.h"
#include "s2mPatch.h"

class s2mJoints;
class s2mIMU;
namespace biorbd { namespace utils {
class Matrix;
class GenCoord;
}}
class BIORBD_API s2mIMUs
{
public:
    s2mIMUs();
    virtual ~s2mIMUs();

    // Set and get
    void addIMU(
            const biorbd::utils::Attitude &pos = biorbd::utils::Attitude(),
            const biorbd::utils::String &name = "",
            const biorbd::utils::String &parentName = "",
            const bool &technical = true,
            const bool &anatomical = false,
            const int &id = -1); // Ajouter un nouveau marker
    unsigned int nIMUs() const; // Retourne le nombre de marqueurs

    std::vector<biorbd::utils::String> IMUsNames();
    std::vector<biorbd::utils::String> technicalIMUsNames();
    std::vector<biorbd::utils::String> anatomicalIMUsNames();

    std::vector<s2mIMU> IMU(); // Retour d'un STL vector de tous les IMU
    std::vector<s2mIMU> IMU(s2mJoints& m, unsigned int idx); // Retour d'un STL vector de tous les IMU d'un segment idx
    const s2mIMU& IMU(const unsigned int&); // Retour d'un IMU ind idx

    std::vector<s2mIMU> IMU(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const bool &updateKin = true); // Retour d'un STL vector de tous les IMUs
    s2mIMU IMU(
            s2mJoints& model,
            const biorbd::utils::GenCoord&,
            const unsigned int& idx,
            const bool &updateKin = true); // Retour d'un IMU ind idx
    std::vector<s2mIMU> segmentIMU(
            s2mJoints& model,
            const biorbd::utils::GenCoord&,
            const unsigned int& idx,
            const bool &updateKin = true); // Retour d'un STL vector de tous les IMUs d'un segment


    unsigned int nTechIMUs(); // Retourne le nombre de marqueurs techniques
    unsigned int nAnatIMUs(); // Retourne le nombre de marqueurs anatomiques
    std::vector<s2mIMU> technicalIMU(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<s2mIMU> technicalIMU(); // Retour d'un STL vector de tous les IMUs
    std::vector<s2mIMU> anatomicalIMU(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            bool updateKin = true); // Retour d'un STL vector de tous les IMUs
    std::vector<s2mIMU> anatomicalIMU(); // Retour d'un STL vector de tous les IMUs




    std::vector<biorbd::utils::Matrix> IMUJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const bool &updateKin = true); // Retourne la jacobienne des Tags
    std::vector<biorbd::utils::Matrix> TechnicalIMUJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const bool &updateKin = true); // Retourne la jacobienne des Tags pour les marqueurs techniques

protected:
    std::vector<biorbd::utils::Matrix> IMUJacobian(
            s2mJoints& model,
            const biorbd::utils::GenCoord &Q,
            const bool &updateKin,
            bool lookForTechnical); // Retourne la jacobienne des Tags

    std::vector <s2mIMU> m_IMUs;

};

#endif // S2M_IMUS_H
