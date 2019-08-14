#define BIORBD_API_EXPORTS
#include "Muscles/Caracteristics.h"

biorbd::muscles::Caracteristics::Caracteristics(
        const double &optLength,
        const double &fmax,
        const double &PCSA,
        const double &tendonSlackLength,
        const double &pennAngle,
        const biorbd::muscles::State &stateMax,
        const biorbd::muscles::FatigueParameters &fatigueParameters,
        const double tauAct,
        const double tauDeact,
        const double &minAct):
    m_optimalLength(optLength),
    m_fIsoMax(fmax),
    m_PCSA(PCSA),
    m_tendonSlackLength(tendonSlackLength),
    m_pennationAngle(pennAngle),
    m_stateMax(stateMax),
    m_minActivation(minAct),
    m_tauActivation(tauAct),
    m_tauDeactivation(tauDeact),
    m_fatigueParameters(fatigueParameters)
{

}

// Get et Set
double biorbd::muscles::Caracteristics::optimalLength() const { return m_optimalLength; }
double biorbd::muscles::Caracteristics::forceIsoMax() const { return m_fIsoMax; }
double biorbd::muscles::Caracteristics::tendonSlackLength() const {return m_tendonSlackLength;}
double biorbd::muscles::Caracteristics::pennationAngle() const {return m_pennationAngle;}
double biorbd::muscles::Caracteristics::PCSA() const {return m_PCSA;}

void biorbd::muscles::Caracteristics::minActivation(double val){ m_minActivation = val;}
double biorbd::muscles::Caracteristics::minActivation() const { return m_minActivation;}
void biorbd::muscles::Caracteristics::tauActivation(double val){ m_tauActivation = val;}
double biorbd::muscles::Caracteristics::tauActivation() const { return m_tauActivation;}
void biorbd::muscles::Caracteristics::tauDeactivation(double val){ m_tauDeactivation = val;}
double biorbd::muscles::Caracteristics::tauDeactivation() const { return m_tauDeactivation;}


void biorbd::muscles::Caracteristics::setOptimalLength(const double &val) { m_optimalLength = val; }
void biorbd::muscles::Caracteristics::setForceIsoMax(const double &val) { m_fIsoMax = val; }
void biorbd::muscles::Caracteristics::PCSA(const double &val) {m_PCSA = val;}
void biorbd::muscles::Caracteristics::setTendonSlackLength(const double &val) {m_tendonSlackLength = val;}
void biorbd::muscles::Caracteristics::setPennationAngle(const double &val) {m_pennationAngle = val;}



biorbd::muscles::Caracteristics::Caracteristics(const biorbd::muscles::Caracteristics& c):
    m_optimalLength(c.m_optimalLength),
    m_fIsoMax(c.m_fIsoMax),
    m_PCSA(c.m_PCSA),
    m_tendonSlackLength(c.tendonSlackLength()),
    m_pennationAngle(c.m_pennationAngle),
    m_stateMax(c.m_stateMax),
    m_minActivation(c.m_minActivation),
    m_tauActivation(c.m_tauActivation),
    m_tauDeactivation(c.m_tauDeactivation),
    m_fatigueParameters(c.m_fatigueParameters)
{

}

biorbd::muscles::Caracteristics& biorbd::muscles::Caracteristics::operator=(const biorbd::muscles::Caracteristics& c){
    if (this==&c) // check for self-assigment
        return *this;

    m_optimalLength = c.m_optimalLength;
    m_fIsoMax = c.m_fIsoMax;
    m_PCSA = c.m_PCSA;
    m_tendonSlackLength = c.m_tendonSlackLength;
    m_pennationAngle = c.m_pennationAngle;
    setStateMax(c.m_stateMax);
    m_minActivation = c.m_minActivation;
    m_tauActivation = c.m_tauActivation;
    m_tauDeactivation = c.m_tauDeactivation;
    m_fatigueParameters = c.m_fatigueParameters;

    return *this;
}

biorbd::muscles::Caracteristics::~Caracteristics()
{

}

void biorbd::muscles::Caracteristics::setStateMax(const biorbd::muscles::State &stateMax) {
    m_stateMax = stateMax;
}

const biorbd::muscles::State &biorbd::muscles::Caracteristics::stateMax() const {
    return m_stateMax;
}

const biorbd::muscles::FatigueParameters &biorbd::muscles::Caracteristics::fatigueParameters() const
{
    return m_fatigueParameters;
}

void biorbd::muscles::Caracteristics::fatigueParameters(const biorbd::muscles::FatigueParameters &fatigueParameters)
{
    m_fatigueParameters = fatigueParameters;
}
