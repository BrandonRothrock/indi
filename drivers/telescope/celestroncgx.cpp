/*******************************************************************************
 Copyright(c) 2015 Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "celestroncgx.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>

#define CCGX_VERSION_MAJOR 2
#define CCGX_VERSION_MINOR 0

// We declare an auto pointer to CelestronCGX.
static std::unique_ptr<CelestronCGX> cgx(new CelestronCGX());

#define MAX_SLEW_RATE 0x09
#define FIND_SLEW_RATE 0x07
#define CENTERING_SLEW_RATE 0x03
#define GUIDE_SLEW_RATE 0x02

void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    cgx->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    cgx->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    cgx->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    cgx->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
               char *formats[], char *names[], int n)
{
    cgx->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISSnoopDevice(XMLEle *root)
{
    cgx->ISSnoopDevice(root);
}

const uint32_t CelestronCGX::STEPS_PER_REVOLUTION = 0x1000000;
const double CelestronCGX::STEPS_PER_DEGREE       = STEPS_PER_REVOLUTION / 360.0;

CelestronCGX::CelestronCGX() : GI(this), m_communicator(Aux::ANY), m_alignment(STEPS_PER_REVOLUTION)
{
    setVersion(CCGX_VERSION_MAJOR, CCGX_VERSION_MINOR);

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO |
                               TELESCOPE_CAN_ABORT | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION |
                               TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK |
                               TELESCOPE_HAS_PIER_SIDE,
                           4);
}

const char *CelestronCGX::getDefaultName()
{
    return "Celestron CGX";
}

bool CelestronCGX::initProperties()
{
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    IUFillNumber(&EncoderTicksN[AXIS_RA], "ENCODER_TICKS_RA", "RA Encoder Ticks", "%.0f", 0,
                 STEPS_PER_REVOLUTION - 1, 1, m_alignment.GetStepsAtHomePositionRA());
    IUFillNumber(&EncoderTicksN[AXIS_DE], "ENCODER_TICKS_DEC", "Dec Encoder Ticks", "%.0f", 0,
                 STEPS_PER_REVOLUTION - 1, 1, m_alignment.GetStepsAtHomePositionDec());
    IUFillNumberVector(&EncoderTicksNP, EncoderTicksN, 2, getDeviceName(), "ENCODER_TICKS",
                       "Encoder Ticks", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    IUFillNumber(&LocationDebugN[0], "HA", "HA (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    IUFillNumber(&LocationDebugN[1], "LST", "LST (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    IUFillNumberVector(&LocationDebugNP, LocationDebugN, 2, getDeviceName(), "MOUNT_POINTING_DEBUG",
                       "Mount Pointing", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Add Tracking Modes, the order must match the order of the TelescopeTrackMode enum
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_SOLAR", "Solar");
    AddTrackMode("TRACK_LUNAR", "Lunar");

    IUFillSwitch(&AlignS[0], "ALIGN", "Align", ISS_OFF);
    IUFillSwitchVector(&AlignSP, AlignS, 1, getDeviceName(), "ALIGN", "Align", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    IUFillText(&VersionT[0], "VERSION_MAIN", "Main Version", "");
    IUFillText(&VersionT[1], "VERSION_DEC", "Dec Motor Version", "");
    IUFillText(&VersionT[2], "VERSION_RA", "RA Motor Version", "");
    IUFillTextVector(&VersionTP, VersionT, 3, getDeviceName(), "CGX_VERSION", "CGX Version",
                     OPTIONS_TAB, IP_RO, 0, IPS_IDLE);

    // Use the HA to park, as it is constant for a given mount orientation.
    SetParkDataType(PARK_HA_DEC);

    GI::initProperties(GUIDE_TAB);

    /* How fast do we guide compared to sidereal rate */
    IUFillNumber(&GuideRateN[AXIS_RA], "GUIDE_RATE_WE", "W/E Rate", "%.0f", 10, 100, 1, 50);
    IUFillNumber(&GuideRateN[AXIS_DE], "GUIDE_RATE_NS", "N/S Rate", "%.0f", 10, 100, 1, 50);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate",
                       GUIDE_TAB, IP_RW, 0, IPS_IDLE);

    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();

    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    serialConnection->setDefaultBaudRate(Connection::Serial::BaudRate::B_115200);

    setDefaultPollingPeriod(250);

    return true;
}

void CelestronCGX::ISGetProperties(const char *dev)
{
    INDI::Telescope::ISGetProperties(dev);
}

bool CelestronCGX::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        defineProperty(GuideNSNP);
        defineProperty(GuideWENP);
        defineNumber(&GuideRateNP);
        loadConfig(true, GuideRateNP.name);

        defineNumber(&EncoderTicksNP);
        defineNumber(&LocationDebugNP);

        defineSwitch(&AlignSP);
        defineText(&VersionTP);

        if (InitPark())
        {
            if (isParked())
            {
            }
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(-6.);
            SetAxis2ParkDefault(0.);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(-6.);
            SetAxis2Park(0.);
            SetAxis1ParkDefault(-6.);
            SetAxis2ParkDefault(0.);
        }

        sendTimeFromSystem();
    }
    else
    {
        deleteProperty(GuideNSNP.getName());
        deleteProperty(GuideWENP.getName());
        deleteProperty(GuideRateNP.name);
        deleteProperty(EncoderTicksNP.name);
        deleteProperty(LocationDebugNP.name);
        deleteProperty(AlignSP.name);
        deleteProperty(VersionTP.name);
    }

    return true;
}

bool CelestronCGX::ISNewNumber(const char *dev, const char *name, double values[], char *names[],
                               int n)
{
    // Check guider interface
    if (GI::processNumber(dev, name, values, names, n))
        return true;

    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, "GUIDE_RATE") == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);

            uint8_t ra =
                static_cast<uint8_t>(std::min(GuideRateN[AXIS_RA].value * 256 / 100, 255.0));
            uint8_t dec =
                static_cast<uint8_t>(std::min(GuideRateN[AXIS_DE].value * 256 / 100, 255.0));

            Aux::buffer raData(1);
            raData[0] = ra;

            Aux::buffer decData(1);
            decData[0] = dec;

            sendCmd(Aux::MC_SET_AUTOGUIDE_RATE, Aux::RA, raData);
            sendCmd(Aux::MC_SET_AUTOGUIDE_RATE, Aux::DEC, decData);

            return true;
        }

    }

    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    INDI::Telescope::ISNewNumber(dev, name, values, names, n);
    return true;
}

bool CelestronCGX::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[],
                               int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Alignment
        if (strcmp(name, AlignSP.name) == 0)
        {
            if (IUUpdateSwitch(&AlignSP, states, names, n) < 0)
                return false;

            startAlign();

            return true;
        }
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool CelestronCGX::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[],
                             char *blobs[], char *formats[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool CelestronCGX::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool CelestronCGX::Connect()
{
    LOG_INFO("CGX is online.");

    Aux::Communicator::setDeviceName(getDeviceName());

    return INDI::Telescope::Connect();
}

bool CelestronCGX::Disconnect()
{
    LOG_INFO("CGX is offline.");
    return INDI::Telescope::Disconnect();
}

bool CelestronCGX::Handshake()
{
    LOG_INFO("Starting Handshake");

    if (!sendCmd(Aux::GET_VER, Aux::RA))
    {
        LOG_ERROR("error sending raVer");
        return false;
    }

    if (!sendCmd(Aux::GET_VER, Aux::DEC))
    {
        LOG_ERROR("error sending decVer");
        return false;
    }

    return INDI::Telescope::Handshake();
}

bool CelestronCGX::sendCmd(Aux::Command cmd, Aux::Target dest, Aux::buffer data)
{
    Aux::Packet reply;
    if (!m_communicator.sendCommand(PortFD, dest, cmd, data, reply))
        return false;

    return handleResponse(reply);
}

bool CelestronCGX::handleResponse(Aux::Packet &pkt)
{
    switch (pkt.command)
    {
    case Aux::GET_VER:
    {
        char version[16];
        snprintf(version, sizeof(version), "%d.%d", pkt.data[0], pkt.data[1]);

        if (pkt.source == Aux::MB)
        {
            IUSaveText(&VersionTP.tp[0], version);
        }
        else if (pkt.source == Aux::DEC)
        {
            IUSaveText(&VersionTP.tp[1], version);
        }
        else if (pkt.source == Aux::RA)
        {
            IUSaveText(&VersionTP.tp[2], version);
        }
    }

        VersionTP.s = IPS_OK;
        IDSetText(&VersionTP, nullptr);

        return true;
    case Aux::MC_GET_POSITION:
        if (pkt.source == Aux::DEC)
        {
            uint32_t steps               = pkt.getPosition();
            EncoderTicksN[AXIS_DE].value = steps;
            m_alignment.UpdateStepsDec(steps);
        }
        else if (pkt.source == Aux::RA)
        {
            uint32_t steps               = pkt.getPosition();
            EncoderTicksN[AXIS_RA].value = steps;
            m_alignment.UpdateStepsRA(steps);

            LocationDebugN[0].value = m_alignment.hourAngleFromEncoder();
            LocationDebugN[1].value = m_alignment.localSiderealTime();

            IDSetNumber(&LocationDebugNP, nullptr);
        }
        EncoderTicksNP.s = IPS_OK;
        IDSetNumber(&EncoderTicksNP, nullptr);
        return true;
    case Aux::MC_LEVEL_START:
        return true;
    case Aux::MC_LEVEL_DONE:
        if (pkt.source == Aux::DEC)
        {
            m_decAligned = pkt.data.size() > 0 && pkt.data[0] == 0xff;
        }
        else if (pkt.source == Aux::RA)
        {
            m_raAligned = pkt.data.size() > 0 && pkt.data[0] == 0xff;
        }
        return true;

    case Aux::MC_MOVE_NEG:
        return true;
    case Aux::MC_MOVE_POS:
        return true;
    case Aux::MC_GOTO_FAST:
        return true;
    case Aux::MC_GOTO_SLOW:
        return true;
    case Aux::MC_SET_POSITION:
        return true;
    case Aux::MC_SET_POS_GUIDERATE:
        return true;
    case Aux::MC_SLEW_DONE:
        if (pkt.source == Aux::DEC)
        {
            m_decSlewing = pkt.data[0] == 0x00;
        }
        else if (pkt.source == Aux::RA)
        {
            m_raSlewing = pkt.data[0] == 0x00;
        }
        return true;
    case Aux::MC_GET_AUTOGUIDE_RATE:
        if (pkt.source == Aux::DEC)
        {
            GuideRateN[AXIS_DE].value = pkt.data[0] * 100.0 / 255;
        }
        else if (pkt.source == Aux::RA)
        {
            GuideRateN[AXIS_RA].value = pkt.data[0] * 100.0 / 255;
        }
        IDSetNumber(&GuideRateNP, nullptr);

        return true;
    case Aux::MC_SET_AUTOGUIDE_RATE:
        return true;
    case Aux::MC_AUX_GUIDE:
        return true;
    case Aux::MC_AUX_GUIDE_ACTIVE:
        if (pkt.source == Aux::DEC)
        {
            if (pkt.data[0] == 0)
            {
                GuideComplete(AXIS_DE);
            }
        }
        else if (pkt.source == Aux::RA)
        {
            if (pkt.data[0] == 0)
            {
                GuideComplete(AXIS_RA);
            }
        }
        return true;
    case Aux::MC_SET_CORDWRAP_POS:
        return true;
    case Aux::MC_ENABLE_CORDWRAP:
        return true;
    default:
        break;
    }

    LOGF_WARN("Unknown CMD=0x%02x src=0x%02x dst=0x%02x", pkt.command, pkt.source, pkt.destination);

    return true;
}

bool CelestronCGX::startAlign()
{
    AlignSP.s = IPS_BUSY;
    IDSetSwitch(&AlignSP, nullptr);

    m_raAligned    = false;
    m_decAligned   = false;
    m_alignSettling = false;

    if (!sendCmd(Aux::MC_LEVEL_START, Aux::RA))
    {
        LOG_ERROR("error starting align on az");
        return false;
    }

    if (!sendCmd(Aux::MC_LEVEL_START, Aux::DEC))
    {
        LOG_ERROR("error starting align on alt");
        return false;
    }

    return true;
}

bool CelestronCGX::getDec()
{
    return sendCmd(Aux::MC_GET_POSITION, Aux::DEC);
}

bool CelestronCGX::getRA()
{
    return sendCmd(Aux::MC_GET_POSITION, Aux::RA);
}

bool CelestronCGX::ReadScopeStatus()
{
    // Read any unsolicited messages from the mount.
    Aux::Packet unsolicited;
    while (m_communicator.readUnsolicited(PortFD, unsolicited))
        handleResponse(unsolicited);

    getDec();
    getRA();

    sendCmd(Aux::MC_GET_AUTOGUIDE_RATE, Aux::RA);
    sendCmd(Aux::MC_GET_AUTOGUIDE_RATE, Aux::DEC);

    if (GuideNSNP.getState() == IPS_BUSY)
    {
        sendCmd(Aux::MC_AUX_GUIDE_ACTIVE, Aux::DEC);
    }

    if (GuideWENP.getState() == IPS_BUSY)
    {
        sendCmd(Aux::MC_AUX_GUIDE_ACTIVE, Aux::RA);
    }

    if (AlignSP.s == IPS_BUSY)
    {
        if (!m_alignSettling)
        {
            sendCmd(Aux::MC_LEVEL_DONE, Aux::RA);
            sendCmd(Aux::MC_LEVEL_DONE, Aux::DEC);

            if (m_raAligned && m_decAligned)
            {
                // Motors reported aligned - wait a few polling cycles for them to fully stop
                m_alignSettling = true;
                m_alignSettleCount = 0;
            }
        }
        else
        {
            // Wait ~500ms worth of polling cycles (2 cycles at 250ms default polling)
            m_alignSettleCount++;
            if (m_alignSettleCount >= 2)
            {
                m_alignSettling = false;

                // We are at switch position, so set the motor position to be
                // in the middle of the range.
                Aux::Packet raCmd(Aux::ANY, Aux::RA, Aux::MC_SET_POSITION);
                raCmd.setPosition(m_alignment.GetStepsAtHomePositionRA());
                sendCmd(Aux::MC_SET_POSITION, Aux::RA, raCmd.data);

                Aux::Packet decCmd(Aux::ANY, Aux::DEC, Aux::MC_SET_POSITION);
                decCmd.setPosition(m_alignment.GetStepsAtHomePositionDec());
                sendCmd(Aux::MC_SET_POSITION, Aux::DEC, decCmd.data);

                Aux::Packet wrapCmd(Aux::ANY, Aux::RA, Aux::MC_SET_CORDWRAP_POS);
                wrapCmd.setPosition(m_alignment.encoderFromHourAngle(13.0));
                sendCmd(Aux::MC_SET_CORDWRAP_POS, Aux::RA, wrapCmd.data);

                sendCmd(Aux::MC_ENABLE_CORDWRAP, Aux::RA);

                TelescopeStatus state = TrackState;

                SetTrackEnabled(false);

                getDec();
                getRA();

                AlignSP.s   = IPS_OK;
                AlignS[0].s = ISS_OFF;
                IDSetSwitch(&AlignSP, nullptr);

                if (m_raTarget.has_value() && m_decTarget.has_value())
                {
                    // We are actually doing a slew to this target, so keep going.
                    StartSlew(*m_raTarget, *m_decTarget, state, true);

                    m_raTarget.reset();
                    m_decTarget.reset();
                }
                else
                {
                    LOG_INFO("CGX is now aligned");
                }
            }
        }
    }

    if (TrackState == SCOPE_SLEWING)
    {
        sendCmd(Aux::MC_SLEW_DONE, Aux::RA);
        sendCmd(Aux::MC_SLEW_DONE, Aux::DEC);

        if (m_manualSlew)
        {
            if (MovementNSSP.getState() == IPS_IDLE && MovementWESP.getState() == IPS_IDLE)
            {
                TrackState = RememberTrackState;
            }
        }
        else if (!m_decSlewing && !m_raSlewing)
        {
            // Always track after slew
            SetTrackEnabled(true);
        }
    }
    else if (TrackState == SCOPE_PARKING)
    {
        sendCmd(Aux::MC_SLEW_DONE, Aux::RA);
        sendCmd(Aux::MC_SLEW_DONE, Aux::DEC);

        if (!m_decSlewing && !m_raSlewing)
        {
            SetTrackEnabled(false);
            SetParked(true);
        }
    }

    EQAlignment::TelescopePierSide pierSide;
    double ra, dec;

    m_alignment.RADecFromEncoderValues(ra, dec, pierSide);

    setPierSide(static_cast<TelescopePierSide>(pierSide));
    NewRaDec(ra, dec);

    return true;
}

bool CelestronCGX::Goto(double r, double d)
{
    return StartSlew(r, d, SCOPE_SLEWING, true);
}

bool CelestronCGX::Abort()
{
    Aux::buffer dat(1);
    dat[0] = 0x00;

    sendCmd(Aux::MC_MOVE_POS, Aux::DEC, dat);
    sendCmd(Aux::MC_MOVE_POS, Aux::RA, dat);

    m_manualSlew = false;
    m_raTarget.reset();
    m_decTarget.reset();

    return INDI::Telescope::Abort();
}

bool CelestronCGX::Park()
{
    double hourAngle = GetAxis1Park();
    double dec       = GetAxis2Park();

    double lst = m_alignment.localSiderealTime();
    double ra  = lst - hourAngle;

    return StartSlew(ra, dec, SCOPE_PARKING);
}

bool CelestronCGX::UnPark()
{
    SetParked(false);
    return true;
}

bool CelestronCGX::SetTrackMode(uint8_t mode)
{
    INDI_UNUSED(mode);

    if (TrackStateSP.getState() == IPS_BUSY)
    {
        return SetTrackEnabled(true);
    }

    return true;
}

bool CelestronCGX::SetTrackEnabled(bool enabled)
{
    if (enabled)
    {
        Aux::buffer data(2);

        TelescopeTrackMode mode =
            static_cast<TelescopeTrackMode>(IUFindOnSwitchIndex(TrackModeSP));

        switch (mode)
        {
        case TRACK_SIDEREAL:
            data[0] = 0xff;
            data[1] = 0xff;
            break;
        case TRACK_SOLAR:
            data[0] = 0xff;
            data[1] = 0xfe;
            break;
        case TRACK_LUNAR:
            data[0] = 0xff;
            data[1] = 0xfd;
            break;
        default:
            return false;
        }

        TrackState = SCOPE_TRACKING;

        return sendCmd(Aux::MC_SET_POS_GUIDERATE, Aux::RA, data);
    }
    else
    {
        Aux::buffer data(3);
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;

        TrackState = SCOPE_IDLE;

        return sendCmd(Aux::MC_SET_POS_GUIDERATE, Aux::RA, data);
    }
}

bool CelestronCGX::SetCurrentPark()
{
    EQAlignment::TelescopePierSide pierSide;
    double ra, dec;

    m_alignment.RADecFromEncoderValues(ra, dec, pierSide);

    double lst       = m_alignment.localSiderealTime();
    double hourAngle = lst - ra;

    if (static_cast<TelescopePierSide>(pierSide) == PIER_WEST)
    {
        hourAngle -= 12.0;
    }

    SetAxis1Park(hourAngle);
    SetAxis2Park(dec);

    return true;
}

bool CelestronCGX::SetDefaultPark()
{
    SetAxis1Park(6.0);
    SetAxis2Park(90.0);

    return true;
}

bool CelestronCGX::SetParkPosition(double Axis1Value, double Axis2Value)
{
    SetAxis1Park(Axis1Value);
    SetAxis2Park(Axis2Value);

    return true;
}

bool CelestronCGX::Sync(double ra, double dec)
{
    EQAlignment::TelescopePierSide pierSide;
    uint32_t raSteps, decSteps;

    m_alignment.EncoderValuesFromRADec(ra, dec, raSteps, decSteps, pierSide);

    setPierSide(static_cast<TelescopePierSide>(pierSide));

    Aux::Packet raCmd(Aux::ANY, Aux::RA, Aux::MC_SET_POSITION);
    raCmd.setPosition(raSteps);
    sendCmd(Aux::MC_SET_POSITION, Aux::RA, raCmd.data);

    Aux::Packet decCmd(Aux::ANY, Aux::DEC, Aux::MC_SET_POSITION);
    decCmd.setPosition(decSteps);
    sendCmd(Aux::MC_SET_POSITION, Aux::DEC, decCmd.data);

    LOGF_INFO("sync: ra %0.3f; dec %0.3f; stepsRa %d; stepsDec %d;", ra, dec, raSteps, decSteps);

    // Be sure to update our local status.
    getDec();
    getRA();

    return true;
}

// common code for GoTo and park
bool CelestronCGX::StartSlew(double ra, double dec, TelescopeStatus status, bool skipPierSideCheck)
{
    const char *statusStr;
    switch (status)
    {
    case SCOPE_PARKING:
        statusStr = "Parking";
        break;
    case SCOPE_SLEWING:
        statusStr = "Slewing";
        break;
    default:
        statusStr = "unknown";
    }
    RememberTrackState = TrackState;
    TrackState         = status;

    EQAlignment::TelescopePierSide pierSide;
    uint32_t raSteps, decSteps;

    m_alignment.EncoderValuesFromRADec(ra, dec, raSteps, decSteps, pierSide);

    double currentRASteps  = EncoderTicksN[AXIS_RA].value;
    double currentDecSteps = EncoderTicksN[AXIS_DE].value;

    if (!skipPierSideCheck && currentPierSide != static_cast<TelescopePierSide>(pierSide))
    {
        // The mount will take the shortest distance to the new stepper count, so make sure we go
        // through home if we would otherwise do something crazy.

        if (std::abs(long(raSteps) - long(currentRASteps)) > long(STEPS_PER_REVOLUTION / 2) ||
            std::abs(long(decSteps) - long(currentDecSteps)) > long(STEPS_PER_REVOLUTION / 2))
        {
            m_raTarget  = ra;
            m_decTarget = dec;

            // Let's go back to home since we are changing pier sides. The mount otherwise wants to
            // take shortest distance, which can be wrong.
            // Takes a little longer to slew, but keeps things simple.

            LOGF_INFO("%s to home, then to %f %f, %d, %d", statusStr, ra, dec, raSteps, decSteps);

            return startAlign();
        }
    }

    bool raClose, decClose = false;

    raClose  = std::abs(long(raSteps) - long(currentRASteps)) < long(STEPS_PER_DEGREE * 4);
    decClose = std::abs(long(decSteps) - long(currentDecSteps)) < long(STEPS_PER_DEGREE * 4);

    Aux::Command cmd = raClose && decClose ? Aux::MC_GOTO_SLOW : Aux::MC_GOTO_FAST;

    Aux::Packet raCmd(Aux::ANY, Aux::RA, cmd);
    raCmd.setPosition(raSteps);
    if (!sendCmd(cmd, Aux::RA, raCmd.data))
        return false;

    Aux::Packet decCmd(Aux::ANY, Aux::DEC, cmd);
    decCmd.setPosition(decSteps);
    if (!sendCmd(cmd, Aux::DEC, decCmd.data))
        return false;

    m_manualSlew = false;

    LOGF_INFO("%s to %f %f %d, %d, %d", statusStr, ra, dec, cmd, raSteps, decSteps);

    return true;
}

uint8_t CelestronCGX::slewRate()
{
    int index = IUFindOnSwitchIndex(SlewRateSP);

    switch (index)
    {
    case SLEW_GUIDE:
        return GUIDE_SLEW_RATE;
    case SLEW_CENTERING:
        return CENTERING_SLEW_RATE;
    case SLEW_FIND:
        return FIND_SLEW_RATE;
    case SLEW_MAX:
        return MAX_SLEW_RATE;
    }

    return FIND_SLEW_RATE;
}

bool CelestronCGX::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    m_manualSlew = true;

    Aux::buffer dat(1);
    dat[0] = 0x00;

    if (command == MOTION_STOP)
    {
        LOG_INFO("Stopping DEC motor");
        return sendCmd(Aux::MC_MOVE_POS, Aux::DEC, dat);
    }

    TrackState = SCOPE_SLEWING;

    dat[0] = slewRate();

    // On a GEM, motor direction must be inverted when on the west side of the pier
    INDI_DIR_NS actualDir = dir;
    if (currentPierSide == PIER_WEST)
        actualDir = (dir == DIRECTION_NORTH) ? DIRECTION_SOUTH : DIRECTION_NORTH;

    return sendCmd(actualDir == DIRECTION_NORTH ? Aux::MC_MOVE_NEG : Aux::MC_MOVE_POS, Aux::DEC, dat);
}

bool CelestronCGX::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    m_manualSlew = true;

    Aux::buffer dat(1);
    dat[0] = 0x00;

    if (command == MOTION_STOP)
    {
        LOG_INFO("Stopping RA motor");
        return sendCmd(Aux::MC_MOVE_POS, Aux::RA, dat);
    }

    TrackState = SCOPE_SLEWING;

    dat[0] = slewRate();

    return sendCmd(dir == DIRECTION_WEST ? Aux::MC_MOVE_POS : Aux::MC_MOVE_NEG, Aux::RA, dat);
}

bool CelestronCGX::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &GuideRateNP);

    return true;
}

bool CelestronCGX::updateLocation(double latitude, double longitude, double elevation)
{
    LOGF_INFO("Update location %8.3f, %8.3f, %4.0f", latitude, longitude, elevation);

    m_alignment.UpdateLongitude(longitude);

    return true;
}

/////////////////////////////////////////////////////////////////////
// Autoguiding

IPState CelestronCGX::GuideNorth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: N %.0f ms", ms);

    uint8_t ticks = std::min(uint32_t(255), ms / 10);

    int8_t rate = static_cast<int8_t>(GuideRateN[AXIS_DE].value);

    Aux::buffer data(2);
    data[0] = rate;
    data[1] = ticks;

    sendCmd(Aux::MC_AUX_GUIDE, Aux::DEC, data);

    return IPS_BUSY;
}

IPState CelestronCGX::GuideSouth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: S %.0f ms", ms);

    uint8_t ticks = std::min(uint32_t(255), ms / 10);

    int8_t rate = static_cast<int8_t>(GuideRateN[AXIS_DE].value);

    Aux::buffer data(2);
    data[0] = -rate;
    data[1] = ticks;

    sendCmd(Aux::MC_AUX_GUIDE, Aux::DEC, data);

    return IPS_BUSY;
}

IPState CelestronCGX::GuideEast(uint32_t ms)
{
    LOGF_DEBUG("Guiding: E %.0f ms", ms);

    uint8_t ticks = std::min(uint32_t(255), ms / 10);

    int8_t rate = static_cast<int8_t>(GuideRateN[AXIS_RA].value);

    Aux::buffer data(2);
    data[0] = -rate;
    data[1] = ticks;

    sendCmd(Aux::MC_AUX_GUIDE, Aux::RA, data);

    return IPS_BUSY;
}

IPState CelestronCGX::GuideWest(uint32_t ms)
{
    LOGF_DEBUG("Guiding: W %.0f ms", ms);

    uint8_t ticks = std::min(uint32_t(255), ms / 10);

    int8_t rate = static_cast<int8_t>(GuideRateN[AXIS_RA].value);

    Aux::buffer data(2);
    data[0] = rate;
    data[1] = ticks;

    sendCmd(Aux::MC_AUX_GUIDE, Aux::RA, data);

    return IPS_BUSY;
}
