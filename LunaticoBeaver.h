//
//  LunaticoBeaver.h
//
//  Created by Rodolphe Pineau on 2020/11/6.
//  LunaticoBeaver X2 plugin

#ifndef __LunaticoBeaver__
#define __LunaticoBeaver__

// standard C includes
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <math.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>

// SB includes
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"

#include "StopWatch.h"

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 500
#define MAX_READ_WAIT_TIMEOUT 25
#define ND_LOG_BUFFER_SIZE 256
#define RAIN_CHECK_INTERVAL 10

// #define PLUGIN_DEBUG 2
#define PLUGIN_VERSION      1.5

/* dome status
 • bit 0: ok moving rot
 • bit 1: ok moving shutter (3 ok moving both aka 2^0 + 2^1 = 3)
 • bit 2: mech error rotation
 • bit 3: mech error shutter
 • bit 4: comms error shutter
 • bit 5: unsafe by CW
 • bit 6: unsafe by Hydreon RG-x
 • bit 7: shutter open
 • bit 8: shutter closed
 • bit 9: shutter opening
 • bit 10. shutter closing
 • bit 11: Dome at home
 • bit 12: Dome at park
 */
#define DOME_STATUS_MASK        0x0007
#define SHUTTER_COM_STATUS_MASK 0x0008
#define RAIN_SENSOR_MASK        0x0060
#define SHUTTER_STATUS_MASK     0x0780
#define DOME_HOME_PARK_MASK     0x1800

#define DOME_MOVING                1
#define SHUTTER_MOVING             2
#define DOME_MECH_ERROR            4
#define SHUTTER_MECH_ERROR         8
#define SHUTTER_COM_ERROR         16
#define RAIN_CW                   32
#define RAIN_RG                   64
#define SHUTTER_OPEN             128
#define SHUTTER_CLOSED           256
#define SHUTTER_OPENING          512
#define SHUTTER_CLOSING         1024
#define DOME_AT_HOME            2048
#define DOME_AT_PARK            4096

// error codes
// Error code
enum DomeErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, COMMAND_TIMEOUT};
enum DomeShutterState {OPEN = 0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR };
enum HomeStatuses {NOT_HOME = 0, AT_HOME};
enum RainActions {DO_NOTHING=0, HOME, PARK};

// RG-11
enum RainSensorStates {RAINING= 0, NOT_RAINING, RAIN_UNNOWN};

class CLunaticoBeaver
{
public:
    CLunaticoBeaver();
    ~CLunaticoBeaver();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    const bool  IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double dNewAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(std::string &sVersion);
    int getFirmwareVersion(float &fVersion);
    int getShutterFirmwareVersion(std::string &sVersion);
    int goHome();
    int calibrateDome();
    int calibrateShutter();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isParkComplete(bool &bComplete);
    int isUnparkComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);
    int isCalibratingDomeComplete(bool &bComplete);
    int isCalibratingShutterComplete(bool &bComplete);

    int abortCurrentCommand();
    int getShutterPresent(bool &bShutterPresent);
    int setShutterPresent(bool bShutterPresent);
    int isShutterDetected(bool &bDetected);

    // getter/setter
    int getDomeStepPerRev();
    int setDomeStepPerRev(int nStep);

    int getBatteryLevel();

    double getHomeAz();
    int setHomeAz(double dAz);

    double getParkAz();
    int setParkAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();
    int getBatteryLevels(double &dShutterVolts, double &dShutterCutOff);
    int setBatteryCutOff(double dShutterCutOff);

    int getRainSensorStatus(int &nStatus);

    int getRotationSpeed(int &nMinSpeed, int &nMaxSpeed, int &nAccel);
    int setRotationSpeed(int nMinSpeed, int nMaxSpeed, int nAccel);

    int getShutterSpeed(int &nMinSpeed, int &nMaxSpeed, int &nAccel);
    int setShutterSpeed(int nMinSpeed, int nMaxSpeed, int nAccel);
    
    void enableRainStatusFile(bool bEnable);
    void getRainStatusFileName(std::string &fName);
    void writeRainStatus();

    int saveSettingsToEEProm();

protected:

    int             domeCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int             shutterCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int             readResponse(std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int             getDomeAz(double &dDomeAz);
    int             getDomeEl(double &dDomeEl);
    int             getDomeHomeAz(double &dAz);
    int             getDomeParkAz(double &dAz);
    int             getShutterState(int &nState);
    int             getDomeStepPerDeg(double &dStepPerDeg);
    int             setDomeStepPerDeg(double dStepPerDeg);
    int             getDomeStatus(int &nStatus);

    int             setMaxRotationTime(int nSeconds);

    bool            isDomeMoving();
    bool            isDomeAtHome();
    bool            checkBoundaries(double dGotoAz, double dDomeAz);

    int             parseFields(std::string sResp, std::vector<std::string> &svFields, char cSeparator);
    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);

    SerXInterface   *m_pSerx;

    bool            m_bIsConnected;
    bool            m_bParked;
    bool            m_bShutterOpened;
    bool            m_bCalibrating;

    double          m_dStepsPerDeg;
    int             m_nNbStepPerRev;
    double          m_dShutterBatteryVolts;
    double          m_dHomeAz;

    double          m_dParkAz;

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;

    std::string     m_sFirmwareVersion;
    int             m_nShutterState;
    bool            m_bShutterOnly; // roll off roof so the arduino is running the shutter firmware only.
    int             m_nHomingTries;
    int             m_nGotoTries;
    bool            m_bParking;
    bool            m_bUnParking;
    int             m_nRainSensorstate;
    bool            m_bHomeOnPark;
    bool            m_bHomeOnUnpark;
    bool            m_bShutterPresent;

    int             m_nDomeRotStatus;
    int             m_nShutStatus;

    std::string     m_sRainStatusfilePath;
    std::ofstream   m_RainStatusfile;
    int             m_nRainStatus;

    bool            m_bSaveRainStatus;
    CStopWatch      m_cRainCheckTimer;
    
#ifdef PLUGIN_DEBUG
    // timestamp for logs
    const std::string getTimeStamp();
    std::ofstream m_sLogFile;
    std::string m_sLogfilePath;
#endif

};

#endif
