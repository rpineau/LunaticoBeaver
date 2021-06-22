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

// SB includes
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#include "StopWatch.h"

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 500
#define MAX_READ_WAIT_TIMEOUT 25
#define NB_RX_WAIT 10
#define ND_LOG_BUFFER_SIZE 256
#define PANID_TIMEOUT 15    // in seconds
#define RAIN_CHECK_INTERVAL 10

#define PLUGIN_DEBUG 2
#define DRIVER_VERSION      2.65

#define DOME_STATUS_MASK        0x01
#define SHUTTER_STATUS_MASK     0x02
#define DOME_MECH_ERR_MASK      0x04
#define SHUTTER_MECH_ERR_MASK   0x08
#define SHUTTER_COM_ERR_MASK    0x10
#define RAIN_STATUS_MASK        0x60

// error codes
// Error code
enum DomeErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, COMMAND_TIMEOUT};
enum DomeShutterState {OPEN = 0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR };
enum HomeStatuses {NOT_HOME = 0, AT_HOME};
enum RainActions {DO_NOTHING=0, HOME, PARK};

// RG-11
enum RainSensorStates {RAINING= 0, NOT_RAINING};

class CLunaticoBeaver
{
public:
    CLunaticoBeaver();
    ~CLunaticoBeaver();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    const bool  IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeprPinter(SleeperInterface *p) {m_pSleeper = p; }

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double dNewAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *szVersion, int nStrMaxLen);
    int getFirmwareVersion(float &fVersion);
    int getShutterFirmwareVersion(char *szVersion, int nStrMaxLen);
    int goHome();
    int calibrate();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isParkComplete(bool &bComplete);
    int isUnparkComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);
    int isCalibratingComplete(bool &bComplete);

    int abortCurrentCommand();
    int getShutterPresent(bool &bShutterPresent);
    // getter/setter
    int getNbTicksPerRev();
    int setNbTicksPerRev(int nSteps);
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

    int getDefaultDir(bool &bNormal);
    int setDefaultDir(bool bNormal);

    int getRainSensorStatus(int &nStatus);

    int getRotationSpeed(int &nSpeed);
    int setRotationSpeed(int nSpeed);

    int getRotationAcceleration(int &nAcceleration);
    int setRotationAcceleration(int nAcceleration);

    int getShutterSpeed(int &nSpeed);
    int setShutterSpeed(int nSpeed);

    int getShutterAcceleration(int &nAcceleration);
    int setShutterAcceleration(int nAcceleration);

    void setHomeOnPark(const bool bEnabled);
    void setHomeOnUnpark(const bool bEnabled);

	int	getSutterWatchdogTimerValue(int &nValue);
	int	setSutterWatchdogTimerValue(const int &nValue);

    int getRainAction(int &nAction);
    int setRainAction(const int &nAction);
    
    
    void enableRainStatusFile(bool bEnable);
    void getRainStatusFileName(std::string &fName);
    void writeRainStatus();
    
protected:

    int             domeCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int             readResponse(std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int             getDomeAz(double &dDomeAz);
    int             getDomeEl(double &dDomeEl);
    int             getDomeHomeAz(double &dAz);
    int             getDomeParkAz(double &dAz);
    int             getShutterState(int &nState);
    int             getDomeStepPerDeg(int &nStepPerDeg);
    int             setDomeStepPerDeg(int nStepPerDeg);
    int             getDomeStatus(int &nStatus);
    
    bool            isDomeMoving();
    bool            isDomeAtHome();
    bool            checkBoundaries(double dGotoAz, double dDomeAz);

    int             parseFields(std::string sResp, std::vector<std::string> &svFields, char cSeparator);
    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);

    SerXInterface   *m_pSerx;
    SleeperInterface *m_pSleeper;

    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bShutterOpened;
    bool            m_bCalibrating;

    int             m_nStepsPerDeg;
    int             m_nNbStepPerRev;
    double          m_dShutterBatteryVolts;
    double          m_dHomeAz;

    double          m_dParkAz;

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bShutterOnly; // roll off roof so the arduino is running the shutter firmware only.
    char            m_szLogBuffer[ND_LOG_BUFFER_SIZE];
    int             m_nHomingTries;
    int             m_nGotoTries;
    bool            m_bParking;
    bool            m_bUnParking;
    int             m_nIsRaining;
    bool            m_bHomeOnPark;
    bool            m_bHomeOnUnpark;
    bool            m_bShutterPresent;

    int             m_nDomeRotStatus;
    int             m_nShutStatus;
    
    std::string     m_sRainStatusfilePath;
    FILE            *RainStatusfile;
    bool            m_bSaveRainStatus;
    CStopWatch      m_cRainCheckTimer;
    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;	  // LogFile
#endif

};

#endif
