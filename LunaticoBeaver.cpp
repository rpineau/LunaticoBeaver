//
//  LunaticoBeaver.cpp
//  LunaticoBeaver X2 plugin
//
//  Created by Rodolphe Pineau on 2020/11/6.


#include "LunaticoBeaver.h"

CLunaticoBeaver::CLunaticoBeaver()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nStepsPerDeg = 0;
    m_dShutterBatteryVolts = 0.0;

    m_dHomeAz = 0;
    m_dParkAz = 0;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;
    m_bParking = false;
    m_bUnParking = false;

    m_bShutterOpened = false;

    m_bParked = true;
    m_bHomed = false;

    m_nHomingTries = 0;
    m_nGotoTries = 0;

    m_nIsRaining = NOT_RAINING;
    m_bSaveRainStatus = false;
    RainStatusfile = NULL;
    m_cRainCheckTimer.Reset();

    m_bHomeOnPark = false;
    m_bHomeOnUnpark = false;

    m_bShutterPresent = false;

#ifdef    PLUGIN_DEBUG
    Logfile = NULL;
#endif
    
    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,ND_LOG_BUFFER_SIZE);

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\LunaticoBeaver-Log.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/LunaticoBeaver-Log.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/LunaticoBeaver-Log.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined(SB_WIN_BUILD)
    m_sRainStatusfilePath = getenv("HOMEDRIVE");
    m_sRainStatusfilePath += getenv("HOMEPATH");
    m_sRainStatusfilePath += "\\LunaticoBeaver_Rain.txt";
#elif defined(SB_LINUX_BUILD)
    m_sRainStatusfilePath = getenv("HOME");
    m_sRainStatusfilePath += "/LunaticoBeaver_Rain.txt";
#elif defined(SB_MAC_BUILD)
    m_sRainStatusfilePath = getenv("HOME");
    m_sRainStatusfilePath += "/LunaticoBeaver_Rain.txt";
#endif
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver] Version %3.2f build 2019_12_06_1810.\n", timestamp, DRIVER_VERSION);
    fprintf(Logfile, "[%s] [CLunaticoBeaver] Constructor Called.\n", timestamp);
    fprintf(Logfile, "[%s] [CLunaticoBeaver] Rains status file : '%s'.\n", timestamp, m_sRainStatusfilePath.c_str());
    fflush(Logfile);
#endif

}

CLunaticoBeaver::~CLunaticoBeaver()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if (Logfile)
        fclose(Logfile);
#endif
    if(RainStatusfile) {
        fclose(RainStatusfile);
        RainStatusfile = NULL;
    }
}

int CLunaticoBeaver::Connect(const char *pszPort)
{
    int nErr;
    bool bDummy;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif
    m_bIsConnected = false;
    m_bCalibrating = false;
    m_bUnParking = false;
    m_bHomed = false;

    // 115200 8N1
    nErr = m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY);
    if(nErr) {
        return nErr;
    }
    m_bIsConnected = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] connected to %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] Getting Firmware\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] Error Getting Firmware.\n", timestamp);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }
    nErr = getDomeParkAz(m_dCurrentAzPosition);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] getDomeParkAz nErr : %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    nErr = getDomeHomeAz(m_dHomeAz);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::Connect] getDomeHomeAz nErr : %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    getShutterPresent(bDummy);

    return SB_OK;
}


void CLunaticoBeaver::Disconnect()
{
    if(m_bIsConnected) {
        abortCurrentCommand();
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
    m_bCalibrating = false;
    m_bUnParking = false;
    m_bHomed = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::Disconnect] m_bIsConnected = %d\n", timestamp, m_bIsConnected);
    fflush(Logfile);
#endif
}


int CLunaticoBeaver::domeCommand(const std::string sCmd, std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::domeCommand] sending : %s\n", timestamp, sCmd.c_str());
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    nErr = readResponse(sResp, nTimeout);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::domeCommand] ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::domeCommand] response : %s\n", timestamp, sResp.c_str());
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::readResponse(std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    char pszBuf[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    memset(pszBuf, 0, (size_t) SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;
    sResp.clear();

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] nBytesWaiting = %d\n", timestamp, nBytesWaiting);
        fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] nBytesWaiting nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        if(!nBytesWaiting) {
            if(nbTimeouts++ >= NB_RX_WAIT) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] bytesWaitingRx timeout, no data for %d loops\n", timestamp, NB_RX_WAIT);
                fflush(Logfile);
#endif
                nErr = ERR_RXTIMEOUT;
                break;
            }
            m_pSleeper->sleep(MAX_READ_WAIT_TIMEOUT);
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] readFile error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] readFile Timeout Error\n", timestamp);
            fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] readFile nBytesWaiting = %d\n", timestamp, nBytesWaiting);
            fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] readFile ulBytesRead = %lu\n", timestamp, ulBytesRead);
            fflush(Logfile);
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    } while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != '#');

    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else
        *(pszBufPtr-1) = 0; //remove the #

    sResp.assign(pszBuf);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::readResponse] sResp  = '%s'\n", timestamp, sResp.c_str());
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::getDomeAz(double &dDomeAz)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!dome getaz#", sResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeAz] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        dDomeAz = std::stod(svFields[1]);
        m_dCurrentAzPosition = dDomeAz;
    }
    if(m_cRainCheckTimer.GetElapsedSeconds() > RAIN_CHECK_INTERVAL) {
        writeRainStatus();
        m_cRainCheckTimer.Reset();
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeAz] Dome Az = %3.2f\n", timestamp, dDomeAz);
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::getDomeEl(double &dDomeEl)
{
    int nErr = PLUGIN_OK;
    // std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    if(!m_bShutterOpened)
    {
        dDomeEl = 0.0;
        return nErr;
    }
    else {
        dDomeEl = 90.0;
        return nErr;
    }

}


int CLunaticoBeaver::getDomeHomeAz(double &dAz)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    dAz = 0;
    
    nErr = domeCommand("!domerot gethome#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeHomeAz] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        dAz = std::stod(svFields[1]);
    }

    m_dHomeAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeHomeAz] m_dHomeAz = %3.2f\n", timestamp, m_dHomeAz);
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::getDomeParkAz(double &dAz)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    dAz = 0;

    nErr = domeCommand("!domerot getpark#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeParkAz] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }

    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        dAz = std::stod(svFields[1]);
    }
    m_dParkAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeParkAz] m_dParkAz = %3.2f\n", timestamp, m_dParkAz);
        fflush(Logfile);
#endif

    return nErr;
}


int CLunaticoBeaver::getShutterState(int &nState)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> shutterStateFileds;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nState = SHUTTER_ERROR;
        return nErr;
    }

    if(m_bCalibrating)
        return nErr;

	
    nState = SHUTTER_ERROR;
    
    nErr = domeCommand("!dome shutterstatus#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterState] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        nState = SHUTTER_ERROR;
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterState] response = '%s'\n", timestamp, sResp.c_str());
    fflush(Logfile);
#endif

    parseFields(sResp, shutterStateFileds, ':');
    if(shutterStateFileds.size()>=2) {
        nState = std::stoi(shutterStateFileds[1]);
    }


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterState] nState = '%d'\n", timestamp, nState);
    fflush(Logfile);
#endif

    return nErr;
}


int CLunaticoBeaver::getDomeStepPerDeg(int &nStepsPerDeg)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nStepsPerDeg = 0;
    nErr = domeCommand("domerot getstepsperdegree#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeStepPerDeg] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }

    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        nStepsPerDeg = std::stoi(svFields[1]);
    }

    m_nStepsPerDeg = nStepsPerDeg;
    return nErr;
}

int CLunaticoBeaver::setDomeStepPerDeg(int nStepsPerDeg)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;
    
    m_nStepsPerDeg = nStepsPerDeg;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssTmp<<"!domerot setstepsperdegree " << nStepsPerDeg <<"#";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;

}

int CLunaticoBeaver::getBatteryLevels(double &dShutterVolts, double &dShutterCutOff)
{
    int nErr = PLUGIN_OK;
    int rc = 0;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    dShutterVolts  = 0;
    dShutterCutOff = 0;
    if(m_bShutterPresent) {
        nErr = domeCommand("shutter getvoltage#", sResp);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CLunaticoBeaver::getBatteryLevels] ERROR = %s\n", timestamp, sResp.c_str());
            fflush(Logfile);
#endif
            return nErr;
        }
        
        parseFields(sResp, svFields, ':');
        if(svFields.size()>=2) {
            dShutterVolts = std::stoi(svFields[1]);
        }

        nErr = domeCommand("shutter getsafevoltage#", sResp);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CLunaticoBeaver::getBatteryLevels] ERROR = %s\n", timestamp, sResp.c_str());
            fflush(Logfile);
#endif
            return nErr;
        }
        
        parseFields(sResp, svFields, ':');
        if(svFields.size()>=2) {
            dShutterCutOff = std::stoi(svFields[1]);
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getBatteryLevels] dShutterVolts = %f\n", timestamp, dShutterVolts);
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getBatteryLevels] dShutterCutOff = %f\n", timestamp, dShutterCutOff);
        fflush(Logfile);
#endif

    }
    return nErr;
}

int CLunaticoBeaver::setBatteryCutOff(double dShutterCutOff)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    ssTmp<<"!shutter setsafevoltage " << dShutterCutOff <<"#";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

bool CLunaticoBeaver::isDomeMoving()
{
    bool bIsMoving = false;
    int nTmp;
    
    getDomeStatus(nTmp);
    
    if(m_nDomeRotStatus)
        bIsMoving = true;
    
    return bIsMoving;
}

int CLunaticoBeaver::getDomeStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    
    nStatus = 0;
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("dome status#", sResp);
    if(nErr & !m_bCalibrating) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isDomeMoving] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }

    // need to parse sResp
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=1) {
        nStatus = std::stoi(svFields[1]);
    }

    m_nDomeRotStatus = nStatus & DOME_STATUS_MASK;
    m_nShutStatus = nStatus & SHUTTER_STATUS_MASK;
    m_nIsRaining = nStatus & RAIN_STATUS_MASK;
    
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeStatus] nStatus : %d\n", timestamp, nStatus);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeStatus] m_nDomeRotStatus : %d\n", timestamp, m_nDomeRotStatus);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeStatus] m_nShutStatus : %d\n", timestamp, m_nShutStatus);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDomeStatus] m_nIsRaining : %d\n", timestamp, m_nIsRaining);
    fflush(Logfile);
#endif
    
    return nErr;
}

bool CLunaticoBeaver::isDomeAtHome()
{
    bool bAthome;
    int nTmp;
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("dome athome#", sResp);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isDomeAtHome] z# response = %s\n", timestamp, sResp.c_str());
    fflush(Logfile);
#endif
    if(nErr) {
        return false;
    }

    bAthome = false;
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=1) {
        nTmp = std::stoi(svFields[1]);
    }
    else
        nTmp = 0;

    if(nTmp == AT_HOME)
        bAthome = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isDomeAtHome] nTmp = %d\n", timestamp, nTmp);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isDomeAtHome] bAthome : %s\n", timestamp, bAthome?"True":"False");
    fflush(Logfile);
#endif

    return bAthome;

}

int CLunaticoBeaver::syncDome(double dAz, double dEl)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "s%3.2f#", dAz);
    nErr = domeCommand(szBuf, sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::syncDome] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    // TODO : Also set Elevation when supported by the firmware.
    // m_dCurrentElPosition = dEl;
    return nErr;
}

int CLunaticoBeaver::parkDome()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bHomeOnPark) {
        m_bParking = true;
        nErr = goHome();
    } else {
        nErr = domeCommand("!dome gopark#", sResp);
    }
    return nErr;

}

int CLunaticoBeaver::unparkDome()
{
    if(m_bHomeOnUnpark) {
        m_bUnParking = true;
        goHome();
    }
    else {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::unparkDome] m_dParkAz = %3.3f\n", timestamp, m_dParkAz);
        fflush(Logfile);
#endif
        syncDome(m_dParkAz, m_dCurrentElPosition);
        m_bParked = false;
        m_bUnParking = false;
    }

    return 0;
}

int CLunaticoBeaver::gotoAzimuth(double dNewAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "g%3.2f#", dNewAz);
    nErr = domeCommand(szBuf, sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::gotoAzimuth] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    m_dGotoAz = dNewAz;
    m_nGotoTries = 0;
    return nErr;
}

int CLunaticoBeaver::openShutter()
{
    int nErr = PLUGIN_OK;
    bool bDummy;
    std::string sResp;
    double domeVolts;
    double dDomeCutOff;
    double dShutterVolts;
    double dShutterCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    getShutterPresent(bDummy);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::openShutter] m_bShutterPresent = %s\n", timestamp, m_bShutterPresent?"Yes":"No");
    fflush(Logfile);
#endif
    if(!m_bShutterPresent) {
        return SB_OK;
    }

    getBatteryLevels(dShutterVolts, dShutterCutOff);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::openShutter] Opening shutter\n", timestamp);
    fflush(Logfile);
#endif

	
    nErr = domeCommand("shutter open#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::openShutter] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }
//    if(szResp[0] == 'L') { // batteryb LOW.. can't open
//#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
//        ltime = time(NULL);
//        timestamp = asctime(localtime(&ltime));
//        timestamp[strlen(timestamp) - 1] = 0;
//        fprintf(Logfile, "[%s] [CLunaticoBeaver::openShutter] Voltage too low to open\n", timestamp);
//        fflush(Logfile);
//#endif
//        nErr = ERR_CMDFAILED;
//    }
    return nErr;
}

int CLunaticoBeaver::closeShutter()
{
    int nErr = PLUGIN_OK;
    bool bDummy;
    std::string sResp;
    double dShutterVolts;
    double dShutterCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    getShutterPresent(bDummy);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::closeShutter] m_bShutterPresent = %s\n", timestamp, m_bShutterPresent?"Yes":"No");
    fflush(Logfile);
#endif

    if(!m_bShutterPresent) {
        return SB_OK;
    }

    getBatteryLevels(dShutterVolts, dShutterCutOff);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::closeShutter] Closing shutter\n", timestamp);
    fflush(Logfile);
#endif

	
    nErr = domeCommand("dome closeshutter#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::openShutter] closeShutter = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

    return nErr;
}

int CLunaticoBeaver::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    memset(szVersion, 0, nStrMaxLen);

    nErr = domeCommand("!seletek version#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getFirmwareVersion] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = parseFields(sResp, firmwareFields, ':');
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getFirmwareVersion] parsing error = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    if(firmwareFields.size()>=2) {
        std::stringstream ssTmp;
        if(firmwareFields[1].size()>=3) {
            ssTmp << firmwareFields[1].at(1) << "." << firmwareFields[1].at(2) << "." << firmwareFields[1].at(3);
            strncpy(szVersion, ssTmp.str().c_str(), nStrMaxLen);
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getFirmwareVersion] firmware = %s\n", timestamp, szVersion);
    fflush(Logfile);
#endif

    return nErr;
}


int CLunaticoBeaver::getShutterFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    memset(szVersion, 0, nStrMaxLen);

    nErr = domeCommand("!dome sendtoshutter \"seletek version\"#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterFirmwareVersion] ERROR = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = parseFields(sResp, firmwareFields, ':');
    if(nErr) {
        return ERR_CMDFAILED;
    }
    if(firmwareFields.size()>=2) {
        std::stringstream ssTmp;
        if(firmwareFields[1].size()>=3) {
            ssTmp << firmwareFields[1].at(1) << "." << firmwareFields[1].at(2) << "." << firmwareFields[1].at(3);
            strncpy(szVersion, ssTmp.str().c_str(), nStrMaxLen);
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterFirmwareVersion] firmware = %s\n", timestamp, szVersion);
    fflush(Logfile);
#endif

    return nErr;
}


int CLunaticoBeaver::goHome()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating) {
        return SB_OK;
    }
    else if(isDomeAtHome()){
            m_bHomed = true;
            return PLUGIN_OK;
    }
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::goHome] \n", timestamp);
    fflush(Logfile);
#endif

    m_nHomingTries = 0;
    nErr = domeCommand("!dome gohome#", sResp);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::goHome] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    return nErr;
}

int CLunaticoBeaver::calibrate()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("!dome autocalshutter#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::calibrate] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = domeCommand("!dome autocalrot#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::calibrate] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    m_bCalibrating = true;

    return nErr;
}

int CLunaticoBeaver::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bComplete = false;
    if(isDomeMoving()) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] Dome is still moving\n", timestamp);
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
        fflush(Logfile);
#endif
        return nErr;
    }

    getDomeAz(dDomeAz);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] DomeAz    = %3.2f\n", timestamp, dDomeAz);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] m_dGotoAz = %3.2f\n", timestamp, m_dGotoAz);
    fflush(Logfile);
#endif

    if(checkBoundaries(m_dGotoAz, dDomeAz)) {
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] ***** ERROR **** domeAz = %3.2f, m_dGotoAz = %3.2f\n", timestamp, dDomeAz, m_dGotoAz);
        fflush(Logfile);
#endif
        if(m_nGotoTries == 0) {
            bComplete = false;
            m_nGotoTries = 1;
            gotoAzimuth(m_dGotoAz);
        }
        else {
            m_nGotoTries = 0;
            nErr = ERR_CMDFAILED;
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isGoToComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

bool CLunaticoBeaver::checkBoundaries(double dGotoAz, double dDomeAz)
{
    double highMark;
    double lowMark;
    double roundedGotoAz;

    // we need to test "large" depending on the heading error and movement coasting
    highMark = ceil(dDomeAz)+2;
    lowMark = ceil(dDomeAz)-2;
    roundedGotoAz = ceil(dGotoAz);

    if(lowMark < 0 && highMark>0) { // we're close to 0 degre but above 0
        if((roundedGotoAz+2) >= 360)
            roundedGotoAz = (roundedGotoAz+2)-360;
        if ( (roundedGotoAz > lowMark) && (roundedGotoAz <= highMark)) {
            return true;
        }
    }
    if ( lowMark > 0 && highMark>360 ) { // we're close to 0 but from the other side
        if( (roundedGotoAz+360) > lowMark && (roundedGotoAz+360) <= highMark) {
            return true;
        }
    }
    if (roundedGotoAz > lowMark && roundedGotoAz <= highMark) {
        return true;
    }

    return false;
}


int CLunaticoBeaver::isOpenComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;
    bool bDummy;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterPresent(bDummy);
    if(!m_bShutterPresent) {
        bComplete = true;
        return SB_OK;
    }

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isOpenComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::isCloseComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;
    bool bDummy;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterPresent(bDummy);
    if(!m_bShutterPresent) {
        bComplete = true;
        return SB_OK;
    }

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isCloseComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}


int CLunaticoBeaver::isParkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz=0;
    bool bFoundHome;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isParkComplete] m_bParking = %s\n", timestamp, m_bParking?"True":"False");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        getDomeAz(dDomeAz);
        bComplete = false;
        return nErr;
    }

    if(m_bParking) {
        bComplete = false;
        nErr = isFindHomeComplete(bFoundHome);
        if(bFoundHome) { // we're home, now park
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CLunaticoBeaver::isParkComplete] found home, now parking\n", timestamp);
            fflush(Logfile);
#endif
            m_bParking = false;
            nErr = gotoAzimuth(m_dParkAz);
        }
        return nErr;
    }

    getDomeAz(dDomeAz);

    // we need to test "large" depending on the heading error
    if ((ceil(m_dParkAz) <= ceil(dDomeAz)+3) && (ceil(m_dParkAz) >= ceil(dDomeAz)-3)) {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::isUnparkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bParked) {
        bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isUnparkComplete] UNPARKED \n", timestamp);
        fflush(Logfile);
#endif
    }
    else if (m_bUnParking) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isUnparkComplete] unparking.. checking if we're home \n", timestamp);
        fflush(Logfile);
#endif
        nErr = isFindHomeComplete(bComplete);
        if(nErr)
            return nErr;
        if(bComplete) {
            m_bParked = false;
        }
        else {
            m_bParked = true;
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isUnparkComplete] m_bParked = %s\n", timestamp, m_bParked?"True":"False");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isUnparkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::isFindHomeComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::isFindHomeComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        m_bHomed = false;
        bComplete = false;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isFindHomeComplete] still moving\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;

    }

    if(isDomeAtHome()){
        m_bHomed = true;
        bComplete = true;
        if(m_bUnParking)
            m_bParked = false;
        syncDome(m_dHomeAz, m_dCurrentElPosition);
        m_nHomingTries = 0;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isFindHomeComplete] At Home\n", timestamp);
        fflush(Logfile);
#endif
    }
    else {
        // we're not moving and we're not at the home position !!!
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::isFindHomeComplete] Not moving and not at home !!!\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        // sometimes we pass the home sensor and the dome doesn't rotate back enough to detect it.
        // this is mostly the case with firmware 1.10 with the new error correction ...
        // so give it another try
        if(m_nHomingTries == 0) {
            m_nHomingTries = 1;
            goHome();
        }
        return ERR_CMDFAILED;
    }

    return nErr;
}


int CLunaticoBeaver::isCalibratingComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }


    nErr = getDomeAz(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    }

    nErr = getDomeStepPerDeg(m_nStepsPerDeg);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] final m_nStepsPerDeg = %d\n", timestamp, m_nStepsPerDeg);
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] final m_bHomed = %s\n", timestamp, m_bHomed?"True":"False");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] final m_bCalibrating = %s\n", timestamp, m_bCalibrating?"True":"False");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] final bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif
    return nErr;
}


int CLunaticoBeaver::abortCurrentCommand()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bHomed = false;
    m_bParked = false;
    m_bCalibrating = false;
    m_bParking = false;
    m_bUnParking = false;
    m_nGotoTries = 1;   // prevents the goto retry
    m_nHomingTries = 1; // prevents the find home retry

    nErr = domeCommand("!dome abort 1 1 1#", sResp);

    getDomeAz(m_dGotoAz);

    return nErr;
}


int CLunaticoBeaver::getShutterPresent(bool &bShutterPresent)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    nErr = domeCommand("!dome getshutterenable#", sResp);
    if(nErr) {
        return nErr;
    }
    m_bShutterPresent = false;

    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        bShutterPresent = (std::stoi(svFields[1])==1);
        m_bShutterPresent = bShutterPresent;
    }

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterPresent] szResp =  %s\n", timestamp, sResp.c_str());
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterPresent] m_bShutterPresent =  %s\n", timestamp, m_bShutterPresent?"Yes":"No");
    fflush(Logfile);
#endif


    bShutterPresent = m_bShutterPresent;
    return nErr;

}

#pragma mark - Getter / Setter

int CLunaticoBeaver::getNbTicksPerRev()
{
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] m_bIsConnected = %s\n", timestamp, m_bIsConnected?"True":"False");
    fflush(Logfile);
#endif

    if(m_bIsConnected) {
        getDomeStepPerDeg(m_nStepsPerDeg);
        m_nNbStepPerRev = m_nStepsPerDeg*360.0;
    }
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getNbTicksPerRev] m_nNbStepPerRev = %d\n", timestamp, m_nNbStepPerRev);
    fflush(Logfile);
#endif

    return m_nNbStepPerRev;
}

int CLunaticoBeaver::setNbTicksPerRev(int nSteps)
{
    int nErr = PLUGIN_OK;

    if(m_bIsConnected) {
        nErr = setDomeStepPerRev(nSteps);
        m_nNbStepPerRev = nSteps*360.0;

    }
    return nErr;
}

int CLunaticoBeaver::setDomeStepPerRev(int nSteps)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    return nErr;
}

double CLunaticoBeaver::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz(m_dHomeAz);
    return m_dHomeAz;
}

int CLunaticoBeaver::setHomeAz(double dAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    m_dHomeAz = dAz;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "!domerot sethome %3.2f#", dAz);
    nErr = domeCommand(szBuf, sResp);
    return nErr;
}


double CLunaticoBeaver::getParkAz()
{
    if(m_bIsConnected)
        getDomeParkAz(m_dParkAz);

    return m_dParkAz;

}

int CLunaticoBeaver::setParkAz(double dAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    m_dParkAz = dAz;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "!domerot setpark %3.2f#", dAz);
    nErr = domeCommand(szBuf, sResp);
    return nErr;
}


double CLunaticoBeaver::getCurrentAz()
{

    if(m_bIsConnected) {
        getDomeAz(m_dCurrentAzPosition);
   }
    return m_dCurrentAzPosition;
}

double CLunaticoBeaver::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);

    return m_dCurrentElPosition;
}

int CLunaticoBeaver::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}


int CLunaticoBeaver::getDefaultDir(bool &bNormal)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    bNormal = true;
    // nErr = domeCommand("y#", sResp);
    // if(nErr) {
    //     return nErr;
    // }


    // bNormal = atoi(szResp) ? false:true;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fflush(Logfile);
#endif


    return nErr;
}

int CLunaticoBeaver::setDefaultDir(bool bNormal)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "y %1d#", bNormal?0:1);

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::setDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::setDefaultDir] szBuf =  %s\n", timestamp, szBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(szBuf, sResp);
    return nErr;

}

int CLunaticoBeaver::getRainSensorStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nStatus = NOT_RAINING;
//    nErr = domeCommand("F#", sResp);
//    if(nErr) {
//        return nErr;
//    }

//    nStatus = atoi(szResp) ? false:true;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getRainSensorStatus] nStatus =  %d\n", timestamp, nStatus);
    fflush(Logfile);
#endif


    m_nIsRaining = nStatus;
    return nErr;
}

int CLunaticoBeaver::getRotationSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
/*
    nErr = domeCommand("r#", sResp);
    if(nErr) {
        return nErr;
    }

    nSpeed = atoi(szResp);
*/

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getRotationSpeed] nSpeed =  %d\n", timestamp, nSpeed);
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::setRotationSpeed(int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "r%d#", nSpeed);
    nErr = domeCommand(szBuf, sResp);
    return nErr;
}


int CLunaticoBeaver::getRotationAcceleration(int &nAcceleration)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
/*
    nErr = domeCommand("e#", sResp);
    if(nErr) {
        return nErr;
    }

    nAcceleration = atoi(szResp);
*/
 #ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getRotationAcceleration] nAcceleration =  %d\n", timestamp, nAcceleration);
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::setRotationAcceleration(int nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "e%d#", nAcceleration);
    nErr = domeCommand(szBuf, sResp);

    return nErr;
}

int CLunaticoBeaver::getShutterSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nSpeed = 0;
        return SB_OK;
    }
/*
	
    nErr = domeCommand("R#", sResp);
    if(nErr) {
        return nErr;
    }

    nSpeed = atoi(szResp);
*/
 #ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterSpeed] nSpeed =  %d\n", timestamp, nSpeed);
    fflush(Logfile);
#endif

    return nErr;
}

int CLunaticoBeaver::setShutterSpeed(int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        return SB_OK;
    }

	
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "R%d#", nSpeed);
    nErr = domeCommand(szBuf, sResp);

    return nErr;
}

int CLunaticoBeaver::getShutterAcceleration(int &nAcceleration)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nAcceleration = 0;
        return SB_OK;
    }
/*
	
    nErr = domeCommand("E#", sResp);
    if(nErr) {
        return nErr;
    }

    nAcceleration = atoi(szResp);
*/
 #ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getShutterAcceleration] nAcceleration =  %d\n", timestamp, nAcceleration);
    fflush(Logfile);
#endif
    return nErr;
}

int CLunaticoBeaver::setShutterAcceleration(int nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        return SB_OK;
    }

	
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "E%d#", nAcceleration);
    nErr = domeCommand(szBuf, sResp);
    return nErr;
}

void CLunaticoBeaver::setHomeOnPark(const bool bEnabled)
{
    m_bHomeOnPark = bEnabled;
}

void CLunaticoBeaver::setHomeOnUnpark(const bool bEnabled)
{
    m_bHomeOnUnpark = bEnabled;
}

int	CLunaticoBeaver::getSutterWatchdogTimerValue(int &nValue)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nValue = 0;
        return SB_OK;
    }

/*
	nErr = domeCommand("I#", sResp);
	if(nErr) {
		return nErr;
	}

	nValue = atoi(szResp)/1000; // value is in ms
*/
 #ifdef PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CLunaticoBeaver::getSutterWatchdogTimerValue] nValue =  %d\n", timestamp, nValue);
	fflush(Logfile);
#endif
	return nErr;
}

int	CLunaticoBeaver::setSutterWatchdogTimerValue(const int &nValue)
{
	int nErr = PLUGIN_OK;
	char szBuf[SERIAL_BUFFER_SIZE];
	std::string sResp;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        return SB_OK;
    }

	
	snprintf(szBuf, SERIAL_BUFFER_SIZE, "I%d#", nValue * 1000); // value is in ms
	nErr = domeCommand(szBuf, sResp);
	return nErr;
}

int CLunaticoBeaver::getRainAction(int &nAction)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
/*
    nErr = domeCommand("n#", sResp);
    if(nErr) {
        return nErr;
    }

    nAction = atoi(szResp);
*/

 #ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::getRainTimerValue] nValue =  %d\n", timestamp, nAction);
    fflush(Logfile);
#endif
    return nErr;

}

int CLunaticoBeaver::setRainAction(const int &nAction)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "n%d#", nAction);
    nErr = domeCommand(szBuf, sResp);
    return nErr;

}


void CLunaticoBeaver::enableRainStatusFile(bool bEnable)
{
    if(bEnable) {
        if(!RainStatusfile)
            RainStatusfile = fopen(m_sRainStatusfilePath.c_str(), "w");
        if(RainStatusfile) {
            m_bSaveRainStatus = true;
        }
        else { // if we failed to open the file.. don't log ..
            RainStatusfile = NULL;
            m_bSaveRainStatus = false;
        }
    }
    else {
        if(RainStatusfile) {
            fclose(RainStatusfile);
            RainStatusfile = NULL;
        }
        m_bSaveRainStatus = false;
    }
}

void CLunaticoBeaver::getRainStatusFileName(std::string &fName)
{
    fName.assign(m_sRainStatusfilePath);
}

void CLunaticoBeaver::writeRainStatus()
{
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CLunaticoBeaver::writeRainStatus] m_nIsRaining =  %s\n", timestamp, m_nIsRaining==RAINING?"Raining":"Not Raining");
    fprintf(Logfile, "[%s] [CLunaticoBeaver::writeRainStatus] m_bSaveRainStatus =  %s\n", timestamp, m_bSaveRainStatus?"YES":"NO");
    fflush(Logfile);
#endif

    if(m_bSaveRainStatus && RainStatusfile) {
        int nStatus;
        getRainSensorStatus(nStatus);
        fseek(RainStatusfile, 0, SEEK_SET);
        fprintf(RainStatusfile, "Raining:%s", nStatus == RAINING?"YES":"NO");
        fflush(RainStatusfile);
    }
}


int CLunaticoBeaver::parseFields(std::string sResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    std::string sTmp;
    
    sResp = trim(sResp,"!#\r\n");
    if(!sResp.size()) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::parseFields] pszResp is empty\n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    
    std::stringstream ssTmp(sResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CLunaticoBeaver::parseFields] no field found in '%s'\n", timestamp, sResp.c_str());
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}


std::string& CLunaticoBeaver::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& CLunaticoBeaver::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& CLunaticoBeaver::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}
