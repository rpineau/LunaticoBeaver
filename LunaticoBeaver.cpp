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

    m_dStepsPerDeg = 0;
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

    m_nHomingTries = 0;
    m_nGotoTries = 0;

    m_nRainSensorstate = NOT_RAINING;
    m_nRainStatus = RAIN_UNNOWN;
    m_bSaveRainStatus = false;
    m_cRainCheckTimer.Reset();

    m_bHomeOnPark = false;
    m_bHomeOnUnpark = false;

    m_bShutterPresent = false;

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
    m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CLunaticoBeaver] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CLunaticoBeaver] Constructor Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CLunaticoBeaver] Rains status file : " << m_sRainStatusfilePath<<std::endl;
    m_sLogFile.flush();
#endif

}

CLunaticoBeaver::~CLunaticoBeaver()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if(m_sLogFile.is_open())
        m_sLogFile.close();
#endif
}

int CLunaticoBeaver::Connect(const char *pszPort)
{
    int nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connect Called." << std::endl;
    m_sLogFile.flush();
#endif
    m_bIsConnected = false;
    m_bCalibrating = false;
    m_bUnParking = false;

    // 115200 8N1
    nErr = m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY);
    if(nErr) {
        return nErr;
    }
    m_bIsConnected = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] connected to " << pszPort << std::endl;
    m_sLogFile.flush();
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Getting Firmware." << std::endl;
    m_sLogFile.flush();
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_sFirmwareVersion);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Error getting Firmware : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }
    nErr = getDomeParkAz(m_dCurrentAzPosition);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Error getDomeParkAz : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    nErr = getDomeHomeAz(m_dHomeAz);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Error getDomeHomeAz : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    getShutterPresent(m_bShutterPresent);

    writeRainStatus();
    m_cRainCheckTimer.Reset();

    setMaxRotationTime(300);
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] Error m_bIsConnected : " << (m_bIsConnected?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif
}


int CLunaticoBeaver::domeCommand(const std::string sCmd, std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] sending : " << sCmd << std::endl;
    m_sLogFile.flush();
#endif

    nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    nErr = readResponse(sResp, nTimeout);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] ***** ERROR READING RESPONSE **** error = " << nErr << " , response : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] response : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::shutterCommand(const std::string sCmd, std::string &sResp, int nTimeout)
{
    std::string newCmd;

    newCmd="!dome sendtoshutter \""+sCmd+"\"#";
    return domeCommand(newCmd, sResp, nTimeout);
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

    sResp.clear();
    memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting      : " << nBytesWaiting << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting nErr : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        if(!nBytesWaiting) {
            nbTimeouts += MAX_READ_WAIT_TIMEOUT;
            if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] bytesWaitingRx timeout, no data for " << nbTimeouts << " ms"<< std::endl;
                m_sLogFile.flush();
#endif
                nErr = COMMAND_TIMEOUT;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
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
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile error : " << nErr << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] rreadFile Timeout Error." << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile nBytesWaiting : " << nBytesWaiting << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile ulBytesRead   : " << ulBytesRead << std::endl;
            m_sLogFile.flush();
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    }  while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != '#');

    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else
        *(pszBufPtr-1) = 0; //remove the #

    sResp.assign(pszBuf);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
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

    nErr = domeCommand("!dome getaz#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAz] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            dDomeAz = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAz] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
        m_dCurrentAzPosition = dDomeAz;
    }
    if(m_cRainCheckTimer.GetElapsedSeconds() > RAIN_CHECK_INTERVAL) {
        writeRainStatus();
        m_cRainCheckTimer.Reset();
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAz] Dome Az  : " << std::fixed << std::setprecision(2) << m_dParkAz << std::endl;
    m_sLogFile.flush();
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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeAz] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            dAz = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeAz] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    m_dHomeAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeAz] Dome Az  : " << std::fixed << std::setprecision(2) << m_dHomeAz << std::endl;
    m_sLogFile.flush();
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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            dAz = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }
    m_dParkAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] Dome Az  : " << std::fixed << std::setprecision(2) << m_dHomeAz << std::endl;
    m_sLogFile.flush();
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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterState] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterState] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    parseFields(sResp, shutterStateFileds, ':');
    if(shutterStateFileds.size()>=2) {
        try {
            nState = std::stoi(shutterStateFileds[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterState] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterState] nState : " << nState << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CLunaticoBeaver::getBatteryLevels(double &dShutterVolts, double &dShutterCutOff)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    dShutterVolts  = 0;
    dShutterCutOff = 0;
    if(m_bShutterPresent) {
        nErr = shutterCommand("shutter getvoltage", sResp);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }
        
        parseFields(sResp, svFields, ':');
        if(svFields.size()>=2) {
            try {
                dShutterVolts = std::stof(svFields[1]);
            }
            catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] conversion exception : " << e.what() << std::endl;
                m_sLogFile.flush();
#endif
                return ERR_CMDFAILED;
            }

        }

        nErr = shutterCommand("shutter getsafevoltage", sResp);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }
        
        parseFields(sResp, svFields, ':');
        if(svFields.size()>=2) {
            try {
                dShutterCutOff = std::stof(svFields[1]);
            }
            catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] conversion exception : " << e.what() << std::endl;
                m_sLogFile.flush();
#endif
                return ERR_CMDFAILED;
            }
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] dShutterVolts  : " << std::fixed << std::setprecision(2) << dShutterVolts << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] dShutterCutOff : " << std::fixed << std::setprecision(2) << dShutterCutOff << std::endl;
        m_sLogFile.flush();
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
    
    ssTmp<<"shutter setsafevoltage " << dShutterCutOff;
    nErr = shutterCommand(ssTmp.str(), sResp);
    return nErr;
}

bool CLunaticoBeaver::isDomeMoving()
{
    bool bIsMoving = false;
    int nTmp;
    
    getDomeStatus(nTmp);

    bIsMoving = ((nTmp & DOME_MOVING) == DOME_MOVING);

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

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!dome status#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    // need to parse sResp
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=1) {
        try {
            nStatus = std::stoi(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    m_nDomeRotStatus = nStatus & DOME_STATUS_MASK;
//    m_nShutStatus = nStatus & SHUTTER_STATUS_MASK;
    m_nRainSensorstate = ((nStatus & RAIN_SENSOR_MASK) != 0 ? RAINING : NOT_RAINING);
    
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] nStatus            : " << nStatus << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] m_nDomeRotStatus   : " << m_nDomeRotStatus << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] m_nRainSensorstate : " << m_nRainSensorstate << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStatus] m_nRainSensorstate : " << (m_nRainSensorstate==RAINING?"Raining":"Not Raining") << std::endl;
    m_sLogFile.flush();
#endif
    
    return nErr;
}

int CLunaticoBeaver::setMaxRotationTime(int nSeconds)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!domerot setmaxfullrotsecs 300#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setMaxRotationTime] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return false;
    }

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

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!dome athome#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return false;
    }

    bAthome = false;
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=1) {
        try {
            nTmp = std::stoi(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }
    else
        nTmp = 0;

    if(nTmp == AT_HOME)
        bAthome = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] nTmp    : " << m_nDomeRotStatus << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] bAthome : " << (bAthome?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    return bAthome;

}

int CLunaticoBeaver::syncDome(double dAz, double dEl)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    m_dCurrentAzPosition = dAz;
    ssTmp << "!dome setaz " << std::fixed << std::setprecision(2) << dAz << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDome] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
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

    if(m_bCalibrating)
        return nErr;

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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unparkDome] m_dParkAz : " << std::fixed << std::setprecision(2) << m_dParkAz << std::endl;
        m_sLogFile.flush();
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
    std::string sResp;
    std::stringstream ssTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

    ssTmp<<"!dome gotoaz " << dNewAz << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoAzimuth] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
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
    std::string sResp;
    double dShutterVolts;
    double dShutterCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openShutter] m_bShutterPresent : " << (m_bShutterPresent?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif
    if(!m_bShutterPresent) {
        return SB_OK;
    }

    getBatteryLevels(dShutterVolts, dShutterCutOff);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openShutter] Opening shutter." << std::endl;
    m_sLogFile.flush();
#endif

	
    nErr = domeCommand("!dome openshutter#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openShutter] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}

int CLunaticoBeaver::closeShutter()
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    double dShutterVolts;
    double dShutterCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeShutter] m_bShutterPresent : " << (m_bShutterPresent?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bShutterPresent) {
        return SB_OK;
    }

    getBatteryLevels(dShutterVolts, dShutterCutOff);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeShutter] Closing shutter." << std::endl;
    m_sLogFile.flush();
#endif

	
    nErr = domeCommand("!dome closeshutter#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeShutter] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
    }

    return nErr;
}

int CLunaticoBeaver::getFirmwareVersion(std::string &sVersion)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    sVersion.clear();
    nErr = domeCommand("!seletek version#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    nErr = parseFields(sResp, firmwareFields, ':');
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] parsing error : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return ERR_CMDFAILED;
    }

    if(firmwareFields.size()>=2) {
        std::stringstream ssTmp;
        if(firmwareFields[1].size()>=3) {
            ssTmp << firmwareFields[1].at(1) << "." << firmwareFields[1].at(2) << "." << firmwareFields[1].at(3);
            sVersion.assign(ssTmp.str());
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] firmware : " << sVersion << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CLunaticoBeaver::getShutterFirmwareVersion(std::string &sVersion)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    sVersion.clear();
    nErr = shutterCommand("seletek version", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterFirmwareVersion] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
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
            sVersion.assign(ssTmp.str());
        }
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterFirmwareVersion] Shutter firmware : " << sVersion << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CLunaticoBeaver::goHome()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    if(isDomeAtHome()){
            return PLUGIN_OK;
    }
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goHome]" << std::endl;
    m_sLogFile.flush();
#endif

    m_nHomingTries = 0;
    nErr = domeCommand("!dome gohome 300#", sResp);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goHome] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    return nErr;
}

int CLunaticoBeaver::calibrateDome()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!domerot calibrate 2 300#", sResp); // 5 minute timeout .. to be on the safe side
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [calibrate] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    m_bCalibrating = true;

    return nErr;
}

int CLunaticoBeaver::calibrateShutter()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!dome autocalshutter#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [calibrateShutter] ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete]" << std::endl;
    m_sLogFile.flush();
#endif

    bComplete = false;
    if(isDomeMoving()) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Dome is still moving" << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] bComplete : " << (bComplete?"True":"False") << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    getDomeAz(dDomeAz);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] dDomeAz : "  << std::fixed << std::setprecision(2) << dDomeAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] m_dGotoAz : "  << std::fixed << std::setprecision(2) << m_dGotoAz << std::endl;
    m_sLogFile.flush();
#endif

    if(checkBoundaries(m_dGotoAz, dDomeAz)) {
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete]  ***** ERROR **** domeAz =  dDomeAz : "  << std::fixed << std::setprecision(2) << dDomeAz << " , m_dGotoAz : "  << std::fixed << std::setprecision(2) << m_dGotoAz << std::endl;
        m_sLogFile.flush();
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] bComplete : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

bool CLunaticoBeaver::checkBoundaries(double dGotoAz, double dDomeAz)
{
    double highMark;
    double lowMark;
    double roundedGotoAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [checkBoundaries]" << std::endl;
    m_sLogFile.flush();
#endif

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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isOpenComplete] bComplete : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::isCloseComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCloseComplete] bComplete : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CLunaticoBeaver::isParkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz=0;
    bool bFoundHome;
    std::string sResp;

    if(m_bCalibrating)
        return nErr;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] m_bParking : " << (m_bParking?"True":"False") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] bComplete  : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
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
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] found home, now parking." << std::endl;
            m_sLogFile.flush();
#endif
            m_bParking = false;
            nErr = domeCommand("!dome gopark#", sResp);
        }
        return nErr;
    }

    getDomeAz(dDomeAz);

    if(checkBoundaries(m_dParkAz, dDomeAz)) {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] bComplete  : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::isUnparkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    if(!m_bParked) {
        bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] Unparked." << std::endl;
        m_sLogFile.flush();
#endif
    }
    else if (m_bUnParking) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] unparking.. checking if we're home." << std::endl;
        m_sLogFile.flush();
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] m_bParked : " << (m_bParked?"True":"False") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] bComplete : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::isFindHomeComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isFindHomeComplete]" << std::endl;
    m_sLogFile.flush();
#endif

    if(isDomeMoving()) {
        bComplete = false;
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isFindHomeComplete] still moving." << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;

    }

    if(isDomeAtHome()){
        bComplete = true;
        if(m_bUnParking)
            m_bParked = false;
        syncDome(m_dHomeAz, m_dCurrentElPosition);
        m_nHomingTries = 0;
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isFindHomeComplete] At Home." << std::endl;
        m_sLogFile.flush();
#endif
    }
    else {
        // we're not moving and we're not at the home position !!!
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isFindHomeComplete] Not moving and not at home !!!" << std::endl;
        m_sLogFile.flush();
#endif
        bComplete = false;
        m_bParked = false;
        // so give it another try
        if(m_nHomingTries == 0) {
            m_nHomingTries = 1;
            goHome();
        }
        return ERR_CMDFAILED;
    }

    return nErr;
}


int CLunaticoBeaver::isCalibratingDomeComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    int nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bComplete = false;
    // check if calibration is done : !domerot getcalibrationstatus#

    nErr = domeCommand("!domerot getcalibrationstatus#", sResp);
    if(nErr) {
        return ERR_CMDFAILED;
    }

    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nTmp = std::stoi(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingDomeComplete] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
        switch(nTmp) {
            case 0:
                bComplete = true;
                break;
            case 1:
                bComplete = false;
                break;
            case 2:
                bComplete = true;
                break;
            default:
                bComplete = false;
                nErr = ERR_CMDFAILED;
        }
    }

    if(bComplete) {
        m_bCalibrating = false;
        nErr = getDomeStepPerDeg(m_dStepsPerDeg);
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingDomeComplete] final m_dStepsPerDeg  : "  << std::fixed << std::setprecision(2) << m_dStepsPerDeg << std::endl;
        m_sLogFile.flush();
#endif
    }
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingDomeComplete] final m_bCalibrating  : " << (m_bCalibrating?"True":"False") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingDomeComplete] final bComplete       : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;
}

int CLunaticoBeaver::isCalibratingShutterComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;
    int nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bComplete = false;
    // check if calibration is done : !shutter getcalibrationstatus#

    nErr = shutterCommand("shutter getcalibrationstatus", sResp);
    if(nErr) {
        return ERR_CMDFAILED;
    }

    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nTmp = std::stoi(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingShutterComplete] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
        switch(nTmp) {
            case 0:
                bComplete = true;
                break;
            case 1:
                bComplete = false;
                break;
            case 2:
                bComplete = true;
                break;
            default:
                bComplete = false;
                nErr = ERR_CMDFAILED;
        }
    }

    if(bComplete)
        m_bCalibrating = false;

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingShutterComplete] final m_bCalibrating  : " << (m_bCalibrating?"True":"False") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCalibratingShutterComplete] final bComplete       : " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;
}


int CLunaticoBeaver::abortCurrentCommand()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

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

    bShutterPresent = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!dome getshutterenable#", sResp);
    if(nErr) {
        return nErr;
    }

    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            bShutterPresent = (std::stoi(svFields[1])==1);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterPresent] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
        m_bShutterPresent = bShutterPresent;
    }

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterPresent] sResp             : " << sResp << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterPresent] m_bShutterPresent : " << (m_bShutterPresent?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif


    bShutterPresent = m_bShutterPresent;
    return nErr;

}

int CLunaticoBeaver::setShutterPresent(bool bShutterPresent)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(!m_bIsConnected) {
        return NOT_CONNECTED;
    }

    ssTmp<<"!dome setshutterenable " << (bShutterPresent?"1":"0") << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    m_bShutterPresent = bShutterPresent;
    return nErr;
}

int CLunaticoBeaver::saveSettingsToEEProm()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = domeCommand("!seletek savefs#", sResp);
    return nErr;
}

int CLunaticoBeaver::isShutterDetected(bool &bDetected)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;

    bDetected = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = shutterCommand("!seletek version#", sResp);
    if(nErr)
        return nErr;
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] szFirmware : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    nErr = parseFields(sResp, firmwareFields, ':');
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] parsing error : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return ERR_CMDFAILED;
    }

    if(firmwareFields.size()>=2) {
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] sResp.size()          : " << sResp.size() << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] sResp                 : " << sResp << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] sResp.find(\"error\") : " << sResp.find("error") << std::endl;
        m_sLogFile.flush();
#endif
        if(firmwareFields[1].size()>5 && firmwareFields[1].find("error") == 0) {
            bDetected = false;
        }
        else {
            bDetected = true;
        }
    }


#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isShutterDetected] bDetected : " << (bDetected?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;

}
#pragma mark - Getter / Setter

int CLunaticoBeaver::getDomeStepPerRev()
{
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getNbTicksPerRev] m_bIsConnected : " << (m_bIsConnected?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bIsConnected) {
        getDomeStepPerDeg(m_dStepsPerDeg);
        m_nNbStepPerRev = int(m_dStepsPerDeg*360.0);
    }
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getNbTicksPerRev] m_nNbStepPerRev : " << m_nNbStepPerRev << std::endl;
    m_sLogFile.flush();
#endif

    return m_nNbStepPerRev;
}


int CLunaticoBeaver::setDomeStepPerRev(int nSteps)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;
    double dStepPerDeg = 0;

    if(m_bCalibrating)
        return nErr;

    dStepPerDeg = float(nSteps)/360.0;

    ssTmp << "!domerot setstepsperdegree " << std::fixed << std::setprecision(6) << dStepPerDeg << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;
    m_nNbStepPerRev = nSteps;
    return nErr;
}

int CLunaticoBeaver::getDomeStepPerDeg(double &dStepsPerDeg)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    
    dStepsPerDeg = 0;
    nErr = domeCommand("!domerot getstepsperdegree#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStepPerDeg] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            dStepsPerDeg = std::stof(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeStepPerDeg] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    m_dStepsPerDeg = dStepsPerDeg;
    return nErr;
}

int CLunaticoBeaver::setDomeStepPerDeg(double dStepsPerDeg)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    m_dStepsPerDeg = dStepsPerDeg;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    ssTmp<<"!domerot setstepsperdegree " << dStepsPerDeg <<"#";
    nErr = domeCommand(ssTmp.str(), sResp);
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
    std::string sResp;
    std::stringstream ssTmp;

    m_dHomeAz = dAz;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssTmp << "!domerot sethome " << std::fixed << std::setprecision(2) << dAz << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
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
    std::stringstream ssTmp;
    std::string sResp;

    m_dParkAz = dAz;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssTmp << "!domerot setpark " << std::fixed << std::setprecision(2) << dAz << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
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


int CLunaticoBeaver::getRainSensorStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    int nTmp;

    nStatus = NOT_RAINING;
    nErr = getDomeStatus(nTmp);
    if(nErr)
        return nErr;

    nStatus = m_nRainSensorstate;

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRainSensorStatus] nStatus : " << (m_nRainSensorstate==RAINING?"Raining":"Not Raining") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::getRotationSpeed(int &nMinSpeed, int &nMaxSpeed, int &nAccel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("!domerot getminspeed#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] getminspeed ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nMinSpeed = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    nErr = domeCommand("!domerot getmaxspeed#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] getmaxspeed ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();

#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nMaxSpeed = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    nErr = domeCommand("!domerot getacceleration#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] getacceleration ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nAccel = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }


#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] nMinSpeed : " << nMinSpeed << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] nMaxSpeed : " << nMaxSpeed << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRotationSpeed] nAccel    : " << nAccel << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CLunaticoBeaver::setRotationSpeed(int nMinSpeed, int nMaxSpeed, int nAccel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssTmp<<"!domerot setminspeed " << nMinSpeed << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;
    std::stringstream().swap(ssTmp);
    ssTmp<<"!domerot setmaxspeed " << nMaxSpeed << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    std::stringstream().swap(ssTmp);
    ssTmp<<"!domerot setacceleration " << nAccel << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    return nErr;
}


int CLunaticoBeaver::getShutterSpeed(int &nMinSpeed, int &nMaxSpeed, int &nAccel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("!dome getshutterminspeed#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] getshutterminspeed ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nMinSpeed = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    nErr = domeCommand("!dome getshuttermaxspeed#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] getshuttermaxspeed ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nMaxSpeed = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }

    nErr = domeCommand("!dome getshutteracceleration#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] getshutteracceleration ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // convert Az string to double
    parseFields(sResp, svFields, ':');
    if(svFields.size()>=2) {
        try {
            nAccel = std::stod(svFields[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_CMDFAILED;
        }
    }


#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] nMinSpeed : " << nMinSpeed << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] nMaxSpeed : " << nMaxSpeed << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterSpeed] nAccel    : " << nAccel << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CLunaticoBeaver::setShutterSpeed(int nMinSpeed, int nMaxSpeed, int nAccel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssTmp<<"!dome setshutterminspeed " << nMinSpeed << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    std::stringstream().swap(ssTmp);
    ssTmp<<"!dome setshuttermaxspeed " << nMaxSpeed << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    std::stringstream().swap(ssTmp);
    ssTmp<<"!dome setshutteracceleration " << nAccel << "#";
    nErr = domeCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    return nErr;
}


void CLunaticoBeaver::enableRainStatusFile(bool bEnable)
{
    m_bSaveRainStatus = bEnable;
}

void CLunaticoBeaver::getRainStatusFileName(std::string &fName)
{
    fName.assign(m_sRainStatusfilePath);
}

void CLunaticoBeaver::writeRainStatus()
{
    int nStatus;

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus]" << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] m_bSaveRainStatus : " << (m_bSaveRainStatus?"YES":"NO") << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bSaveRainStatus) {
        getRainSensorStatus(nStatus);
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] m_nRainStatus      : " << m_nRainStatus << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] m_nRainSensorstate : " << m_nRainSensorstate << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] nStatus            : " << nStatus << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] nStatus            : " << (nStatus==RAINING?"Raining":"Not Raining") << std::endl;
        m_sLogFile.flush();
#endif
        if(m_nRainStatus != nStatus) {
#ifdef PLUGIN_DEBUG
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] state changed, wrinting new status : " << (nStatus==RAINING?"Raining":"Not Raining") << std::endl;
            m_sLogFile.flush();
#endif
            m_nRainStatus = nStatus;
            if(m_RainStatusfile.is_open())
                m_RainStatusfile.close();
            try {
                m_RainStatusfile.open(m_sRainStatusfilePath, std::ios::out |std::ios::trunc);
                if(m_RainStatusfile.is_open()) {
                    m_RainStatusfile << "Raining:" << (nStatus == RAINING?"YES":"NO") << std::endl;
                    m_RainStatusfile.close();
                }
            }
            catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] Error writing file = " << e.what() << std::endl;
                m_sLogFile.flush();
#endif
                if(m_RainStatusfile.is_open())
                    m_RainStatusfile.close();
            }
        }
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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] sResp is empty." << std::endl;
        m_sLogFile.flush();
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
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] no field found in : '" << sResp << "'" << std::endl;
        m_sLogFile.flush();
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

#ifdef PLUGIN_DEBUG
const std::string CLunaticoBeaver::getTimeStamp()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
#endif
