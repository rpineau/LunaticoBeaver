#include "x2dome.h"


X2Dome::X2Dome(const char* pszSelection, const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyX,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			    pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyX			            = pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bCalibratingDome = false;
    m_nBattRequest = 0;
    m_bSettingPanID = false;
    m_bHasShutterControl = false;
    
    m_LunaticoBeaver.setSerxPointer(pSerX);
    m_LunaticoBeaver.setSleeprPinter(pSleeper);

    if (m_pIniUtil)
    {
        m_bLogRainStatus = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_LOG_RAIN_STATUS, false);
        m_bHomeOnPark = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_HOME_ON_PARK, false);
        m_bHomeOnUnpark = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_HOME_ON_UNPARK, false);
        m_LunaticoBeaver.setHomeOnPark(m_bHomeOnPark);
        m_LunaticoBeaver.setHomeOnUnpark(m_bHomeOnUnpark);
        m_LunaticoBeaver.enableRainStatusFile(m_bLogRainStatus);
        m_LunaticoBeaver.setShutterPresent(true);
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyX)
		delete m_pTheSkyX;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)
{
    int nErr;
    char szPort[SERIAL_BUFFER_SIZE];

    X2MutexLocker ml(GetMutex());

    m_bLinked = false;
    // get serial port device name
    portNameOnToCharPtr(szPort,SERIAL_BUFFER_SIZE);
    nErr = m_LunaticoBeaver.Connect(szPort);
    if(nErr) {
        return nErr;
    }

    m_bLinked = true;
    m_LunaticoBeaver.getShutterPresent(m_bHasShutterControl);
	return nErr;
}

int X2Dome::terminateLink(void)
{
    X2MutexLocker ml(GetMutex());

    m_LunaticoBeaver.Disconnect();
	m_bLinked = false;

    return SB_OK;
}

 bool X2Dome::isLinked(void) const
{
    return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);

    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    std::string fName;
    double dHomeAz;
    double dParkAz;
    double dShutterBattery, dShutterCutOff;
    bool nReverseDir;
    int n_nbStepPerRev;
    int nRainSensorStatus = NOT_RAINING;
    int nRSpeed;
    int nRAcc;
    int nSSpeed;
    int nSAcc;
	int nWatchdog;
    int nRainAction;
    double  batRotCutOff;
    double  batShutCutOff;
    
    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("LunaticoBeaver.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
    m_LunaticoBeaver.getShutterPresent(m_bHasShutterControl);

    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);
    // set controls state depending on the connection state
    if(m_bHasShutterControl) {
        dx->setChecked("hasShutterCtrl",true);
    }
    else {
        dx->setChecked("hasShutterCtrl",false);
    }

    if(m_bHomeOnPark) {
        dx->setChecked("homeOnPark",true);
    }
    else {
        dx->setChecked("homeOnPark",false);
    }

    if(m_bHomeOnUnpark) {
        dx->setChecked("homeOnUnpark",true);
    }
    else {
        dx->setChecked("homeOnUnpark",false);
    }

    if(m_bLogRainStatus) {
        dx->setChecked("checkBox",true);
        m_LunaticoBeaver.getRainStatusFileName(fName);
        dx->setPropertyString("filePath","text", fName.c_str());
    }
    else {
        dx->setChecked("checkBox",false);
        dx->setPropertyString("filePath","text", "");
    }

    if(m_bLinked) {
        dx->setEnabled("homePosition",true);
        dx->setEnabled("parkPosition",true);
        dx->setEnabled("needReverse",true);
        nErr = m_LunaticoBeaver.getDefaultDir(nReverseDir);
        if(nReverseDir)
            dx->setChecked("needReverse",false);
        else
            dx->setChecked("needReverse",true);

        // read values from dome controller
        dx->setEnabled("ticksPerRev",true);
        n_nbStepPerRev = m_LunaticoBeaver.getNbTicksPerRev();
        dx->setPropertyInt("ticksPerRev","value", n_nbStepPerRev);

        dx->setEnabled("rotationSpeed",true);
        m_LunaticoBeaver.getRotationSpeed(nRSpeed);
        dx->setPropertyInt("rotationSpeed","value", nRSpeed);

        dx->setEnabled("rotationAcceletation",true);
        m_LunaticoBeaver.getRotationAcceleration(nRAcc);
        dx->setPropertyInt("rotationAcceletation","value", nRAcc);

        dx->setEnabled("pushButton_3", true);

        if(m_bHasShutterControl) {
            dx->setEnabled("shutterSpeed",true);
            nErr = m_LunaticoBeaver.getShutterSpeed(nSSpeed);
            dx->setPropertyInt("shutterSpeed","value", nSSpeed);

            dx->setEnabled("shutterAcceleration",true);
            m_LunaticoBeaver.getShutterAcceleration(nSAcc);
            dx->setPropertyInt("shutterAcceleration","value", nSAcc);

            dx->setEnabled("pushButton_4", true);

            dx->setEnabled("shutterWatchdog",true);
            m_LunaticoBeaver.getSutterWatchdogTimerValue(nWatchdog);
            dx->setPropertyInt("shutterWatchdog", "value", nWatchdog);

            dx->setEnabled("lowShutBatCutOff",true);
        } else {
            dx->setEnabled("shutterSpeed",false);
            dx->setPropertyInt("shutterSpeed","value",0);
            dx->setEnabled("shutterAcceleration",false);
            dx->setPropertyInt("shutterAcceleration","value",0);
            dx->setEnabled("shutterWatchdog",false);
            dx->setPropertyInt("shutterWatchdog","value",0);
            dx->setEnabled("pushButton_4", false);
            dx->setPropertyInt("shutterWatchdog", "value", 0);
            dx->setEnabled("lowShutBatCutOff",false);
        }

        dx->setEnabled("lowRotBatCutOff",true);

        if(m_bHasShutterControl) {
            dx->setText("shutterPresent", "Shutter present");
        }
        else {
            dx->setText("shutterPresent", "No Shutter detected");
        }

        if(m_bHasShutterControl) {
            m_LunaticoBeaver.getBatteryLevels( dShutterBattery, dShutterCutOff);
            dx->setPropertyDouble("lowShutBatCutOff","value", dShutterCutOff);

            if(dShutterBattery>=0.0f)
                snprintf(szTmpBuf,16,"%2.2f V",dShutterBattery);
            else
                snprintf(szTmpBuf,16,"--");
            dx->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
        }
        else {
            dx->setPropertyDouble("lowShutBatCutOff","value", 0);
            dx->setPropertyString("shutterBatteryLevel","text", "--");
        }

        m_LunaticoBeaver.getRainAction(nRainAction);
        dx->setCurrentIndex("comboBox", nRainAction);
        
        nErr = m_LunaticoBeaver.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            dx->setPropertyString("rainStatus","text", "--");
        else {
            snprintf(szTmpBuf, 16, nRainSensorStatus==NOT_RAINING ? "Not raining" : "Raining");
            dx->setPropertyString("rainStatus","text", szTmpBuf);
        }

        dx->setEnabled("pushButton",true);
    }
    else {
        dx->setEnabled("homePosition", false);
        dx->setEnabled("parkPosition", false);
        dx->setEnabled("needReverse", false);
        dx->setEnabled("ticksPerRev", false);
        dx->setEnabled("rotationSpeed", false);
        dx->setEnabled("rotationAcceletation", false);
        dx->setEnabled("shutterSpeed", false);
        dx->setEnabled("shutterAcceleration", false);
		dx->setEnabled("shutterWatchdog", false);
        dx->setEnabled("lowRotBatCutOff", false);
        dx->setEnabled("lowShutBatCutOff", false);
        dx->setEnabled("comboBox", false);
        dx->setPropertyString("domeBatteryLevel", "text", "--");
        dx->setPropertyString("shutterBatteryLevel", "text", "--");
        dx->setEnabled("panID", false);
        dx->setEnabled("pushButton_2", false);
        dx->setEnabled("pushButton", false);
        dx->setEnabled("pushButton_3", false);
        dx->setEnabled("pushButton_4", false);
        dx->setPropertyString("domePointingError", "text", "--");
        dx->setPropertyString("rainStatus","text", "--");
    }
    dx->setPropertyDouble("homePosition","value", m_LunaticoBeaver.getHomeAz());
    dx->setPropertyDouble("parkPosition","value", m_LunaticoBeaver.getParkAz());


    m_nBattRequest = 0;

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        dx->propertyInt("ticksPerRev", "value", n_nbStepPerRev);
        dx->propertyDouble("homePosition", "value", dHomeAz);
        dx->propertyDouble("parkPosition", "value", dParkAz);
        dx->propertyInt("rotationSpeed", "value", nRSpeed);
        dx->propertyInt("rotationAcceletation", "value", nRAcc);
        dx->propertyInt("shutterSpeed", "value", nSSpeed);
        dx->propertyInt("shutterAcceleration", "value", nSAcc);
		dx->propertyInt("shutterWatchdog", "value", nWatchdog);
        dx->propertyDouble("lowRotBatCutOff", "value", batRotCutOff);
        dx->propertyDouble("lowShutBatCutOff", "value", batShutCutOff);
        nRainAction = dx->currentIndex("comboBox");
        m_bHomeOnPark = dx->isChecked("homeOnPark");
        m_LunaticoBeaver.setHomeOnPark(m_bHomeOnPark);
        m_bHomeOnUnpark = dx->isChecked("homeOnUnpark");
        m_LunaticoBeaver.setHomeOnUnpark(m_bHomeOnUnpark);
        nReverseDir = dx->isChecked("needReverse");
        m_bLogRainStatus = dx->isChecked("checkBox");
        m_LunaticoBeaver.enableRainStatusFile(m_bLogRainStatus);

        if(m_bLinked) {
            m_LunaticoBeaver.setDefaultDir(!nReverseDir);
            m_LunaticoBeaver.setHomeAz(dHomeAz);
            m_LunaticoBeaver.setParkAz(dParkAz);
            m_LunaticoBeaver.setNbTicksPerRev(n_nbStepPerRev);
            m_LunaticoBeaver.setRotationSpeed(nRSpeed);
            m_LunaticoBeaver.setRotationAcceleration(nRAcc);
			// m_LunaticoBeaver.setBatteryCutOff(batRotCutOff, batShutCutOff);
            m_LunaticoBeaver.setRainAction(nRainAction);
			if(m_bHasShutterControl) {
				m_LunaticoBeaver.setShutterSpeed(nSSpeed);
				m_LunaticoBeaver.setShutterAcceleration(nSAcc);
				m_LunaticoBeaver.setSutterWatchdogTimerValue(nWatchdog);
			}
        }

        // save the values to persistent storage
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_HOME_ON_PARK, m_bHomeOnPark);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_HOME_ON_UNPARK, m_bHomeOnUnpark);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_LOG_RAIN_STATUS, m_bLogRainStatus);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr;
    double dShutterBattery, dShutterCutOff;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    char szErrorMessage[LOG_BUFFER_SIZE];
    std::string fName;
    int nRainSensorStatus = NOT_RAINING;
    bool bShutterPresent;
    int nSpeed;
    int nAcc;
    int nWatchdog;

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked") && m_bCalibratingDome)
        m_LunaticoBeaver.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        bShutterPresent = uiex->isChecked("hasShutterCtrl");
        m_LunaticoBeaver.getShutterPresent(bShutterPresent);

        if(bShutterPresent != m_bHasShutterControl) {
            m_bHasShutterControl = bShutterPresent;
            if(m_bHasShutterControl && m_bLinked) {
                uiex->setText("shutterPresent", "Shutter present");
                uiex->setEnabled("shutterSpeed",true);
                m_LunaticoBeaver.getShutterSpeed(nSpeed);
                uiex->setPropertyInt("shutterSpeed","value", nSpeed);

                uiex->setEnabled("shutterAcceleration",true);
                m_LunaticoBeaver.getShutterAcceleration(nAcc);
                uiex->setPropertyInt("shutterAcceleration","value", nAcc);

                uiex->setEnabled("shutterWatchdog",true);
                m_LunaticoBeaver.getSutterWatchdogTimerValue(nWatchdog);
                uiex->setPropertyInt("shutterWatchdog", "value", nWatchdog);
            }
            else {
                uiex->setText("shutterPresent", "No Shutter detected");
                uiex->setPropertyInt("shutterSpeed","value", 0);
                uiex->setPropertyInt("shutterAcceleration","value", 0);
                uiex->setPropertyInt("shutterWatchdog", "value", 0);
                uiex->setEnabled("shutterSpeed",false);
                uiex->setEnabled("shutterAcceleration",false);
                uiex->setEnabled("shutterWatchdog",false);
                uiex->setPropertyString("shutterBatteryLevel","text", "--");

            }

        }
        if(m_bLinked) {
           if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_LunaticoBeaver.isCalibratingComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled("pushButtonOK",true);
					uiex->setEnabled("pushButtonCancel", true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                    uiex->messageBox("Dome Calibrate", szErrorMessage);
                    m_bCalibratingDome = false;
                    return;
                }

                if(!bComplete) {
                    return;
                }

                // enable buttons
                uiex->setEnabled("pushButtonOK",true);
				uiex->setEnabled("pushButtonCancel", true);
				m_bCalibratingDome = false;
				uiex->setText("pushButton", "Calibrate");
                uiex->setEnabled("pushButton_2", true);
                // read step per rev from controller
                uiex->setPropertyInt("ticksPerRev","value", m_LunaticoBeaver.getNbTicksPerRev());
			}

            else if(m_bHasShutterControl && !m_bCalibratingDome) {
                // don't ask to often
                if (m_bHasShutterControl && !(m_nBattRequest%4)) {
                    m_LunaticoBeaver.getBatteryLevels(dShutterBattery, dShutterCutOff);
                    if(dShutterCutOff < 1.0f) // not right.. ask again
                        m_LunaticoBeaver.getBatteryLevels(dShutterBattery, dShutterCutOff);
                        if(dShutterBattery>=0.0f)
                            snprintf(szTmpBuf,16,"%2.2f V",dShutterBattery);
                        else
                            snprintf(szTmpBuf,16,"--");
                        uiex->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
                        uiex->setPropertyDouble("lowShutBatCutOff","value", dShutterCutOff);
                }
                m_nBattRequest++;
                nErr = m_LunaticoBeaver.getRainSensorStatus(nRainSensorStatus);
                if(nErr)
                    uiex->setPropertyString("rainStatus","text", "--");
                else {
                    snprintf(szTmpBuf, 16, nRainSensorStatus==NOT_RAINING ? "Not raining" : "Raining");
                    uiex->setPropertyString("rainStatus","text", szTmpBuf);
                }

            }
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_clicked"))
    {
        if(m_bLinked) {
            if( m_bCalibratingDome) { // Abort
                // enable buttons
                uiex->setEnabled("pushButtonOK", true);
                uiex->setEnabled("pushButtonCancel", true);
                uiex->setEnabled("pushButton_2", true);
                // stop everything
                m_LunaticoBeaver.abortCurrentCommand();
                m_bCalibratingDome = false;
                // set button text the Calibrate
                uiex->setText("pushButton", "Calibrate");
				// restore saved ticks per rev
				uiex->setPropertyInt("ticksPerRev","value", m_nSavedTicksPerRev);
				m_LunaticoBeaver.setNbTicksPerRev(m_nSavedTicksPerRev);
            } else {								// Calibrate
                // disable buttons
                uiex->setEnabled("pushButtonOK", false);
                uiex->setEnabled("pushButtonCancel", false);
                uiex->setEnabled("pushButton_2", false);
                // change "Calibrate" to "Abort"
                uiex->setText("pushButton", "Abort");
				m_nSavedTicksPerRev = m_LunaticoBeaver.getNbTicksPerRev();
                m_LunaticoBeaver.calibrate();
                m_bCalibratingDome = true;
            }
        }
    }
    
    if (!strcmp(pszEvent, "on_checkBox_stateChanged"))
    {
        m_bLogRainStatus = uiex->isChecked("checkBox");
        if(m_bLogRainStatus) {
            m_LunaticoBeaver.getRainStatusFileName(fName);
            uiex->setPropertyString("filePath","text", fName.c_str());
        }
        else {
            uiex->setPropertyString("filePath","text", "");
        }
    }

}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const
{
	str = "Lunatico Beaver";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const
{
    str = "Lunatico Beaver";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
    str = "Lunatico Beaver Dome Controller for NexDome V4";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)
{

    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
		X2MutexLocker ml(GetMutex());
        m_LunaticoBeaver.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    str = "Lunatico Beaver Dome Controller for NexDome V4";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = "Lunatico Beaver Dome Controller for NexDome V4 X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    *pdAz = m_LunaticoBeaver.getCurrentAz();
    *pdEl = m_LunaticoBeaver.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_LunaticoBeaver.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{
    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    m_LunaticoBeaver.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    m_LunaticoBeaver.getShutterPresent(m_bHasShutterControl);
    
	if(!m_bHasShutterControl)
        return SB_OK;


    nErr = m_LunaticoBeaver.openShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    m_LunaticoBeaver.getShutterPresent(m_bHasShutterControl);

    if(!m_bHasShutterControl)
        return SB_OK;


    nErr = m_LunaticoBeaver.closeShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_LunaticoBeaver.parkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_LunaticoBeaver.unparkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.goHome();
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isGoToComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
    {
        *pbComplete = true;
        return SB_OK;
    }

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
    {
        *pbComplete = false;    // it can't be open and closed at the same time :)
        return SB_OK;
    }

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isParkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isUnparkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_LunaticoBeaver.syncDome(dAz, dEl);
    if(nErr)
        return ERR_CMDFAILED;
	return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[SERIAL_BUFFER_SIZE];

    portNameOnToCharPtr(szPortName, SERIAL_BUFFER_SIZE);

    str = szPortName;

}

void X2Dome::setPortName(const char* szPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, szPort);

}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);

}



