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
    m_bCalibratingShutter = false;
    m_nBattRequest = 0;
    m_bSettingPanID = false;
    m_bHasShutterControl = false;
    
    m_LunaticoBeaver.setSerxPointer(pSerX);
    m_LunaticoBeaver.setSleeprPinter(pSleeper);

    if (m_pIniUtil)
    {
        m_bLogRainStatus = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_LOG_RAIN_STATUS, false);
        m_LunaticoBeaver.enableRainStatusFile(m_bLogRainStatus);
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
    int n_nbStepPerRev;
    int nRainSensorStatus = NOT_RAINING;
    int nRMinSpeed;
    int nRMaxSpeed;
    int nRAcc;
    int nSMinSpeed;
    int nSMaxSpeed;
    int nSAcc;
    double  batShutCutOff;
    bool bShutterDetected;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("LunaticoBeaver.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());

    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);

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
        m_LunaticoBeaver.getShutterPresent(m_bHasShutterControl);
        dx->setEnabled("homePosition",true);
        dx->setEnabled("parkPosition",true);

        dx->setEnabled("checkBox_2",true);
        // set controls state depending on the connection state
        dx->setChecked("checkBox_2",m_bHasShutterControl?true:false);

        // read values from dome controller
        dx->setEnabled("ticksPerRev",true);
        n_nbStepPerRev = m_LunaticoBeaver.getDomeStepPerRev();
        dx->setPropertyInt("ticksPerRev","value", n_nbStepPerRev);

        m_LunaticoBeaver.getRotationSpeed(nRMinSpeed, nRMaxSpeed, nRAcc);

        dx->setEnabled("rotationMinSpeed",true);
        dx->setPropertyInt("rotationMinSpeed","value", nRMinSpeed);

        dx->setEnabled("rotationSpeed",true);
        dx->setPropertyInt("rotationSpeed","value", nRMaxSpeed);

        dx->setEnabled("rotationAcceletation",true);
        dx->setPropertyInt("rotationAcceletation","value", nRAcc);

        m_LunaticoBeaver.isShutterDetected(bShutterDetected);

        if(m_bHasShutterControl && bShutterDetected) {
            dx->setEnabled("pushButton_3", true);
            m_LunaticoBeaver.getShutterSpeed(nSMinSpeed, nSMaxSpeed, nSAcc);

            dx->setEnabled("shutterMinSpeed",true);
            dx->setPropertyInt("shutterMinSpeed","value", nSMinSpeed);

            dx->setEnabled("shutterSpeed",true);
            dx->setPropertyInt("shutterSpeed","value", nSMaxSpeed);

            dx->setEnabled("shutterAcceleration",true);
            dx->setPropertyInt("shutterAcceleration","value", nSAcc);

            dx->setEnabled("lowShutBatCutOff",true);
            dx->setText("shutterPresent", "<html><head/><body><p><span style=\" color:#00FF00;\">Detected</span></p></body></html>");

            m_LunaticoBeaver.getBatteryLevels( dShutterBattery, dShutterCutOff);
            dx->setPropertyDouble("lowShutBatCutOff", "value", dShutterCutOff);

            if(dShutterBattery>=0.0f)
                snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "%2.2f V",dShutterBattery);
            else
                snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "--");
            dx->setPropertyString("shutterBatteryLevel","text", szTmpBuf);

        } else {
            dx->setEnabled("shutterMinSpeed",false);
            dx->setPropertyInt("shutterMinSpeed","value",0);
            dx->setEnabled("shutterSpeed",false);
            dx->setPropertyInt("shutterSpeed","value",0);
            dx->setEnabled("shutterAcceleration",false);
            dx->setPropertyInt("shutterAcceleration","value",0);
            dx->setEnabled("lowShutBatCutOff",false);
            dx->setText("shutterPresent", "<html><head/><body><p><span style=\" color:#FF0000;\">Not detected</span></p></body></html>");
            dx->setPropertyDouble("lowShutBatCutOff","value", 0);
            dx->setPropertyString("shutterBatteryLevel","text", "--");
        }


        nErr = m_LunaticoBeaver.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            dx->setPropertyString("rainStatus","text", "--");
        else {
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, nRainSensorStatus==NOT_RAINING ? "<html><head/><body><p><span style=\" color:#00FF00;\">Not raining</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Raining</span></p></body></html>");
            dx->setPropertyString("rainStatus","text", szTmpBuf);
        }

        dx->setEnabled("pushButton",true);
    }
    else {
        dx->setEnabled("homePosition", false);
        dx->setEnabled("parkPosition", false);
        dx->setEnabled("checkBox_2",false);
        dx->setEnabled("ticksPerRev", false);
        dx->setEnabled("rotationMinSpeed", false);
        dx->setEnabled("rotationSpeed", false);
        dx->setEnabled("rotationAcceletation", false);
        dx->setEnabled("shutterMinSpeed", false);
        dx->setEnabled("shutterSpeed", false);
        dx->setEnabled("shutterAcceleration", false);
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
        dx->propertyInt("rotationMinSpeed", "value", nRMinSpeed);
        dx->propertyInt("rotationSpeed", "value", nRMaxSpeed);
        dx->propertyInt("rotationAcceletation", "value", nRAcc);
        dx->propertyInt("shutterMinSpeed", "value", nSMinSpeed);
        dx->propertyInt("shutterSpeed", "value", nSMaxSpeed);
        dx->propertyInt("shutterAcceleration", "value", nSAcc);
        dx->propertyDouble("lowShutBatCutOff", "value", batShutCutOff);
        m_bLogRainStatus = dx->isChecked("checkBox");
        m_LunaticoBeaver.enableRainStatusFile(m_bLogRainStatus);

        if(m_bLinked) {
            m_LunaticoBeaver.setHomeAz(dHomeAz);
            m_LunaticoBeaver.setParkAz(dParkAz);
            m_LunaticoBeaver.setDomeStepPerRev(n_nbStepPerRev);
            m_LunaticoBeaver.setRotationSpeed(nRMinSpeed, nRMaxSpeed, nRAcc);
			if(m_bHasShutterControl) {
				m_LunaticoBeaver.setShutterSpeed(nSMinSpeed, nSMaxSpeed, nSAcc);
                m_LunaticoBeaver.setBatteryCutOff(batShutCutOff);
			}
        }
        // save settings to eeprom
        m_LunaticoBeaver.saveSettingsToEEProm();
        // save the values to persistent storage
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
    int nMinSpeed;
    int nMaxSpeed;
    int nAcc;
    int nSMinSpeed;
    int nSMaxSpeed;
    int nSAcc;
    int nRainSensorStatus = NOT_RAINING;
    bool bShutterPresent = false;
    bool bShutterDetected = false;

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked") && (m_bCalibratingDome || m_bCalibratingShutter))
        m_LunaticoBeaver.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(!m_bCalibratingShutter && !m_bCalibratingDome) {
            m_bHasShutterControl = (uiex->isChecked("checkBox_2") ==1);
            if(m_bHasShutterControl)
                m_LunaticoBeaver.isShutterDetected(bShutterDetected);
            if(bShutterDetected) {
                uiex->setText("shutterPresent", "<html><head/><body><p><span style=\" color:#00FF00;\">Detected</span></p></body></html>");
                m_LunaticoBeaver.getShutterSpeed(nMinSpeed, nMaxSpeed, nAcc);
                uiex->setEnabled("shutterMinSpeed",true);
                uiex->setPropertyInt("shutterMinSpeed","value", nMinSpeed);
                uiex->setEnabled("shutterSpeed",true);
                uiex->setPropertyInt("shutterSpeed","value", nMaxSpeed);
                uiex->setEnabled("shutterAcceleration",true);
                uiex->setPropertyInt("shutterAcceleration","value", nAcc);

            }
            else {
                uiex->setText("shutterPresent", "<html><head/><body><p><span style=\" color:#FF0000;\">Not detected</span></p></body></html>");
                uiex->setPropertyInt("shutterMinSpeed","value", 0);
                uiex->setPropertyInt("shutterSpeed","value", 0);
                uiex->setPropertyInt("shutterAcceleration","value", 0);
                uiex->setEnabled("shutterMinSpeed",false);
                uiex->setEnabled("shutterSpeed",false);
                uiex->setEnabled("shutterAcceleration",false);
                uiex->setPropertyString("shutterBatteryLevel","text", "--");
            }
        }

        if(m_bLinked) {
           if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_LunaticoBeaver.isCalibratingDomeComplete(bComplete);
                if(nErr) {
                    if(m_nCalibratingError<4) { // this is to protect from the reboot
                        m_nCalibratingError++;
                    }
                    else {
                        uiex->setEnabled("pushButtonOK",true);
                        uiex->setEnabled("pushButtonCancel", true);
                        snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                        uiex->messageBox("Dome Calibrate", szErrorMessage);
                        m_bCalibratingDome = false;
                    }
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
                uiex->setPropertyInt("ticksPerRev","value", m_LunaticoBeaver.getDomeStepPerRev());
			}

            else if(m_bCalibratingShutter) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_LunaticoBeaver.isCalibratingShutterComplete(bComplete);
                if(nErr) {
                    if(m_nCalibratingError<4) { // this is to protect from the reboot
                        m_nCalibratingError++;
                    }
                    else {
                        uiex->setEnabled("pushButtonOK",true);
                        uiex->setEnabled("pushButtonCancel", true);
                        snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating shutter : Error %d", nErr);
                        uiex->messageBox("Shutter Calibrate", szErrorMessage);
                        m_bCalibratingShutter = false;
                    }
                    return;
                }

                if(!bComplete) {
                    return;
                }

                // enable buttons
                uiex->setEnabled("pushButtonOK",true);
                uiex->setEnabled("pushButtonCancel", true);
                m_bCalibratingShutter = false;
                uiex->setText("pushButton_3", "Calibrate");
            }

            else if(m_bHasShutterControl && !m_bCalibratingDome && !m_bCalibratingShutter) {
                // don't ask to often
                if (!(m_nBattRequest%4)) {
                    m_LunaticoBeaver.getBatteryLevels(dShutterBattery, dShutterCutOff);
                    if(dShutterCutOff < 1.0f) // not right.. ask again
                        m_LunaticoBeaver.getBatteryLevels(dShutterBattery, dShutterCutOff);
                        if(dShutterBattery>=0.0f)
                            snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "%2.2f V",dShutterBattery);
                        else
                            snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "--");
                        uiex->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
                        uiex->setPropertyDouble("lowShutBatCutOff","value", dShutterCutOff);
                }
                m_nBattRequest++;
                nErr = m_LunaticoBeaver.getRainSensorStatus(nRainSensorStatus);
                if(nErr)
                    uiex->setPropertyString("rainStatus","text", "--");
                else {
                    snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, nRainSensorStatus==NOT_RAINING ? "<html><head/><body><p><span style=\" color:#00FF00;\">Not raining</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Raining</span></p></body></html>");
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
				m_LunaticoBeaver.setDomeStepPerRev(m_nSavedTicksPerRev);
            } else { // Calibrate
                // disable buttons
                uiex->setEnabled("pushButtonOK", false);
                uiex->setEnabled("pushButtonCancel", false);
                uiex->setEnabled("pushButton_2", false);
                // change "Calibrate" to "Abort"
                uiex->setText("pushButton", "Abort");
				m_nSavedTicksPerRev = m_LunaticoBeaver.getDomeStepPerRev();
                m_LunaticoBeaver.calibrateDome();
                m_bCalibratingDome = true;
                m_nCalibratingError = 0;
            }
        }
    }


    if (!strcmp(pszEvent, "on_pushButton_3_clicked"))
    {
        if(m_bLinked) {
            if(m_bCalibratingShutter) { // Abort
                // enable buttons
                uiex->setEnabled("pushButtonOK", true);
                uiex->setEnabled("pushButtonCancel", true);
                // stop everything
                m_LunaticoBeaver.abortCurrentCommand();
                m_bCalibratingShutter = false;
                // set button text the Calibrate
                uiex->setText("pushButton_3", "Calibrate");
            } else { // Calibrate
                // disable buttons
                uiex->setEnabled("pushButtonOK", false);
                uiex->setEnabled("pushButtonCancel", false);
                // change "Calibrate" to "Abort"
                uiex->setText("pushButton_3", "Abort");
                m_LunaticoBeaver.calibrateShutter();
                m_bCalibratingShutter = true;
                m_nCalibratingError = 0;
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

    if (!strcmp(pszEvent, "on_checkBox_2_stateChanged")) {
        bShutterPresent = uiex->isChecked("checkBox_2");
        m_LunaticoBeaver.setShutterPresent(bShutterPresent);
        m_pSleeper->sleep(250);
        m_LunaticoBeaver.isShutterDetected(bShutterDetected);
        if(bShutterPresent && bShutterDetected) {
            uiex->setEnabled("pushButton_3", true);
            m_LunaticoBeaver.getShutterSpeed(nSMinSpeed, nSMaxSpeed, nSAcc);

            uiex->setEnabled("shutterMinSpeed",true);
            uiex->setPropertyInt("shutterMinSpeed","value", nSMinSpeed);

            uiex->setEnabled("shutterSpeed",true);
            uiex->setPropertyInt("shutterSpeed","value", nSMaxSpeed);

            uiex->setEnabled("shutterAcceleration",true);
            uiex->setPropertyInt("shutterAcceleration","value", nSAcc);

            uiex->setEnabled("lowShutBatCutOff",true);
            uiex->setText("shutterPresent", "<html><head/><body><p><span style=\" color:#00FF00;\">Detected</span></p></body></html>");

            m_LunaticoBeaver.getBatteryLevels( dShutterBattery, dShutterCutOff);
            uiex->setPropertyDouble("lowShutBatCutOff", "value", dShutterCutOff);

            if(dShutterBattery>=0.0f)
                snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "%2.2f V",dShutterBattery);
            else
                snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "--");
            uiex->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
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
    str = "Lunatico Beaver Dome Controller for NexDome";
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
    str = "Lunatico Beaver Dome Controller for NexDome";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = "Lunatico Beaver Dome Controller for NexDome X2 plugin by Rodolphe Pineau";
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



