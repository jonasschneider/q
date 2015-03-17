// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
uint16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SERIAL_RX)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
  uint8_t  spekFrameDone;
#endif

#if defined(OPENLRSv2MULTI)
  uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif










// in tick()
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos


  static uint16_t rcTime  = 0;
  static int16_t initialThrottleHold;
  int16_t rc;
  int32_t prop = 0;

    rcTime = currentTime + 20000;
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = conf.failsafe_throttle;
        if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeEvents++;
      }
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeCnt++;
    #endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for(i=0;i<4;i++) {
      stTmp >>= 2;
      if(rcData[i] > MINCHECK) stTmp |= 0x80;      // check for MIN
      if(rcData[i] < MAXCHECK) stTmp |= 0x40;      // check for MAX
    }
    if(stTmp == rcSticks) {
      if(rcDelayCommand<250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
      #if !defined(FIXEDWING)
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        #if PID_CONTROLLER == 1
          errorGyroI_YAW = 0;
        #elif PID_CONTROLLER == 2
          errorGyroI[YAW] = 0;
        #endif
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      #endif
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if(rcDelayCommand == 20) {
      if(f.ARMED) {                   // actions during armed
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
          if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
        #endif
      } else {                        // actions during not armed
        i=0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG=512;
          #if GPS
            GPS_reset_home_position();
          #endif
          #if BARO
            calibratingB=10;  // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
          #endif
        }
        #if defined(INFLIGHT_ACC_CALIBRATION)
         else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
            if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
              AccInflightCalibrationMeasurementDone = 0;
              AccInflightCalibrationSavetoEEProm = 1;
            }else{
              AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
              #if defined(BUZZER)
                if (AccInflightCalibrationArmed) SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_2);
                else     SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_ELSE);
              #endif
            }
         }
        #endif
        #ifdef MULTIPLE_CONFIGURATION_PROFILES
          if      (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) i=1;    // ROLL left  -> Profile 1
          else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) i=2;    // PITCH up   -> Profile 2
          else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) i=3;    // ROLL right -> Profile 3
          if(i) {
            global_conf.currentSet = i-1;
            writeGlobalSet(0);
            readEEPROM();
            blinkLED(2,40,i);
            SET_ALARM(ALRM_FAC_TOGGLE, i);
          }
        #endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
          #if defined(LCD_CONF)
            configurationLoop(); // beginning LCD configuration
          #endif
          previousTime = micros();
        }
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
          else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
        #endif
        #ifdef LCD_TELEMETRY_AUTO
          else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {              // Auto telemetry ON/OFF
            if (telemetry_auto) {
              telemetry_auto = 0;
              telemetry = 0;
            } else
              telemetry_auto = 1;
          }
        #endif
        #ifdef LCD_TELEMETRY_STEP
          else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {              // Telemetry next step
            telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
            #if defined( OLED_I2C_128x64)
              if (telemetry != 0) i2c_OLED_init();
            #elif defined(OLED_DIGOLE)
              if (telemetry != 0) i2c_OLED_DIGOLE_init();
            #endif
            LCDclear();
          }
        #endif
        #if ACC
          else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA=512;     // throttle=max, yaw=left, pitch=min
        #endif
        #if MAG
          else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
        #endif
        i=0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {conf.angleTrim[PITCH]+=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {conf.angleTrim[PITCH]-=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {conf.angleTrim[ROLL] +=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {conf.angleTrim[ROLL] -=2; i=1;}
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
          #if defined(LED_RING)
            blinkLedRing();
          #endif
        }
      }
    }
    #if defined(LED_FLASHER)
      led_flasher_autoselect_sequence();
    #endif

    #if defined(INFLIGHT_ACC_CALIBRATION)
      if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = 0;
      }
      if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
          InflightcalibratingA = 50;
        }
      }else if(AccInflightCalibrationMeasurementDone && !f.ARMED){
        AccInflightCalibrationMeasurementDone = 0;
        AccInflightCalibrationSavetoEEProm = 1;
      }
    #endif

    #if defined(EXTENDED_AUX_STATES)
    uint32_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |=
      (uint32_t)(rcData[AUX1+i]<1230)<<(6*i) |
      (uint32_t)(1231<rcData[AUX1+i] && rcData[AUX1+i]<1360)<<(6*i+1) |
      (uint32_t)(1361<rcData[AUX1+i] && rcData[AUX1+i]<1490)<<(6*i+2) |
      (uint32_t)(1491<rcData[AUX1+i] && rcData[AUX1+i]<1620)<<(6*i+3) |
      (uint32_t)(1621<rcData[AUX1+i] && rcData[AUX1+i]<1749)<<(6*i+4) |
      (uint32_t)(rcData[AUX1+i]>1750)<<(6*i+5);
    #else
    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    #endif

    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
    #if ACC
      if ( rcOptions[BOXANGLE] || (failsafeCnt > 5*FAILSAFE_DELAY) ) {
        // bumpless transfer to Level mode
        if (!f.ANGLE_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ANGLE_MODE = 1;
        }
      } else {
        if(f.ANGLE_MODE){
          errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        }
        f.ANGLE_MODE = 0;
      }
      if ( rcOptions[BOXHORIZON] ) {
        f.ANGLE_MODE = 0;
        if (!f.HORIZON_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.HORIZON_MODE = 1;
        }
      } else {
        if(f.HORIZON_MODE){
          errorGyroI[ROLL] = 0;errorGyroI[PITCH] = 0;
        }
        f.HORIZON_MODE = 0;
      }
    #endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
    #if !defined(GPS_LED_INDICATOR)
      if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
    #endif



    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    #if defined(HEADFREE)
      if (rcOptions[BOXHEADFREE]) {
        if (!f.HEADFREE_MODE) {
          f.HEADFREE_MODE = 1;
        }
        #if defined(ADVANCED_HEADFREE)
          if ((f.GPS_FIX && GPS_numSat >= 5) && (GPS_distanceToHome > ADV_HEADFREE_RANGE) ) {
            if (GPS_directionToHome < 180)  {headFreeModeHold = GPS_directionToHome + 180;} else {headFreeModeHold = GPS_directionToHome - 180;}
          }
        #endif
      } else {
        f.HEADFREE_MODE = 0;
      }
      if (rcOptions[BOXHEADADJ]) {
        headFreeModeHold = att.heading; // acquire new heading
      }
    #endif

    #if GPS
    // This handles the three rcOptions boxes
    // unlike other parts of the multiwii code, it looks for changes and not based on flag settings
    // by this method a priority can be established between gps option

    //Generate a packed byte of all four GPS boxes.
    uint8_t gps_modes_check = (rcOptions[BOXLAND]<< 3) + (rcOptions[BOXGPSHOME]<< 2) + (rcOptions[BOXGPSHOLD]<<1) + (rcOptions[BOXGPSNAV]);

    if (f.ARMED ) {                       //Check GPS status and armed
      //TODO: implement f.GPS_Trusted flag, idea from Dramida - Check for degraded HDOP and sudden speed jumps
      if (f.GPS_FIX) {
        if (GPS_numSat >5 ) {
          if (prv_gps_modes != gps_modes_check) {                           //Check for change since last loop
            NAV_error = NAV_ERROR_NONE;
            if (rcOptions[BOXGPSHOME]) {                                    // RTH has the priotity over everything else
              init_RTH();
            } else if (rcOptions[BOXGPSHOLD]) {                             //Position hold has priority over mission execution  //But has less priority than RTH
              if (f.GPS_mode == GPS_MODE_NAV)
                NAV_paused_at = mission_step.number;
              f.GPS_mode = GPS_MODE_HOLD;
              f.GPS_BARO_MODE = false;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], & GPS_coord[LON]); //hold at the current position
              set_new_altitude(alt.EstAlt);                                //and current altitude
              NAV_state = NAV_STATE_HOLD_INFINIT;
            } else if (rcOptions[BOXLAND]) {                               //Land now (It has priority over Navigation)
              f.GPS_mode = GPS_MODE_HOLD;
              f.GPS_BARO_MODE = true;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], & GPS_coord[LON]);
              set_new_altitude(alt.EstAlt);
              NAV_state = NAV_STATE_LAND_START;
            } else if (rcOptions[BOXGPSNAV]) {                             //Start navigation
              f.GPS_mode = GPS_MODE_NAV;                                   //Nav mode start
              f.GPS_BARO_MODE = true;
              GPS_prev[LAT] = GPS_coord[LAT];
              GPS_prev[LON] = GPS_coord[LON];
              if (NAV_paused_at != 0) {
                next_step = NAV_paused_at;
                NAV_paused_at = 0;                                         //Clear paused step
              } else {
                next_step = 1;
                jump_times = -10;                                          //Reset jump counter
              }
              NAV_state = NAV_STATE_PROCESS_NEXT;
            } else {                                                       //None of the GPS Boxes are switched on
              f.GPS_mode = GPS_MODE_NONE;
              f.GPS_BARO_MODE = false;
              f.THROTTLE_IGNORED = false;
              f.LAND_IN_PROGRESS = 0;
              f.THROTTLE_IGNORED = 0;
              NAV_state = NAV_STATE_NONE;
              GPS_reset_nav();
            }
            prv_gps_modes = gps_modes_check;
          }
        } else { //numSat>5
          //numSat dropped below 5 during navigation
          if (f.GPS_mode == GPS_MODE_NAV) {
            NAV_paused_at = mission_step.number;
            f.GPS_mode = GPS_MODE_NONE;
            set_new_altitude(alt.EstAlt);                                  //and current altitude
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          if (f.GPS_mode == GPS_MODE_HOLD || f.GPS_mode == GPS_MODE_RTH) {
            f.GPS_mode = GPS_MODE_NONE;
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          nav[0] = 0; nav[1] = 0;
        }
      } else { //f.GPS_FIX
        // GPS Fix dissapeared, very unlikely that we will be able to regain it, abort mission
        f.GPS_mode = GPS_MODE_NONE;
        NAV_state = NAV_STATE_NONE;
        NAV_paused_at = 0;
        NAV_error = NAV_ERROR_GPS_FIX_LOST;
        GPS_reset_nav();
        prv_gps_modes = 0xff;                                              //Gives a chance to restart mission when regain fix
      }
    } else { //copter is armed
      //copter is disarmed
      f.GPS_mode = GPS_MODE_NONE;
      f.GPS_BARO_MODE = false;
      f.THROTTLE_IGNORED = false;
      NAV_state = NAV_STATE_NONE;
      NAV_paused_at = 0;
      NAV_error = NAV_ERROR_DISARMED;
      GPS_reset_nav();
    }

    #endif //GPS

    #if defined(FIXEDWING) || defined(HELICOPTER)
      if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
      else {f.PASSTHRU_MODE = 0;}
    #endif
