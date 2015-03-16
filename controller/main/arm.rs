void go_arm() {
  if(calibratingG == 0
  #if defined(ONLYARMWHENFLAT)
    && f.ACC_CALIBRATED
  #endif
  #if defined(FAILSAFE)
    && failsafeCnt < 2
  #endif
  #if GPS && defined(ONLY_ALLOW_ARM_WITH_GPS_3DFIX)
    && (f.GPS_FIX && GPS_numSat >= 5)
  #endif
    ) {
    if(!f.ARMED && !f.BARO_MODE) { // arm now!
      f.ARMED = 1;
      #if defined(HEADFREE)
        headFreeModeHold = att.heading;
      #endif
      magHold = att.heading;
      #if defined(VBAT)
        if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
      #endif
      #ifdef ALTITUDE_RESET_ON_ARM
        #if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
        #endif
      #endif
      #ifdef LCD_TELEMETRY // reset some values when arming
        #if BARO
          BAROaltMax = alt.EstAlt;
        #endif
        #if GPS
          GPS_speedMax = 0;
        #endif
        #if defined( POWERMETER_HARD ) && (defined(LOG_VALUES) || defined(LCD_TELEMETRY))
          powerValueMaxMAH = 0;
        #endif
        #ifdef WATTS
          wattsMax = 0;
        #endif
      #endif
      #ifdef LOG_PERMANENT
        plog.arm++;           // #arm events
        plog.running = 1;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
        // write now.
        writePLog();
      #endif
    }
  } else if(!f.ARMED) {
    blinkLED(2,255,1);
    SET_ALARM(ALRM_FAC_ACC, ALRM_LVL_ON);
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
    #ifdef LOG_PERMANENT
      plog.disarm++;        // #disarm events
      plog.armed_time = armedTime ;   // lifetime in seconds
      if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
      if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
      plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
    #endif
  }
}
