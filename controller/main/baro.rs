// in tick
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  /* Smooth alt change routine , for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
  * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
  */
  if (f.BARO_MODE) {
    static uint8_t isAltHoldChanged = 0;
    static int16_t AltHoldCorr = 0;

    #if GPS
    if (f.LAND_IN_PROGRESS) { //If autoland is in progress then take over and decrease alt slowly
      AltHoldCorr -= GPS_conf.land_speed;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
    }
    #endif
    //IF Throttle not ignored then allow change altitude with the stick....
    if ( (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) && !f.THROTTLE_IGNORED) {
      // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
      AltHoldCorr+= rcCommand[THROTTLE] - initialThrottleHold;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
      isAltHoldChanged = 1;
    } else if (isAltHoldChanged) {
      AltHold = alt.EstAlt;
      isAltHoldChanged = 0;
    }
    rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
  }
  #endif //BARO

    #if BARO
      #if (!defined(SUPPRESS_BARO_ALTHOLD))
        #if GPS
        if (GPS_conf.takeover_baro) rcOptions[BOXBARO] = (rcOptions[BOXBARO] || f.GPS_BARO_MODE);
        #endif
        if (rcOptions[BOXBARO]) {
          if (!f.BARO_MODE) {
            f.BARO_MODE = 1;
            AltHold = alt.EstAlt;
            #if defined(ALT_HOLD_THROTTLE_MIDPOINT)
              initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
            #else
              initialThrottleHold = rcCommand[THROTTLE];
            #endif
            errorAltitudeI = 0;
            BaroPID=0;
          }
        } else {
          f.BARO_MODE = 0;
        }
      #endif
      #ifdef VARIOMETER
        if (rcOptions[BOXVARIO]) {
          if (!f.VARIO_MODE) {
            f.VARIO_MODE = 1;
          }
        } else {
          f.VARIO_MODE = 0;
        }
      #endif
    #endif
