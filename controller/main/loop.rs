

// ******** Main Loop *********
void loop () {
  if ((int16_t)(currentTime-rcTime) >0 ) { // 50Hz
    tickRX();
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if MAG
          if (Mag_getADC() != 0) break; // 320 µs
        #endif
      case 1:
        taskOrder++;
        #if BARO
          if (Baro_update() != 0) break; // for MS baro: I2C set and get: 220 us  -  presure and temperature computation 160 us
        #endif
      case 2:
        taskOrder++;
        #if BARO
          if (getEstimatedAltitude() != 0) break; // 280 us
        #endif
      case 3:
        taskOrder++;
        #if GPS
          if (GPS_Compute() != 0) break;  // performs computation on new frame only if present
          #if defined(I2C_GPS)
          if (GPS_NewData() != 0) break;  // 160 us with no new data / much more with new data
          #endif
        #endif
      case 4:
        taskOrder=0;
        #if SONAR
          Sonar_update(); //debug[2] = sonarAlt;
        #endif
        #ifdef LANDING_LIGHTS_DDR
          auto_switch_landing_lights();
        #endif
        #ifdef VARIOMETER
          if (f.VARIO_MODE) vario_signaling();
        #endif
        break;
    }
  }

  // wait for timeout
  while(1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    if (cycleTime >= LOOP_TIME) break;
  }
  previousTime = currentTime;

  copter.tick()
}
