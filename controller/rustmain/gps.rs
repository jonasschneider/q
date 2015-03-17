
// **********************
// GPS common variables, no need to put them in defines, since compiller will optimize out unused variables
// **********************
#if GPS
  gps_conf_struct GPS_conf;
#endif
  int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  int32_t  GPS_prev[2];                                 //previous pos
  int32_t  GPS_poi[2];
  uint8_t  GPS_numSat;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  int32_t  GPS_directionToPoi;
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10

  //uint8_t GPS_mode  = GPS_MODE_NONE; // contains the current selected gps flight mode --> moved to the f. structure
  uint8_t NAV_state = 0; // NAV_STATE_NONE;  /// State of the nav engine
  uint8_t NAV_error = 0; // NAV_ERROR_NONE;
  uint8_t prv_gps_modes = 0;              /// GPS_checkbox items packed into 1 byte for checking GPS mode changes
  uint32_t nav_timer_stop = 0;            /// common timer used in navigation (contains the desired stop time in millis()
  uint16_t nav_hold_time;                 /// time in seconds to hold position
  uint8_t NAV_paused_at = 0;              // This contains the mission step where poshold paused the runing mission.

  uint8_t next_step = 1;                  /// The mission step which is upcoming it equals with the mission_step stored in EEPROM
  int16_t jump_times = -10;
#if GPS
  mission_step_struct mission_step;
#endif




// jonas: tick



  //***********************************
  // THROTTLE sticks during mission and RTH
  #if GPS
  if (GPS_conf.ignore_throttle == 1) {
    if (f.GPS_mode == GPS_MODE_NAV || f.GPS_mode == GPS_MODE_RTH) {
      //rcCommand[ROLL] = 0;
      //rcCommand[PITCH] = 0;
      //rcCommand[YAW] = 0;
      f.THROTTLE_IGNORED = 1;
    } else
      f.THROTTLE_IGNORED = 0;
  }


  #if GPS
  //TODO: split cos_yaw calculations into two phases (X and Y)
  if (( f.GPS_mode != GPS_MODE_NONE ) && f.GPS_FIX_HOME ) {
    float sin_yaw_y = sin(att.heading*0.0174532925f);
    float cos_yaw_x = cos(att.heading*0.0174532925f);
    GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
    GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
    } else {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
    }

  //Used to communicate back nav angles to the GPS simulator (for HIL testing)
  #if defined(GPS_SIMULATOR)
    SerialWrite(2,0xa5);
    SerialWrite16(2,nav[LAT]+rcCommand[PITCH]);
    SerialWrite16(2,nav[LON]+rcCommand[ROLL]);
    SerialWrite16(2,(nav[LAT]+rcCommand[PITCH])-(nav[LON]+rcCommand[ROLL])); //check
  #endif

  #endif //GPS
