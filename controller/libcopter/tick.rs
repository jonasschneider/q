fn tick() {


  computeIMU();

  // jonas: tick GPS / nav
  // jonas: tick PID

  mixTable();
  writeMotors();
  //if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
}
