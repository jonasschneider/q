#include <Arduino.h>
#include <stdint.h>
#include "virtual_wire.h"
#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void mySerialCom() {
  if(vx_tx_active()) {
    // transmitter is still sending, handle that somehow?
  } else {
    uint16_t msg[4];
    msg[0] = (uint16_t)(f.ARMED ? '!' : '-');
    msg[1] = imu.accSmooth[0];
    msg[2] = imu.accSmooth[1];
    msg[3] = imu.accSmooth[2];

    // we're done sending, shoot the next one
    vw_send((uint8_t*)msg, sizeof(msg));
  }
}
