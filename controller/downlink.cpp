#include <Arduino.h>
#include <stdint.h>
#include "virtual_wire.h"

void mySerialCom() {
  if(vx_tx_active()) {
    // transmitter is still sending, handle that somehow?
  } else {
    const char *msg = "Hello!";
    // we're done sending, shoot the next one
    vw_send((uint8_t*)msg, strlen(msg));
  }
}
