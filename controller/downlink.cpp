#include <Arduino.h>
#include <stdint.h>
#include "virtual_wire.h"
#include <avr/io.h>
#include <HardwareSerial.h>

#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

#include <downlink.pb.h>
#include <pb_encode.h>

void mySerialCom() {
  static uint64_t frame_n = 0;
  static uint8_t buf[256];
  static q_downlink_Frame downframe;

  const int SIZE_ESTIMATE = sizeof(q_downlink_Frame); // availableForWrite doesn't use size_t, either..

  if(Serial1.availableForWrite() < SIZE_ESTIMATE) {
    // transmitter is still sending, handle that somehow?
  } else {
    // we can send the next frame!

    // this is cheap
    pb_ostream_t s = pb_ostream_from_buffer(buf, sizeof(buf));

    downframe.tm.t = micros();
    downframe.tm.n = ++frame_n;

    downframe.tm.armed = f.ARMED;
    downframe.tm.accX_smoothed = imu.accSmooth[0];
    downframe.tm.accY_smoothed = imu.accSmooth[1];
    downframe.tm.accZ_smoothed = imu.accSmooth[2];

    downframe.tm.vbat_smoothed = analog.vbat;

    bool ok = pb_encode(&s, q_downlink_Frame_fields, &downframe);

    if(!ok) {
      // Maybe a buffer overflow? In any case, encoding failed, so bail.
      return;
    }

    if(Serial1.availableForWrite() < (int)s.bytes_written) { // TODO: this cast is pretty worrisome
      // Freak out. Our size estimate was off, and now the packet won't fit in the tx buffer.
      // Actually sending it now would block. So don't send.
      return;
    }

    Serial1.write(buf, s.bytes_written);
  }
}
