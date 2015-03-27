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
  static uint16_t skips = 0;
  static uint16_t errors = 0;
  static uint8_t buf[256];
  static q_downlink_Frame downframe;

  const int SIZE_ESTIMATE = sizeof(q_downlink_Frame); // availableForWrite doesn't use size_t, either..
  Serial.print(Serial1.availableForWrite(), DEC);
  Serial.print(" ");
  Serial.print(SIZE_ESTIMATE, DEC);
  if(Serial1.availableForWrite() < SIZE_ESTIMATE) {
    // transmitter is still sending
    skips++;
    return;
  }
  // we can send the next frame!

  // this is cheap
  pb_ostream_t s = pb_ostream_from_buffer(buf, sizeof(buf));

  downframe.tm.t = micros();
  downframe.tm.n = ++frame_n;
  downframe.tm.skips = skips;

  downframe.tm.comm_errors = errors;

  downframe.tm.armed = f.ARMED;
  downframe.tm.accX_smoothed = imu.accSmooth[0];
  downframe.tm.accY_smoothed = imu.accSmooth[1];
  downframe.tm.accZ_smoothed = imu.accSmooth[2];

  downframe.tm.vbat_smoothed = analog.vbat;

  for(int i = 0; i < 4; i++) {
    downframe.tm.currentMotorValues[i] = motor[i];
  }

  downframe.tm.gyroX_smoothed = imu.gyroData[0];
  downframe.tm.gyroY_smoothed = imu.gyroData[1];
  downframe.tm.gyroZ_smoothed = imu.gyroData[2];

  bool ok = pb_encode(&s, q_downlink_Frame_fields, &downframe);
  if(!ok) {
    // Maybe a buffer overflow? In any case, encoding failed, so bail.
    errors++;
    return;
  }

  if(Serial1.availableForWrite() < (int)s.bytes_written) { // TODO: this cast is pretty worrisome
    // Freak out. Our size estimate was off, and now the packet won't fit in the tx buffer.
    // Actually sending it now would block. So don't send.
    errors++;
    return;
  }
  skips = 0;

  // write the synch marker, then the length, then the message
  Serial1.print("XXXXXXXXXX");
  Serial1.write((char)(s.bytes_written>>8)&0xff);
  Serial1.write((char)(s.bytes_written)&0xff);
  Serial1.write(buf, s.bytes_written);
  Serial.print("sent!");
}
