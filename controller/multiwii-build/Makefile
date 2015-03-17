# http://electronics.stackexchange.com/questions/28455/programming-for-the-arduino-without-the-arduino-ide-but-with-the-provided-libr

# find AVR symbol:
#
#     $ find /usr/local/Cellar/avr-gcc/4.8.3/avr/lib | xargs -n 1 -I {} bash -c 'avr-objdump -x {} |grep __eerd_block_m2560 && echo {}'

mcu=atmega2560

AVR_GCC=avr-gcc
arduino_dot_app=$(shell echo $$HOME)/Downloads/Arduino.app
arduino_variant=$(arduino_dot_app)/Contents/Java/hardware/arduino/avr/variants/mega
arduino_core=$(arduino_dot_app)/Contents/Java/hardware/arduino/avr/cores/arduino
CFLAGS=-I$(arduino_core)
CFLAGS+=-I$(arduino_variant)
CFLAGS+=-Wall -DF_CPU=16000000UL -Os -mmcu=${mcu}
#CFLAGS+=-I../reference-multiwii-firmware

LDFLAGS=-mmcu=${mcu} -Wl,--gc-sections

multiwii_objs=$(foreach o,$(shell cd ../reference-multiwii-firmware; ls *.cpp),$(o).o)

all: upload

%.cpp.o: ../reference-multiwii-firmware/%.cpp
	avr-gcc -c $(CFLAGS) -o $@ $<

main.elf: libarduino.a $(multiwii_objs) arduino-main.o
	avr-gcc $(LDFLAGS) -o $@ $(multiwii_objs) arduino-main.o -L. -larduino

out.hex: main.elf
	avr-objcopy -O ihex -R .eeprom main.elf out.hex

upload: out.hex
	avrdude -c arduino -p m328p -b 57600 -P /dev/ttyUSB0 -U flash:w:out.hex

arduino-main.o: ${arduino_core}/main.cpp
	avr-gcc -c $(CFLAGS) -o $@ $<

clean:
		rm -f ${LIBARDUINO_OBJS} ${LIBARDUINO_CPPOBJS} libarduino.a $(multiwii_objs) out.hex main.elf arduino-main.o


# The following code deals with building libarduino.a
LIBARDUINO_HDRS = Arduino.h Client.h HardwareSerial.h HardwareSerial_private.h IPAddress.h Print.h Printable.h Server.h Stream.h USBAPI.h USBCore.h USBDesc.h Udp.h WCharacter.h WString.h binary.h new.h wiring_private.h
LIBARDUINO_OBJS = WInterrupts.o wiring_analog.o wiring.o wiring_digital.o wiring_pulse.o wiring_shift.o hooks.o
LIBARDUINO_CPPOBJS = CDC.o HardwareSerial.o HID.o IPAddress.o main.o new.o Print.o Stream.o Tone.o USBCore.o WMath.o WString.o
#TODO: may also need to adjust F_CPU if your clock isn't set to 16Mhz
LIBARDUINO_CFLAGS = -I./ -std=gnu99  -DF_CPU=16000000UL -Os -mmcu=${mcu} -I$(arduino_variant) -include avr/io.h
LIBARDUINO_CPPFLAGS = -I./ -DF_CPU=16000000UL -Os -mmcu=${mcu} -I$(arduino_variant) -I$(arduino_core)
LIBARDUINO_CC=avr-gcc
LIBARDUINO_CPP=avr-g++
LIBARDUINO_AR=avr-ar

libarduino.a:   ${LIBARDUINO_OBJS}
		${LIBARDUINO_AR} crs libarduino.a $(LIBARDUINO_OBJS)

$(LIBARDUINO_OBJS): %.o: $(arduino_dot_app)/Contents/Java/hardware/arduino/avr/cores/arduino/%.c $(foreach hdr,${LIBARDUINO_HDRS},$(arduino_dot_app)/Contents/Java/hardware/arduino/avr/cores/arduino/${hdr})
		${LIBARDUINO_CC} ${LIBARDUINO_CFLAGS} -c -o $@ $<

$(LIBARDUINO_CPPOBJS): %.o: $(arduino_dot_app)/Contents/Java/hardware/arduino/avr/cores/arduino/%.cpp $(foreach hdr,${HDRS},$(arduino_dot_app)/Contents/Java/hardware/arduino/avr/cores/arduino/${hdr})
		${LIBARDUINO_CPP} ${LIBARDUINO_CPPFLAGS} -c -o $@ $<
