# http://electronics.stackexchange.com/questions/28455/programming-for-the-arduino-without-the-arduino-ide-but-with-the-provided-libr

# find AVR symbol:
#
#     $ find /usr/local/Cellar/avr-gcc/4.8.3/avr/lib | xargs -n 1 -I {} bash -c 'avr-objdump -x {} |grep __eerd_block_m2560 && echo {}'

# your hardware:
mcu=atmega32u4
arduino_board=leonardo
#mcu=atmega2560
#arduino_board=mega

AVR_GCC=avr-gcc
arduino_sdk=arduino-1.6.1
arduino_variant=${arduino_sdk}/hardware/arduino/avr/variants/${arduino_board}
arduino_core=${arduino_sdk}/hardware/arduino/avr/cores/arduino
CFLAGS=-I$(arduino_core)
CFLAGS+=-I$(arduino_variant)
CFLAGS+=-Wall -DF_CPU=16000000UL -Os -mmcu=${mcu}

LDFLAGS=-mmcu=${mcu} -Wl,--gc-sections

multiwii_objs=$(foreach o,$(shell cd ../multiwii; ls *.cpp |grep -v Protocol),$(o).o)
myobjs=downlink.o
objs=$(multiwii_objs) downlink.pb.c.o $(myobjs)
all: controller.hex

%.cpp.o: ../multiwii/%.cpp ../multiwii/*.h
	avr-gcc -c $(CFLAGS) -o $@ $<

$(myobjs): %.o: ../%.c
	avr-gcc -c $(CFLAGS) -o $@ $<

%.pb.c.o: %.pb.c
	avr-gcc -c $(CFLAGS) -o $@ -Inanopb $<

downlink.pb.c: downlink.pb nanopb
	python nanopb/generator/nanopb_generator.py $<

downlink.pb: ../../downlink.proto
	protoc -o$@ -I../../ $<

main.elf: libarduino.a $(objs) arduino-main.o
	avr-gcc $(LDFLAGS) -o $@ $(objs) arduino-main.o -L. -larduino

controller.hex: main.elf
	avr-objcopy -O ihex -R .eeprom $< $@

arduino-main.o: ${arduino_core}/main.cpp
	avr-gcc -c $(CFLAGS) -o $@ $<

clean:
		rm -f ${LIBARDUINO_OBJS} ${LIBARDUINO_CPPOBJS} libarduino.a $(multiwii_objs) out.hex main.elf arduino-main.o *.pb *.pb.* $(myobjs)

nanopb:
	sudo pip install protobuf
	rm -fr $@~
	git clone https://code.google.com/p/nanopb $@~
	cd $@~; git reset --hard ef422656a57b3f472b192691a40f48d0d72f2927
	cd $@~/generator/proto; make
	mv $@~ $@


# The following code deals with building libarduino.a.

# first, infrastructure for downloading the Arduino SDK sources
arduino-1.6.1/%: arduino-1.6.1
arduino-1.6.1: arduino-1.6.1-linux64.tar.xz
	tar xf $<; true
arduino-1.6.1-linux64.tar.xz:
	rm -f $@~
	curl -L http://arduino.cc/download_handler.php?f=/arduino-1.6.1-linux64.tar.xz > $@~
	echo 'c344b166efe3a6839cc54af16510b6869cbb4f1fcfaf06a45ca4eba3fca8b2b9  arduino-1.6.1-linux64.tar.xz~' | sha256sum -c
	mv $@~ $@

# list out all the files explicitly because make is stupid
LIBARDUINO_HDRS = Arduino.h Client.h HardwareSerial.h HardwareSerial_private.h IPAddress.h Print.h Printable.h Server.h Stream.h USBAPI.h USBCore.h USBDesc.h Udp.h WCharacter.h WString.h binary.h new.h wiring_private.h
LIBARDUINO_OBJS = WInterrupts.o wiring_analog.o wiring.o wiring_digital.o wiring_pulse.o wiring_shift.o hooks.o
LIBARDUINO_CPPOBJS = CDC.o HardwareSerial.o HID.o IPAddress.o main.o new.o Print.o Stream.o Tone.o USBCore.o WMath.o WString.o

#TODO: may also need to adjust F_CPU if your clock isn't set to 16Mhz
LIBARDUINO_FLAGS = -DF_CPU=16000000UL -Os -mmcu=${mcu}

# set build-time arduino USB constants (see arduino-1.6.1/hardware/arduino/avr/platform.txt)
LIBARDUINO_FLAGS += -DUSB_VID=0x2341 -DUSB_PID=0x8036 #might also need  '-DUSB_MANUFACTURER={build.usb_manufacturer}' '-DUSB_PRODUCT={build.usb_product}'

LIBARDUINO_CFLAGS = $(LIBARDUINO_FLAGS) -I./ -std=gnu99 -I$(arduino_variant)
LIBARDUINO_CPPFLAGS = $(LIBARDUINO_FLAGS) -I./ -Os -I$(arduino_variant) -I$(arduino_core)

LIBARDUINO_CC=avr-gcc
LIBARDUINO_CPP=avr-g++
LIBARDUINO_AR=avr-ar

libarduino.a: ${LIBARDUINO_OBJS} $(LIBARDUINO_CPPOBJS)
		${LIBARDUINO_AR} crs libarduino.a $(LIBARDUINO_OBJS) $(LIBARDUINO_CPPOBJS)

$(LIBARDUINO_OBJS): %.o: $(arduino_sdk)/hardware/arduino/avr/cores/arduino/%.c $(foreach hdr,${LIBARDUINO_HDRS},$(arduino_sdk)/hardware/arduino/avr/cores/arduino/${hdr}) $(arduino_sdk)
		${LIBARDUINO_CC} ${LIBARDUINO_CFLAGS} -c -o $@ $<

$(LIBARDUINO_CPPOBJS): %.o: $(arduino_sdk)/hardware/arduino/avr/cores/arduino/%.cpp $(foreach hdr,${HDRS},$(arduino_sdk)/hardware/arduino/avr/cores/arduino/${hdr}) $(arduino_sdk)
		${LIBARDUINO_CPP} ${LIBARDUINO_CPPFLAGS} -c -o $@ $<
