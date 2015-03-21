# Get to da chopper

- Get a [NanoWii](http://www.hobbyking.com/hobbyking/store/__22322__multiwii_nanowii_atmega32u4_micro_flight_controller_usb_gyro_acc.html) board.

- Install [Vagrant](https://www.vagrantup.com/)
- In a terminal:

        $ vagrant up
        [..]
        $ vagrant ssh
        vagrant@vagrant$ cd /vagrant/controller/build
        vagrant@vagrant$ make

  This will build the firmware in `controller/build/controller.hex`.
- To flash the firmware, run this from outside the VM:

  For the Arduino Leonardo (like the one on the NanoWii): first, press the RESET button. Then, within two seconds, do this:

        build$ avrdude -patmega32u4 -cavr109 -U flash:w:controller.hex -P /dev/tty.usbmodem1421

  For the Arduino Mega, it's a bit easier:

        build$ avrdude -c arduino -p m2560 -c stk500v2 -b 115200 -P /dev/tty.usbmodem1421 -U flash:w:controller.hex

