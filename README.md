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

        $ avrdude -c arduino -p m328p -b 57600 -P /dev/ttyUSB0 -U flash:w:out.hex
