# Get to da chopper

- Get a [NanoWii](http://www.hobbyking.com/hobbyking/store/__22322__multiwii_nanowii_atmega32u4_micro_flight_controller_usb_gyro_acc.html) board.

- Install [Vagrant](https://www.vagrantup.com/)
- In a terminal:

        $ vagrant up
        [..]
        $ vagrant ssh
        vagrant@vagrant-ubuntu-trusty-64$ cd /vagrant/controller/multiwii-build
        vagrant@vagrant-ubuntu-trusty-64$ make

  This will build the firmware and flash it!
