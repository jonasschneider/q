# Get to da chopper

- Install [Vagrant](https://www.vagrantup.com/)
- In a terminal:

        $ vagrant up
        [..]
        $ vagrant ssh
        vagrant@vagrant-ubuntu-trusty-64$ cd /vagrant/controller/multiwii-build
        vagrant@vagrant-ubuntu-trusty-64$ make

  This will build the firmware and flash it!
