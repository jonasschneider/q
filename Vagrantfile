# -*- mode: ruby -*-
# vi: set ft=ruby :

$script = <<SCRIPT
set -eux

sudo apt-get update
sudo apt-get -y install gcc-avr avr-libc avrdude
SCRIPT

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/trusty64"
  config.vm.provision "shell", inline: $script

  # for the USB programmer, something like this:
  # http://spin.atomicobject.com/2014/03/21/smartcard-virtualbox-vm/
end
