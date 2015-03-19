# -*- mode: ruby -*-
# vi: set ft=ruby :

$script = <<SCRIPT
set -eux

sudo apt-get update
sudo apt-get -y install gcc-avr avr-libc avrdude protobuf-compiler python-pip
SCRIPT

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/trusty64"
  config.vm.provision "shell", inline: $script

  # http://spin.atomicobject.com/2014/03/21/smartcard-virtualbox-vm/
  config.vm.provider :virtualbox do |vb|
   vb.customize ['modifyvm', :id, '--usb', 'on']
   vb.customize ['usbfilter', 'add', '0', '--target', :id, '--name', 'Arduino Leonardo', '--vendorid', '0x2341', '--productid', '0x8036']
   vb.customize ['usbfilter', 'add', '1', '--target', :id, '--name', 'Arduino', '--vendorid', '0x2341', '--productid', '0x0042']
 end
end
