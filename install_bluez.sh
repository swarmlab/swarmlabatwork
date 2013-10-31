#!/bin/sh

cd ~/Downloads/
wget http://bluez.sf.net/download/bluez-libs-3.36.tar.gz
tar -xvf bluez-libs-3.36.tar.gz
cd bluez-libs-3.36
./configure
make
sudo make install