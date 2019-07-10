#!/bin/bash

#Befehlsliste Tinkerboard

#	sudo tinker-config
#	#hostname auf orion1/orion2 ändern
#	#passwort auf hallo1234 setzen
#	#kamera auf ov5647 ändern
#	#uart aktivieren
#	
#	sudo apt-get install x11vnc
#	sudo nano ~/.config/lxsession/LXDE/autostart
#		@x11vnc -noxrecord -forever #in der letzten Zeile hinzufügen
#		#zum aktivieren:
#		sudo reboot
	
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git build-essential cmake gfortran libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libatlas-base-dev libavcodec-dev libavformat-dev libavresample-dev libcanberra-gtk* libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libswscale-dev libtiff5-dev libv4l-dev libxvidcore-dev libx264-dev pkg-config python python2.7-dev python3-dev unzip wget apt-file v4l-utils
cd /opt
sudo wget -O opencv.zip https://github.com/opencv/opencv/archive/4.1.0.zip
sudo unzip opencv.zip
sudo wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.1.0.zip
sudo unzip opencv_contrib.zip
sudo wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
sudo pip3 install numpy
sudo python get-pip.py
sudo pip2 install numpy
cd /opt/opencv-4.1.0/cmake
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-4.1.0/modules -D ENABLE_NEON=ON -D ENABLE_VFPV3=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D WITH_LIBV4L=ON -D WITH_OPENGL=ON ..
sudo make install -j4
sudo make clean
sudo ldconfig
cd

#Neueste Version pullen
git clone https://github.com/MatteoSchmider/OrionVision.git

#Ausführen
cd OrionVision
g++ main.cpp -o main `pkg-config --cflags --libs opencv4`
./main

# L = p * G * V
# p = 1.2
# G = 2 * pi * radius * 2 * pi * radius * revs/sec
# 