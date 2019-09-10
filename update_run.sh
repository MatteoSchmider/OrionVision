#!/bin/bash

#Alte Version entfernen
cd ..
rm -rf OrionVision

#Neueste Version pullen
git clone https://github.com/MatteoSchmider/OrionVision.git

#Ausführen
cd OrionVision
g++ main.cpp -o main -pthread -lwiringPi `pkg-config --cflags --libs opencv4`
./main
