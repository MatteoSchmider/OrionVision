#!/bin/bash

#cd ..
rm -rf OrionVision
git clone https://github.com/MatteoSchmider/OrionVision.git
cd OrionVision
g++ main.cpp -o main `pkg-config --cflags --libs opencv`
./main