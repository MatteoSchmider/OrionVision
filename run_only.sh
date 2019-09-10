#!/bin/bash
g++ main.cpp -o main -pthread -lwiringPi `pkg-config --cflags --libs opencv4`
./main
