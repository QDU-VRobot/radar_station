#!/bin/bash
cmake -B build&&cmake --build build -j&&build/bin/radar -m=15 -e=10000 -s=/dev/ttyACM0 0&&until build/bin/radar -m=27 -e=10000 -s=/dev/ttyACM0 -R=./ 0;do :;done
#Compile, mark and run without GUI
