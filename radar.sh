#!/bin/bash
cmake -B build&&cmake --build build -j&&build/bin/radar -m=15 -e=8000 -V=-2 -s=/dev/ttyUSB0 DA0229542&&until build/bin/radar -m=59 -e=8000 -V=-2 -s=/dev/ttyUSB0 -R=./ DA0229542;do :;done
#Compile, mark and run without GUI
