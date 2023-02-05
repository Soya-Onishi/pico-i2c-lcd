#!/bin/bash

socat /dev/ttyUSB0,raw,echo=0,b115200 FD:1 | defmt-print -e $TARGET