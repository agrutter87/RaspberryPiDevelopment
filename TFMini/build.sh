#!/bin/bash

# Build script for DeviceController application
g++ main.cpp TFMini.cpp -l wiringPi -o DeviceController