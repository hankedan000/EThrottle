#!/bin/sh

./gen-ms-tables.py \
    --ini-in-file ../arduino/EThrottle/EThrottle.ini.in EThrottleTables.csv \
    --ini-out-file ../arduino/EThrottle/EThrottle-1.0.0.ini \
    --out-file-h ../arduino/EThrottle/EThrottleTables.h \
    --out-file-cpp ../arduino/EThrottle/EThrottleTables.cpp