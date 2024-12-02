#!/bin/bash
mkdir -p build
cd build
cmake ..
cmake --build . -v
cp bin_dsp/libCMSISDSP.a ../
