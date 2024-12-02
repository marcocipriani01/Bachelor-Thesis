mkdir -Force build
cd build
cmake .. -G Ninja
cmake --build . -v
cp bin_dsp/libCMSISDSP.a ../
cd ..
