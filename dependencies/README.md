# How to install dependencies

Download the directory ```dependencies``` and use these commands to install each library.

If CMakeList can't find WiringPi library, you will need to move the file ```FindWiringPi.cmake``` to ```/usr/share/cmake-*.*/Modules``` where *.* is you version of cmake.

## MPU6050

First, install some dependencies
```
sudo apt install i2c-tools libi2c-dev
```
Then install the library
```
cd MPU6050
make all
sudo make install
```

## VL53L5CX_Linux_driver_1.3.11

This folder contains general informations and tests about the ranging sensor used in this setup.
The directory can be downloaded from the [producer website](https://www.st.com/en/embedded-software/stsw-img025.html). Please refer to this page to check updates and/or news about this product.