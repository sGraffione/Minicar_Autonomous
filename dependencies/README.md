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