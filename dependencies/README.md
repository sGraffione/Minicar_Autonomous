# How to install dependencies

Download the directory ```dependencies``` and use these commands to install each library.

## bcm2835

Use the following commands

```                
tar xvfz bcm2835-1.58.tar.gz;                      
cd bcm2835-1.58;                       
./configure;                      
make;        
sudo make install
```

## MPU6050

```
cd MPU6050
make all
sudo make install
```