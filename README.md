# Minicar_Autonomous
Autonomous minicar developed with ROS2 toolkit

Minicar hardware:
- Raspberry Pi 3B+ board
- 3 Laser sensors
- Pi camera
- Localization tag (DWM1001)
  
Dependencies:
- Ubuntu 22.04 (suggested Xubuntu distribution)
- ROS2


# How to install Xubuntu on Raspberry Pi 3B+ board

The only available distribution of Xubuntu on the official website has amd64 architecture which is not compatible with Raspberry Pi boards. To have Xubuntu with arm64 architecture, follow these steps:
- Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and use it to install Ubuntu Server 22.04 LTS on the Micro SD card
- Insert the Micro SD in the Raspberry Pi board and boot it
- After a while, the OS will boot in terminal mode. It will ask for login credentials. Use: login: ubuntu; password: ubuntu. Follow the steps and change the password.
- Once booted, use the following commands to update and install the required packages:
  ```
  sudo apt install xfce4
  sudo apt install xubuntu-desktop
  ```
  This process might ask you to choose the display manager as well. Chose gdm3.
- Reboot the system. After a while, it will show you Xubuntu.
