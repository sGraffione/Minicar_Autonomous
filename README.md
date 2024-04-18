# Minicar_Autonomous
Autonomous minicar developed with ROS2 toolkit

Minicar hardware:
- Raspberry Pi 3B+ board
- 3 Laser sensors
- Pi camera
- Localization tag (DWM1001)
  
Dependencies:
- Ubuntu Server 22.04
- ROS2


# How to install Ubuntu Server 22.04 on Raspberry Pi 3B+ board and add GUI interface

- Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and use it to install Ubuntu Server 22.04 LTS on the Micro SD card
- Insert the Micro SD in the Raspberry Pi board and boot it
- After a while, the OS will boot in terminal mode. It will ask for login credentials. Use: login: ubuntu; password: ubuntu. Follow the steps and change the password.
- Once booted, use the following command to update and install the required packages:
  ```
  sudo apt update && sudo apt upgrade
  ```
- Install a Display Manager such as ```Slim``` (or ```lightdm```)
  ```
  sudo apt install slim
  ```
- Install the GUI you prefer. I suggest installing ```lxde``` because it minimizes the impact on the resources (if you have better hardware, you can choose whatever you prefer)
  ```
  sudo apt install lxde
  ```
- Reboot the system
