# Minicar_Autonomous
Autonomous minicar developed with ROS2 toolkit

Minicar hardware:
- Raspberry Pi 3B+ board
- 3 Laser sensors [VL53L5CX-SATEL](https://www.st.com/en/evaluation-tools/vl53l5cx-satel.html)
- Pi camera
- Localization sensors [Decawave DWM1001c](https://www.qorvo.com/products/p/DWM1001C)
  
Dependencies:
- Ubuntu Server 22.04
- [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages)


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

## Switch between GUI and Command line mode
It is possible to switch between GUI and command line mode once the system has booted.

On the login page, press ```ctrl```+```alt```+```F1``` to go into command line mode. In this way, less resources will be used by the system.

To switch back to GUI, press ```ctrl```+```alt```+```F2```.
