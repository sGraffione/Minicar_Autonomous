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
- [pca9685-hardware-interface](https://github.com/rosblox/pca9685_ros2_control/tree/main)
- [ros2-control](https://control.ros.org/iron/doc/getting_started/getting_started.html)
- [minicar_interfaces](https://github.com/sGraffione/minicar_interfaces)

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
It is reccomended to configure the system to boot in command line mode. To disable graphical login and get a text/getty login, you want to change the default target for systemd:

```
systemctl set-default multi-user.target
reboot
```

To change boot target to the GUI mode:

```
sudo systemctl set-default graphical.target
```

It is possible to switch between GUI and command line mode once the system has booted.

On the login page, press ```ctrl```+```alt```+```F1``` to go into command line mode. In this way, less resources will be used by the system.

To switch back to GUI, press ```ctrl```+```alt```+```F2```.

## Common issues

### System freezing when compiling

It might happen that the system freeze while using the ```colcon build``` command.
colcon tries to use the muximum number of parallel processes to reduce compilation time, but it might be to heavy for some hardware (e.g. Raspberry Pi boards).

This dimension can be limited with the option ```--parallel-workers NUMBER```. It can also be limited to 1 by selecting the sequential executor with ```--executor sequential```.

### Building stuck at 50%

When the build is stuck at a specific progress (e.g. 50%), it might be a memory issue. The system does not have enough RAM memory and kill a running proccess to free memory, leading to a stuck process.

A solution is increasing a swap memory. Follow [this guide](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04-de) to setup a swap memory.

### Build multiple packages simultaneously takes a long time

Due to hardware limitations, building more packages with parallel processes might takes long time and block the building process. To overcome this issue, it is suggested to use the command ```--executor sequential``` to compile each package individually.
Example of command:

```
colcon build --symlink-install --executor sequential
```
