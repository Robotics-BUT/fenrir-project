# RPi Installation

## Required HW

 - Raspberry Pi 4B (8GB RAM recommended)
 - Micro SD Card (minimal 16GB, 64GB recommended)
 - SD Card to USB Adapter

## Installation


Install RPi-Imager on your Linux PC

```
sudo apt install rpi-imager
```

Run RPi-Imager

```
rpi-imager
```

<img src="../images/rpi_install_imager.png" width="300">

Select Device (Raspberry Pi 4)

<img src="../images/rpi_install_select_device.png" width="300">

Select OS - Others General Purpose OS -> Ubuntu -> Ubuntu Server 22.04.5 LTS (64b)

<img src="../images/rpi_install_select_os_1.png" width="300">

<img src="../images/rpi_install_select_os_2.png" width="300">

<img src="../images/rpi_install_select_os_3.png" width="300">

Select target device (SD Card)

<img src="../images/rpi_install_select_msd.png" width="300">

Click Next -> Edit Settings

<img src="../images/rpi_install_settings.png" width="300">

Setup user, password, WiFi and SSH

<img src="../images/rpi_install_config_wifi.png" width="300">

<img src="../images/rpi_install_config_ssh.png" width="300">

Confirm and wait until writing is finished.

<img src="../images/rpi_install_writing.png" width="300">

Now put the SD card into the RPi and power on.

In case of problems, please visit [Official Documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html#install-using-imager) 

## OS Configuration

Connect to the Raspberry Pi

```
ssh <user>@<ip_address>
```

Run first installation script and reboot afterwards. The script may require user input for confirmation on restarting services
Optionaly run script to disable services for faster boot.
```
sudo bash first_start.sh
sudo bash slow_startup.sh #optional
sudo reboot now
```

The script will:
- run system update and upgrade some system packages
- modify the boot firmware config
- create a swapfile (can be removed on systems with more RAM)
- add the user to following groups - `video`, `tty`, `dialout`

The `slow_startup.sh` will disable this services:
- network wait
- snap and related services
- cloud init and related services

After rebooting the system, run the second script.
```
sudo bash installation.sh
```
The script will:
- update system and install necessary packages
- install Python packages
- install ROS 2 and Colcon
- clone `fenrir-project` repository
- build ROS 2 packages
- add services to the system and enable them at startup


If user name is different then "robot", paths in scripts and service files need to be changed, along with the user for one of services.
```
#in repository
fenrir-project/software/raspberry_pi/prp_root.service 
fenrir-project/software/raspberry_pi/prp_user.service 
#or after install
/etc/systemd/system/prp_root.service
/etc/systemd/system/prp_user.service
```
The `ROS_DOMAIN_ID` can be changed in the `*.service` files and optionally (but preferably) in `~/.bashrc`.

## Starting services
Start the necessary services for ROS 2 nodes:
```
sudo systemctl start prp_root.service
sudo systemctl start prp_user.service
```

# Cloning SD card
### Prequisitions: 
- Linux OS
- SD card reader
- A SD card with configured system
- An empty SD card of the same size

Example of how to clone a SD card and rename hostname
```bash
# Insert the SD card with the existing system
lsblk # Identify the SD card and its partitions (e.g., /dev/sdc1 and /dev/sdc2)

# Unmout the SD card
sudo umount /dev/sdc*

# Create an image of the SD card with the system
sudo dd if=/dev/sdc of=~/Documents/prp/robot.img bs=4M status=progress  
sudo eject /dev/sdc

# Swap the SD card with an empty one
lsblk # Identify SD card and its partitions (e.g., /dev/sdc1 and /dev/sdc2)
sudo dd if=~/Documents/prp/robot.img of=/dev/sdc bs=4M status=progress # apply image to the SD card
sudo sync

# Mount the SD card's partitions
sudo mkdir /media/jakub/card1
sudo mount /dev/sdc1 /media/jakub/card1
sudo mkdir /media/jakub/card2
sudo mount /dev/sdc2 /media/jakub/card2

# Replace the old hostname for new one
sudo sed -i 's/prp-red/prp-green/g' /media/jakub/card1/user-data /media/jakub/card2/etc/hostname /media/jakub/card2/etc/hosts

# Unmount SD card and remove created folders
sudo umount /media/jakub/card1
sudo umount /media/jakub/card2
sudo rmdir /media/jakub/card1/
sudo rmdir /media/jakub/card2/
```

`ROS_DOMAIN_ID` can be rewritten in files:
```
~/.bashrc
/etc/systemd/system/prp_user.service
/etc/systemd/system/prp_root.service
```

