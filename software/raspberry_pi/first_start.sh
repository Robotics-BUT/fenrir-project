#!/bin/sh
#update system
sudo apt-get update
##/boot/firmware/config.txt
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup
echo "#[all]
#over_voltage=6
#arm_freq=2000
#gpu_freq=750

#enable_uart=1
#dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt 

#set legacy mode for camera
sudo raspi-config nonint do_legacy 0 

# add user to video group 
sudo usermod -aG video $USER
sudo usermod -aG tty $USER
sudo usermod -aG dialout $USER

# swapfile
sudo fallocate -l 4G /swapfile #create swapfile
sudo chmod 600 /swapfile 
sudo mkswap /swapfile #mark file as swap
sudo swapon /swapfile #OS start using swapfile - only until shutdown
sudo cp /etc/fstab /etc/fstab.back
echo '/swapfile none swap defaults 0 0' | sudo tee -a /etc/fstab # mount swapfile as swap on start

sudo apt-get upgrade -y systemd udev #because of ROS2 #needs reboot!!