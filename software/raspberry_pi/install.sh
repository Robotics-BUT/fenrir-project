#!/bin/sh

#Update system and install apt-get packages
sudo apt-get update
sudo apt-get install -y \
	git vim raspi-config htop \
	locales software-properties-common \
	curl libopencv-dev build-essential \
	wiringpi v4l-utils i2c-tools \
	gstreamer1.0-tools gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-ugly \
	python3-rpi.gpio

#Python packages
sudo apt-get install -y python3-pip
sudo pip3 install rpi_ws281x \
	adafruit-blinka \
	adafruit-circuitpython-neopixel \
	adafruit-circuitpython-bme280 \
	adafruit-circuitpython-ads1x15 \
	adafruit-circuitpython-mpu6050 \
	adafruit-circuitpython-ds3231 \
	RPLCD smbus2 pynmea2



##INSTALL ROS2
#configure locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Add ROS repository
sudo add-apt-repository -y universe #enable universe repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg #add ros2 gpg key
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null #add repository to source list

#Install ROS packages
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport

curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt-get install -y python3-colcon-common-extensions

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> /home/robot/.bashrc
echo "export ROS_DOMAIN_ID=0" >> /home/robot/.bashrc

#colcon
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt-get install -y python3-colcon-common-extensions

#Repo clone
git clone --depth 1 https://github.com/Robotics-BUT/fenrir-project.git
cd fenrir-project
git submodule update --init --recursive

cd /home/robot/fenrir-project/software/raspberry_pi/ros2_ws

colcon build
source /home/robot/fenrir-project/software/raspberry_pi/ros2_ws/install/setup.bash
colcon build
source /home/robot/fenrir-project/software/raspberry_pi/ros2_ws/install/setup.bash


###adding service
sudo cp ../prp_root.service /etc/systemd/system/prp_root.service
sudo cp ../prp_user.service /etc/systemd/system/prp_user.service