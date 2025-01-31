
On first start run `first_start.sh`
Script will:
- run system update and upgrade some system packages
- modify boot firmware config
- create swapfile
- add user to groups - video, tty, dialout
- disable boot ethernet timeout
System needs to be rebooted after.
After reboot run `install.sh`
Script will:
- update system and install packages
- install python packages
- install ROS 2 and colcon
- clone this repository
- build ROS2 packages
- add services to system and enable them at startup

System need to be configured for user "robot" otherwise some paths, especially in services, needs to be altered.

