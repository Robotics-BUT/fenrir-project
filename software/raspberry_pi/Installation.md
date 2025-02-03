On the first start, run `first_start.sh`.
The script will:
- run a system update and upgrade some system packages
- modify the boot firmware config
- create a swapfile
- add the user to the following groups - video, tty, dialout
- disable the boot ethernet timeout

The system needs to be rebooted after this steps.

After rebooting, run `install.sh`
The script will:
- update the system and install required packages
- install python packages
- install ROS 2 and Colcon
- clone this repository
- build ROS2 packages
- add services to the system and enable them at startup

The system must be configured for the user "robot", otherwise some paths, especially in service files, need to be altered.

