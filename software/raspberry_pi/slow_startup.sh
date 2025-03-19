#mask systemd-networkd-wait-online - "harder" disable - link the service to /dev/null
#when normaly disabled the service is still started by some dependencies
#it has long default timeout - about 2 minutes
#alternative way is to set timeout to smaller interval - like 10s
#sudo mkdir -p /etc/systemd/system/systemd-networkd-wait-online.service.d
#echo -e "[Service]\nExecStart=\nExecStart=/lib/systemd/systemd-networkd-wait-online --timeout=10" \
#| sudo tee /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf

sudo systemctl mask systemd-networkd-wait-online.service #reduce startup about 2 min

#disable snap services - reduce startup about 18 s
sudo systemctl disable snapd.service
sudo systemctl disable snapd.seeded.service
sudo systemctl disable snap.lxd.activate.service	

#disable cloud-init - reduce startup about 6 s
sudo systemctl disable cloud-init.service
sudo systemctl disable cloud-init-local.service
sudo systemctl disable cloud-config.service
sudo systemctl disable cloud-final.service

#reboot
sudo reboot
