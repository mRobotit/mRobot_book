echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout",  SYMLINK+="mrobotit_base"' >/etc/udev/rules.d/mrobotit_base.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout",  SYMLINK+="mrobotit_laser"' >/etc/udev/rules.d/mrobotit_laser.rules
service udev reload
sleep 2
service udev restart


