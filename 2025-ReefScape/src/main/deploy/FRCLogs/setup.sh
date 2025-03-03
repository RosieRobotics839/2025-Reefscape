#!/bin/bash

mkdir -p /usr/local/bin/

echo "#!/bin/bash" > /usr/local/bin/movelogs.sh
echo "sed -i 's/\x0D$//' /home/lvuser/deploy/FRCLogs/movelogs.sh" >> /usr/local/bin/movelogs.sh
echo "bash /home/lvuser/deploy/FRCLogs/movelogs.sh" >> /usr/local/bin/movelogs.sh
chmod a+x /usr/local/bin/movelogs.sh

sed -i 's/\x0D$//' zz-movelogs.rules
cp zz-movelogs.rules /etc/udev/rules.d/

udevadm control --reload-rules
udevadm trigger
