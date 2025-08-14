#! /bin/bash

# Launch bluetooth services
#echo "Starting dbus..."
#service dbus start

echo "Starting bluetooth support..."
bluetoothd &

echo "Starting pulseaudio..."
# Start pulseaudio. Can connect and trust without this, but cannot pair
# --exit-idle-time set to a negative value prevent pulseaudio from disconnecting during down time
pulseaudio --start --exit-idle-time=-1

# Launch Xbox One controller support
#systemctl enable xow
#systemctl start xow
#service xow start
#xboxdrv –detach-kernel-driver