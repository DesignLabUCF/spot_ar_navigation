#! /bin/bash

# Launch bluetooth services
service dbus start
bluetoothd &

# Start pulseaudio. Can connect and trust without this, but cannot pair
# --exit-idle-time set to a negative value prevent pulseaudio from disconnecting during down time
#pulseaudio -k
pulseaudio --start --exit-idle-time=-1