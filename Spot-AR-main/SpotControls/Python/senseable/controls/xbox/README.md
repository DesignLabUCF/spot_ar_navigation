## Containerized Controller Support

This container supports the use of a bluetooth dongle. If bluetooth support was installed to the CORE I/O outside of this container, ensure the service has been stopped prior to launching this container.

Input reading is supported via Evdev in Python. This reads the input events by joysticks/controllers in Ubunutu. If connected by Bluetooth in the container, these inputs may not be immediately available, so the container may exit and auto-restart after connection is made. This is not necessary when using a wired controller. 

This script may also attempt to call our custom transform_recording_service. If it is not running in it's own container on Spot, then the call should gracefully exit.

Tested controllers:
1. Xbox One
2. Playstation 4

Xbox One controllers:
Can connect with wired controllers, wireless controllers using a usb cable, or wireless controller using bluetooth as long as the controller model support bluetooth. Older controller variants support only the Xbox-dongle and not bluetooth, so verify your controller is a bluetooth supported model.
https://www.reddit.com/r/xboxone/comments/5fr7r1/easy_way_to_tell_if_a_controller_supports/

PS4 controllers:
https://www.playstation.com/en-us/support/hardware/ps4-pair-dualshock-4-wireless-with-pc-or-mac/#connect