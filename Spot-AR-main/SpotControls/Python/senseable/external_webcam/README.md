## External Peripherals

https://dev.bostondynamics.com/docs/payload/docker_containers#manage-payload-software-in-core-i-o
https://wiki.archlinux.org/title/udev
https://opensource.com/article/18/11/udev 

Running external hardware on the CORE IO requires you to add a couple extra files to your extension. You need a 'udev' rules file, and a 'udev_rules : [YOUR UDEV RULES FILENAME]' option in your 'manifest.json'. You also need to include any files this hardware relies on in your .spx file, and for these files to be mentioned in your udev rules file.

Containers which have links to external devices will not launch if Spot is turned on without the device connected. If the device is disconnected and then later reconnected, the container will require a re-start as well, as the SYMLINK within the container will be linked to an invalid Kernal value.

1a. Find identifiable attributes of your usb device (if it is recognize by Ubunutu)
    SSH into Spot, connect your usb device, then list the connected usb devices using 'lsusb'. If you see your device, then you know it is plugged in. Next, list the connected devices to Spot using 'ls /dev'. Disconnect your usb, the call the 'ls /dev' command again. Note any values that were removed after you disconnected your device. This is the assigned value that Ubunutu gave your device. HOWEVER, this name will change everytime the device is plugged in/disconnected or Spot is powered off. Youll need to get this devices attributes to set up your udev rule to create a permanant name you can refer to. Call the command 'udevadm info --attribute-walk /dev/[UBUNTU_ASSIGNED_DEVICE_NAME]'. Then look around this file for an ATTR{serial}, ATTRS{idVendor}, and/or ATTRS{idProduct} value or other identifiable info you can pass to your udev file. You can be as specific or non-specific as you feel necessary. ALSO, make sure to grab the SUBSYSTEMS value and the top-level KERNEL value. These are ESSENTIAL in getting the device properly linked within your container.
1b. Find the an identifiable number of your usb device (if it is not recognized by Ubunutu).
	TODO should be the /dev/bus/usb/X/Y path?
2. Add rule to udev file (or make the file if the is the first rule).
	This source (https://opensource.com/article/18/11/udev) recommends only adding one attribute to a rule at a time to ensure things work as you build it up. It will be a .rules file. Just copy-paste a line with some of these essential attributes. Make sure to include the SUBSYSTEMS=="usb" and the KERNEL that the device can attach to. Also provide a SYMLINK name, which will be the name that you can access the device thru the /dev/ folder. Add this .rules file to the directory where you put your extension together.
	I found the Spot pre-loaded udev rules in 'etc/udev/rules.d' were essential in learning how to format my own udev rules. SYMLINK will be your new custom name for the device. TODO add|change?
3. Add the udev rules file to your manifest.json (if it isn't already there).
	Example: "udev_rules": "[YOUR_FILENAME].rules"
4. Add your device(s) name(s) to your 'docker-compose.yml' file. Within the desired containers attributes, add a new one name "devices". For each desired device (that was added to the udev file), add a '- /dev/[YOUR-SYMLINK-NAME]:/dev/[YOUR-SYMLINK-NAME]'.
    NOTE: any containers that have a device specified but do not have this device connected when the extension is loaded onto Spot or Spot is rebooted (TODO check the rebooting) will not run. Custom ssh commands to later launch this container are required by an outside application until a better solution is found.
5. Upload your extension to Spot. If done via USB, manually updates to the udev rules may be necessary. See [https://dev.bostondynamics.com/docs/payload/docker_containers#manage-payload-software-in-core-i-o] for more info.
6. Test your udev rule is valid. When SSH'd, run the following command:
	udevadm test $(udevadm info --query=path --name=[YOUR-SYMLINK-NAME]) 2>&1
	If succesfull (and your device is plugged in), your usb device's information should print to the console. Otherwise, you'll get a 'device node not found' error. You should also now see this device file in /dev/[YOUR-SYMLINK-NAME], and you should be able to access it using this same path in any applications that can access other usb devices. Within the containers bash, you should see it in the /dev folder there as well.


Example: Our webcam connected and was recognized as '/dev/video1'. We called 'udevadm info --attribute-walk /dev/video1' and these were the top three blocks.

  looking at device '/devices/3610000.xhci/usb1/1-2/1-2.2/1-2.2.1/1-2.2.1:1.0/video4linux/video1':
    KERNEL=="video1"
    SUBSYSTEM=="video4linux"
    DRIVER==""
    ATTR{dev_debug}=="0"
    ATTR{index}=="0"
    ATTR{name}=="HD Pro Webcam C920"

  looking at parent device '/devices/3610000.xhci/usb1/1-2/1-2.2/1-2.2.1/1-2.2.1:1.0':
    KERNELS=="1-2.2.1:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="uvcvideo"
    ATTRS{authorized}=="1"
    ATTRS{bAlternateSetting}==" 0"
    ATTRS{bInterfaceClass}=="0e"
    ATTRS{bInterfaceNumber}=="00"
    ATTRS{bInterfaceProtocol}=="00"
    ATTRS{bInterfaceSubClass}=="01"
    ATTRS{bNumEndpoints}=="01"
    ATTRS{iad_bFirstInterface}=="00"
    ATTRS{iad_bFunctionClass}=="0e"
    ATTRS{iad_bFunctionProtocol}=="00"
    ATTRS{iad_bFunctionSubClass}=="03"
    ATTRS{iad_bInterfaceCount}=="02"
    ATTRS{supports_autosuspend}=="1"

  looking at parent device '/devices/3610000.xhci/usb1/1-2/1-2.2/1-2.2.1':
    KERNELS=="1-2.2.1"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{authorized}=="1"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{bDeviceClass}=="ef"
    ATTRS{bDeviceProtocol}=="01"
    ATTRS{bDeviceSubClass}=="02"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{bMaxPower}=="500mA"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{bNumInterfaces}==" 4"
    ATTRS{bcdDevice}=="0011"
    ATTRS{bmAttributes}=="80"
    ATTRS{busnum}=="1"
    ATTRS{configuration}==""
    ATTRS{devnum}=="9"
    ATTRS{devpath}=="2.2.1"
    ATTRS{idProduct}=="082d"
    ATTRS{idVendor}=="046d"
    ATTRS{ltm_capable}=="no"
    ATTRS{maxchild}=="0"
    ATTRS{product}=="HD Pro Webcam C920"
    ATTRS{quirks}=="0x42"
    ATTRS{removable}=="unknown"
    ATTRS{serial}=="473B965F"
    ATTRS{speed}=="480"
    ATTRS{urbnum}=="43"
    ATTRS{version}==" 2.00"

And this corresponding udev rule we created. Now we can refer to this device as '/dev/test-webcam1':

KERNEL=="video[0-9]*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="082d", SYMLINK+="test-webcam1"

## Adding Bluetooth Support Via Dongle

We attempted this with the test folder bluetooth audio, but were ultimately unsuccesful. It appears bluetooth and dbus support will need to be stopped on the host first (which is no issues as Spot does not have bluetooth), but we were still unsuccesful in getting the getting bluetooth support running. The Dongle itself was able to be recognized however. No tests were performed on Spot, only on an external Ubuntu machine.
