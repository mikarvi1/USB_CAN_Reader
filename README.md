# USB-CAN Analyzer Linux Support



# For ROS Compatibiity

1)Install USB rules to get the tty right
========================================
Copy the local rules in the folder to the /lib/udev/rules.d
and replug the battery_can or USB_CAN device physically to the board
```
$cp 10-local.rules /lib/udev/rules.d/
$cd /lib/udev/rules.d/

TEST@ubuntu:/lib/udev/rules.d$ ls
10-local.rules                          60-persistent-input.rules              73-special-net-names.rules         80-iio-sensor-proxy.rules
39-usbmuxd.rules                        60-persistent-storage-dm.rules         75-net-description.rules           80-libinput-device-groups.rules
40-usb-media-players.rules              60-persistent-storage.rules            75-probe_mtd.rules                 80-mm-candidate.rules
```
This mounts the USB as /dev/battery symlink

Now we are ready to build an run

2)Build and run
================
  cd to build folder and build
  ```
  $cd build
  $cmake ..
  $make
  ```
  you should get the following
  ```
TEST@ubuntu:~/USB_CAN_Reader/build$ make 
Scanning dependencies of target battery_can_node
[ 50%] Building CXX object CMakeFiles/battery_can_node.dir/src/battery_can_node.cpp.o
[100%] Linking CXX executable devel/lib/battery/battery_can_node
[100%] Built target battery_can_node
```

To run the binary run
```
$./devel/lib/battery/battery_can_node
```


The regular CANUSB.c is removed but the readme is kept for reference
  
This is a small C program that dumps the CAN traffic for one these adapters:
![alt text](USB-CAN.jpg)

These adapters can be found everywhere on Ebay nowadays, but there is no official Linux support. Only a Windows binary file [stored directly on GitHub](https://github.com/SeeedDocument/USB-CAN_Analyzer).

When plugged in, it will show something like this:
```
Bus 002 Device 006: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
```
And the whole thing is actually a USB to serial converter, for which Linux will provide the 'ch341-uart' driver and create a new /dev/ttyUSB device. So this program simply implements part of that serial protocol.

## Build

`canusb.c` can be compile just by running `make` or with:

```
$ gcc canusb.c -o canusb
```
## Usage

```
$ ./canusb -h
Usage: ./canusb <options>
Options:
  -h          Display this help and exit.
  -t          Print TTY/serial traffic debugging info on stderr.
  -d DEVICE   Use TTY DEVICE.
  -s SPEED    Set CAN SPEED in bps.
  -b BAUDRATE Set TTY/serial BAUDRATE (default: 2000000).
  -i ID       Inject using ID (specified as hex string).
  -j DATA     CAN DATA to inject (specified as hex string).
  -n COUNT    Terminate after COUNT frames (default: infinite).
  -g MS       Inject sleep gap in MS milliseconds (default: 200 ms).
  -m MODE     Inject payload MODE (0 = random, 1 = incremental, 2 = fixed).
```


## Example commands:
```
# dump CAN bus traffic from 1 Mbit CAN bus
$ ./canusb -t -d /dev/ttyUSB0 -s 1000000 -t

# send the bytes 0xBEEE from ID 005 on at 1 Mbit CAN bus
$ ./canusb -d /dev/ttyUSB0 -s 1000000 -t -i 5 -j BEEE
```
