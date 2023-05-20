# Hardware Tested

## Monoprice PTZ Camera 39512

Monoprice [camera 39512](https://www.monoprice.com/product?p_id=39512) with

* [Video System Control Architecture, a.k.a. VISCA](https://en.wikipedia.org/wiki/VISCA_Protocol)
- controlled Pan/Tilt/Zoom;
* 1080p resolution;
* USB3 connection to computer carrying both Video stream and VISCA signals;
* VISCA serial socket present

A pleasant surprise: the camera includes a built-in usb hub with a built-in
USB->UART interface,  so a single USB3 cable is used for both VISCA control and
video streaming.  Here is the part of the `lsusb` output contributed by the
camera:
```
Bus 002 Device 010: ID 1bcf:2cbd Sunplus Innovation Technology Inc. USB 3.0 Camera
Bus 001 Device 036: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
Bus 001 Device 038: ID 1a40:0101 Terminus Technology Inc. Hub
```


## Installation on Linux

No to braille display:
```sh
sudo apt remove brltty
```

Connect your USB camera - this may result in /dev/ttyUSB* being created!
Mine did just that!

Connect USB->serial converter, then check kernel messages for possible errors:
```sh
journalctl -b | tail -40
```

A device should have been created:
```sh
ls -la /dev/ttyUSB*
```
Which gave me:
```
crw-rw---- 1 root dialout 188, 0 May 16 08:47 /dev/ttyUSB0
```
An attempt to read/write to such a device fails if your sera account does not
belong top group dialout.  To remedy that:

```sh
sudo usermod -a -G dialout _user_
```

Alternatively, just expose the device to all:
```sh
sudo chmod a+rw /dev/ttyUSB0
```
