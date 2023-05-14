# visca-driver

Some python3 code to:

* drive serial/usb-attached camera supporting
[VISCA](https://en.wikipedia.org/wiki/VISCA_Protocol) PTZ control;
* with ncurses-based PTZ control
* offer rudimentary web GUI for the same
* offer REST API for such control

[More info](doc/README.md)

## How to use it

Prerequisites:

* Tested on linux only
* [VISCA-compatible](doc/hardware.md) camera connected, on, e.g. `/dev/ttyUSB0`
* python3 with pip3 already installed
