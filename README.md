# bat-ager

This project is an automated hardware and software tool for aging batteries in a precise fashion.

**HARDWARE**

The aging process is done by repeatedly charging and discharging the battery. The hardware therefore switches between a charging mode and a discharging mode. Charging is done using a PWM controlled fet. Discharging is done by carefully controlling a fet that shorts the battery to ground. Current is measured using a shunt and op-amp. The PCB is divided into five charging sections so that multiple batteries may be aged simultaneously. The MCU selection and supporting hardware is based on the Arduino Mega 2560. There is also reverse voltage protection on the power supply input (should probably add that to the battery terminals as well...).

**FIRMWARE**

The firmware is Arduino based to make development easy (though I use the [Arduino makefile](https://github.com/sudar/Arduino-Makefile) to make customizing and automation easier in Linux). Features will include:
+ Independent charge/discharge settings for each battery
+ Serial communication via FTDI cable. With a Raspberry Pi attached, can be monitored via SSH.
+ Safeties ?!
