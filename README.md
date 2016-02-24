# bat-ager

This project is an automated hardware and software tool for aging batteries in a precise fashion. The repo includes both hardware schematics and firmware.

**HARDWARE**

The aging process is done by repeatedly charging and discharging the battery. The hardware therefore switches between a charging mode and a discharging mode. Charging is done using a bi-directional DC/DC converter powered by a benchtop supply. Discharging is done via a DC/DC converter, carefully controlling a fet that shorts the battery to ground. Current is measured using a shunt and op-amp. The PCB is divided into five charging sections so that multiple batteries may be aged simultaneously. The system is controlled using an STM32F0 MCU. Also supports injecting a small AC ripple voltage during charging and discharging, which is used to measure the battery's impedance in real time.

**FIRMWARE**

The firmware is based on the STM HAL libraries. Features include: 
+ Independent charge/discharge settings for each battery
+ Constant current / constant voltage charging, low-voltage disconnect
+ Programmable load for discharging
+ Injection of small AC ripple voltage
+ Data collection and reporting

~~The firmware is Arduino based to make development easy (though I use the [Arduino makefile](https://github.com/sudar/Arduino-Makefile) to make customizing and automation easier in Linux). Features will include:~~
~~+ Independent charge/discharge settings for each battery~~
~~+ Serial communication via FTDI cable. With a Raspberry Pi attached, can be monitored via SSH.~~
~~+ Safeties ?!~~
