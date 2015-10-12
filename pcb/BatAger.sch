EESchema Schematic File Version 2
LIBS:BatAger-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:BatAger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title ""
Date "12 oct 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LM7805 U?
U 1 1 5617C63B
P 5750 11450
F 0 "U?" H 5900 11254 60  0000 C CNN
F 1 "LM7805" H 5750 11650 60  0000 C CNN
F 2 "" H 5750 11450 60  0000 C CNN
F 3 "" H 5750 11450 60  0000 C CNN
	1    5750 11450
	1    0    0    -1  
$EndComp
$Comp
L LM7805 U?
U 1 1 561C82D6
P 3400 13650
F 0 "U?" H 3550 13454 60  0000 C CNN
F 1 "LM7805" H 3400 13850 60  0000 C CNN
F 2 "" H 3400 13650 60  0000 C CNN
F 3 "" H 3400 13650 60  0000 C CNN
	1    3400 13650
	1    0    0    -1  
$EndComp
$Sheet
S 9050 1050 1550 1450
U 561CB83B
F0 "Sheet561CB83A" 80
F1 "file561CB83A.sch" 80
F2 "chg_ctrl1" I R 10600 1150 80 
F3 "dchg_ctrl1" I R 10600 1300 80 
F4 "dc_led1" I R 10600 1450 80 
F5 "vbat1" O R 10600 1600 80 
F6 "bat1+" U R 10600 1750 80 
F7 "i1" O R 10600 1900 80 
F8 "+15V" U R 10600 2050 80 
F9 "GND" U R 10600 2200 80 
F10 "-15V" U R 10600 2350 80 
F11 "chg_ctrl2" I L 9050 1150 80 
F12 "dchg_ctrl2" I L 9050 1300 80 
F13 "dc_led2" I L 9050 1450 80 
F14 "vbat2" O L 9050 1600 80 
F15 "bat2+" U L 9050 1750 80 
F16 "i2" O L 9050 1900 80 
$EndSheet
$Sheet
S 9200 2950 1400 1400
U 561D4A91
F0 "Sheet561D4A90" 80
F1 "file561D4A90.sch" 80
$EndSheet
$Comp
L ATMEGA2560-A IC?
U 1 1 561D57F5
P 3900 3800
F 0 "IC?" H 2750 6600 40  0000 L BNN
F 1 "ATMEGA2560-A" H 4600 950 40  0000 L BNN
F 2 "TQFP100" H 3900 3800 30  0000 C CIN
F 3 "" H 3900 3800 60  0000 C CNN
	1    3900 3800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
