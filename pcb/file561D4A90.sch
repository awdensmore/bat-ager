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
Sheet 3 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BC849 Q?
U 1 1 561D53BD
P 4100 4250
F 0 "Q?" H 4300 4325 50  0000 L CNN
F 1 "BC849" H 4300 4250 50  0000 L CNN
F 2 "SOT-23" H 4300 4175 50  0000 L CIN
F 3 "" H 4100 4250 50  0000 L CNN
	1    4100 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q?
U 1 1 561D53C4
P 4200 2350
F 0 "Q?" H 4400 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 4400 2350 50  0000 L CNN
F 2 "TO-220" H 4400 2275 50  0000 L CIN
F 3 "" H 4200 2350 50  0000 L CNN
	1    4200 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D53CB
P 5100 4050
F 0 "R?" V 5180 4050 50  0000 C CNN
F 1 "R" V 5100 4050 50  0000 C CNN
F 2 "" V 5030 4050 30  0000 C CNN
F 3 "" H 5100 4050 30  0000 C CNN
	1    5100 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D53D2
P 5100 3500
F 0 "R?" V 5180 3500 50  0000 C CNN
F 1 "R" V 5100 3500 50  0000 C CNN
F 2 "" V 5030 3500 30  0000 C CNN
F 3 "" H 5100 3500 30  0000 C CNN
	1    5100 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q?
U 1 1 561D53D9
P 3750 1700
F 0 "Q?" H 4000 1775 50  0000 L CNN
F 1 "IRF9530" H 4000 1700 50  0000 L CNN
F 2 "TO-220" H 4000 1625 50  0000 L CIN
F 3 "" H 3750 1700 50  0000 L CNN
	1    3750 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q?
U 1 1 561D53E0
P 2950 1200
F 0 "Q?" H 3150 1275 50  0000 L CNN
F 1 "BC849" H 3150 1200 50  0000 L CNN
F 2 "SOT-23" H 3150 1125 50  0000 L CIN
F 3 "" H 2950 1200 50  0000 L CNN
	1    2950 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561D53E7
P 2150 2100
F 0 "D?" H 2150 2200 50  0000 C CNN
F 1 "LED" H 2150 2000 50  0000 C CNN
F 2 "" H 2150 2100 60  0000 C CNN
F 3 "" H 2150 2100 60  0000 C CNN
	1    2150 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561D53EE
P 4200 3800
F 0 "D?" H 4200 3900 50  0000 C CNN
F 1 "LED" H 4200 3700 50  0000 C CNN
F 2 "" H 4200 3800 60  0000 C CNN
F 3 "" H 4200 3800 60  0000 C CNN
	1    4200 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D53F5
P 4200 3300
F 0 "R?" V 4280 3300 50  0000 C CNN
F 1 "R" V 4200 3300 50  0000 C CNN
F 2 "" V 4130 3300 30  0000 C CNN
F 3 "" H 4200 3300 30  0000 C CNN
	1    4200 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D53FC
P 2150 1600
F 0 "R?" V 2230 1600 50  0000 C CNN
F 1 "R" V 2150 1600 50  0000 C CNN
F 2 "" V 2080 1600 30  0000 C CNN
F 3 "" H 2150 1600 30  0000 C CNN
	1    2150 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5403
P 3700 1050
F 0 "R?" V 3780 1050 50  0000 C CNN
F 1 "R" V 3700 1050 50  0000 C CNN
F 2 "" V 3630 1050 30  0000 C CNN
F 3 "" H 3700 1050 30  0000 C CNN
	1    3700 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561D540A
P 3700 800
F 0 "#PWR?" H 3700 650 50  0001 C CNN
F 1 "+15V" H 3700 940 50  0000 C CNN
F 2 "" H 3700 800 60  0000 C CNN
F 3 "" H 3700 800 60  0000 C CNN
	1    3700 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D5410
P 3050 1450
F 0 "#PWR?" H 3050 1200 50  0001 C CNN
F 1 "GND" H 3050 1300 50  0000 C CNN
F 2 "" H 3050 1450 60  0000 C CNN
F 3 "" H 3050 1450 60  0000 C CNN
	1    3050 1450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5416
P 4750 1800
F 0 "R?" V 4830 1800 50  0000 C CNN
F 1 "R" V 4750 1800 50  0000 C CNN
F 2 "" V 4680 1800 30  0000 C CNN
F 3 "" H 4750 1800 30  0000 C CNN
	1    4750 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D541D
P 4300 2600
F 0 "#PWR?" H 4300 2350 50  0001 C CNN
F 1 "GND" H 4300 2450 50  0000 C CNN
F 2 "" H 4300 2600 60  0000 C CNN
F 3 "" H 4300 2600 60  0000 C CNN
	1    4300 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5423
P 2500 1200
F 0 "R?" V 2580 1200 50  0000 C CNN
F 1 "R" V 2500 1200 50  0000 C CNN
F 2 "" V 2430 1200 30  0000 C CNN
F 3 "" H 2500 1200 30  0000 C CNN
	1    2500 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D542A
P 3750 2400
F 0 "R?" V 3830 2400 50  0000 C CNN
F 1 "R" V 3750 2400 50  0000 C CNN
F 2 "" V 3680 2400 30  0000 C CNN
F 3 "" H 3750 2400 30  0000 C CNN
	1    3750 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D5431
P 4200 4550
F 0 "#PWR?" H 4200 4300 50  0001 C CNN
F 1 "GND" H 4200 4400 50  0000 C CNN
F 2 "" H 4200 4550 60  0000 C CNN
F 3 "" H 4200 4550 60  0000 C CNN
	1    4200 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D5437
P 2150 2300
F 0 "#PWR?" H 2150 2050 50  0001 C CNN
F 1 "GND" H 2150 2150 50  0000 C CNN
F 2 "" H 2150 2300 60  0000 C CNN
F 3 "" H 2150 2300 60  0000 C CNN
	1    2150 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D543D
P 3600 4250
F 0 "R?" V 3680 4250 50  0000 C CNN
F 1 "R" V 3600 4250 50  0000 C CNN
F 2 "" V 3530 4250 30  0000 C CNN
F 3 "" H 3600 4250 30  0000 C CNN
	1    3600 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U?
U 1 1 561D5444
P 2400 3500
F 0 "U?" H 2550 3750 60  0000 L CNN
F 1 "LM358DUAL" H 2400 3000 60  0000 L CNN
F 2 "" H 2400 3500 60  0000 C CNN
F 3 "" H 2400 3500 60  0000 C CNN
	1    2400 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561D544B
P 2400 2850
F 0 "#PWR?" H 2400 2700 50  0001 C CNN
F 1 "+15V" H 2400 2990 50  0000 C CNN
F 2 "" H 2400 2850 60  0000 C CNN
F 3 "" H 2400 2850 60  0000 C CNN
	1    2400 2850
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR?
U 1 1 561D5451
P 2400 4450
F 0 "#PWR?" H 2400 4300 50  0001 C CNN
F 1 "-15V" H 2400 4590 50  0000 C CNN
F 2 "" H 2400 4450 60  0000 C CNN
F 3 "" H 2400 4450 60  0000 C CNN
	1    2400 4450
	-1   0    0    1   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5457
P 1750 3150
F 0 "R?" V 1830 3150 50  0000 C CNN
F 1 "R" V 1750 3150 50  0000 C CNN
F 2 "" V 1680 3150 30  0000 C CNN
F 3 "" H 1750 3150 30  0000 C CNN
	1    1750 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D545E
P 1350 3400
F 0 "R?" V 1430 3400 50  0000 C CNN
F 1 "R" V 1350 3400 50  0000 C CNN
F 2 "" V 1280 3400 30  0000 C CNN
F 3 "" H 1350 3400 30  0000 C CNN
	1    1350 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D5465
P 1100 4100
F 0 "#PWR?" H 1100 3850 50  0001 C CNN
F 1 "GND" H 1100 3950 50  0000 C CNN
F 2 "" H 1100 4100 60  0000 C CNN
F 3 "" H 1100 4100 60  0000 C CNN
	1    1100 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D546B
P 1550 3550
F 0 "R?" V 1630 3550 50  0000 C CNN
F 1 "R" V 1550 3550 50  0000 C CNN
F 2 "" V 1480 3550 30  0000 C CNN
F 3 "" H 1550 3550 30  0000 C CNN
	1    1550 3550
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5472
P 2950 3150
F 0 "R?" V 3030 3150 50  0000 C CNN
F 1 "R" V 2950 3150 50  0000 C CNN
F 2 "" V 2880 3150 30  0000 C CNN
F 3 "" H 2950 3150 30  0000 C CNN
	1    2950 3150
	1    0    0    -1  
$EndComp
Text Label 1450 2900 0    40   ~ 0
shunt3_high
Text Label 750  3550 0    40   ~ 0
shunt3_low
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D547B
P 1600 4100
F 0 "R?" V 1680 4100 50  0000 C CNN
F 1 "R" V 1600 4100 50  0000 C CNN
F 2 "" V 1530 4100 30  0000 C CNN
F 3 "" H 1600 4100 30  0000 C CNN
	1    1600 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D5482
P 1600 4600
F 0 "R?" V 1680 4600 50  0000 C CNN
F 1 "R" V 1600 4600 50  0000 C CNN
F 2 "" V 1530 4600 30  0000 C CNN
F 3 "" H 1600 4600 30  0000 C CNN
	1    1600 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D5489
P 1600 4900
F 0 "#PWR?" H 1600 4650 50  0001 C CNN
F 1 "GND" H 1600 4750 50  0000 C CNN
F 2 "" H 1600 4900 60  0000 C CNN
F 3 "" H 1600 4900 60  0000 C CNN
	1    1600 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 561D548F
P 1600 3850
F 0 "#PWR?" H 1600 3700 50  0001 C CNN
F 1 "VCC" H 1600 4000 50  0000 C CNN
F 2 "" H 1600 3850 60  0000 C CNN
F 3 "" H 1600 3850 60  0000 C CNN
	1    1600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1850 2150 1900
Wire Wire Line
	2150 1350 2150 1200
Wire Wire Line
	4200 4050 4200 4000
Wire Wire Line
	4200 3600 4200 3550
Wire Wire Line
	4200 4450 4200 4550
Wire Wire Line
	3850 4250 3900 4250
Wire Wire Line
	1600 3400 1950 3400
Connection ~ 1750 3400
Wire Wire Line
	1100 3400 1100 4100
Wire Wire Line
	1800 3550 1950 3550
Wire Wire Line
	2950 2900 2950 2650
Wire Wire Line
	2950 2650 1900 2650
Wire Wire Line
	1900 2650 1900 3550
Connection ~ 1900 3550
Wire Wire Line
	2950 3400 2950 4100
Wire Wire Line
	2950 3500 2850 3500
Wire Wire Line
	4300 2550 4300 2600
Wire Wire Line
	3950 1800 4500 1800
Wire Wire Line
	4300 1800 4300 2150
Wire Wire Line
	1750 2900 1450 2900
Wire Wire Line
	1300 3550 750  3550
Wire Wire Line
	2950 4100 1800 4100
Wire Wire Line
	1800 4100 1800 3700
Wire Wire Line
	1800 3700 1950 3700
Connection ~ 2950 3500
Wire Wire Line
	1950 3850 1900 3850
Wire Wire Line
	1900 3850 1900 4350
Wire Wire Line
	1900 4350 1600 4350
Wire Wire Line
	1600 4900 1600 4850
Wire Wire Line
	2850 3700 3200 3700
Wire Wire Line
	4450 1800 4450 1600
Wire Wire Line
	4450 1600 4650 1600
Connection ~ 4450 1800
Text Label 4450 1600 0    40   ~ 0
shunt3_high
Wire Wire Line
	5000 1800 5300 1800
Wire Wire Line
	5050 1800 5050 1600
Wire Wire Line
	5050 1600 5250 1600
Connection ~ 5050 1800
Text Label 5050 1600 0    40   ~ 0
shunt3_low
Connection ~ 4300 1800
Wire Wire Line
	3700 1300 3700 1500
Wire Wire Line
	3700 1400 3500 1400
Wire Wire Line
	3500 1400 3500 1000
Wire Wire Line
	3500 1000 3050 1000
Connection ~ 3700 1400
Wire Wire Line
	3050 1400 3050 1450
Wire Wire Line
	1900 1200 2250 1200
$Comp
L +15V #PWR?
U 1 1 561D54C5
P 3350 1700
F 0 "#PWR?" H 3350 1550 50  0001 C CNN
F 1 "+15V" H 3350 1840 50  0000 C CNN
F 2 "" H 3350 1700 60  0000 C CNN
F 3 "" H 3350 1700 60  0000 C CNN
	1    3350 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 1800 3350 1800
Wire Wire Line
	3350 1800 3350 1700
Connection ~ 2150 1200
Wire Wire Line
	3500 2400 3250 2400
Wire Wire Line
	5100 3800 5100 3750
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D54D3
P 5100 4350
F 0 "#PWR?" H 5100 4100 50  0001 C CNN
F 1 "GND" H 5100 4200 50  0000 C CNN
F 2 "" H 5100 4350 60  0000 C CNN
F 3 "" H 5100 4350 60  0000 C CNN
	1    5100 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4350 5100 4300
Wire Wire Line
	5100 3250 5100 3100
Wire Wire Line
	5100 3100 5250 3100
Wire Wire Line
	5100 3800 5300 3800
Wire Wire Line
	3350 4250 3150 4250
$Comp
L VCC #PWR?
U 1 1 561D54E1
P 4200 3050
F 0 "#PWR?" H 4200 2900 50  0001 C CNN
F 1 "VCC" H 4200 3200 50  0000 C CNN
F 2 "" H 4200 3050 60  0000 C CNN
F 3 "" H 4200 3050 60  0000 C CNN
	1    4200 3050
	1    0    0    -1  
$EndComp
Text Notes 1100 1050 2    60   ~ 0
AI      2\nDO     1\nPWM   2
Wire Notes Line
	600  550  600  5250
Wire Notes Line
	600  5250 5700 5250
Wire Notes Line
	5700 5250 5700 550 
Wire Notes Line
	5700 550  600  550 
Text Label 650  700  0    80   ~ 0
BATTERY3
Text HLabel 1900 1200 0    40   Input ~ 0
chg_ctrl3
Text HLabel 3250 2400 0    40   Input ~ 0
dchg_ctrl3
Text HLabel 5300 1800 2    40   UnSpc ~ 0
bat3+
Text HLabel 3200 3700 2    40   Output ~ 0
i3
Text HLabel 3150 4250 0    40   Input ~ 0
dc_led3
Text HLabel 5300 3800 2    40   Output ~ 0
vbat3
Text HLabel 5250 3100 2    40   UnSpc ~ 0
bat3+
$Comp
L BC849 Q?
U 1 1 561D8451
P 9400 4250
F 0 "Q?" H 9600 4325 50  0000 L CNN
F 1 "BC849" H 9600 4250 50  0000 L CNN
F 2 "SOT-23" H 9600 4175 50  0000 L CIN
F 3 "" H 9400 4250 50  0000 L CNN
	1    9400 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q?
U 1 1 561D8457
P 9500 2350
F 0 "Q?" H 9700 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 9700 2350 50  0000 L CNN
F 2 "TO-220" H 9700 2275 50  0000 L CIN
F 3 "" H 9500 2350 50  0000 L CNN
	1    9500 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D845D
P 10400 4050
F 0 "R?" V 10480 4050 50  0000 C CNN
F 1 "R" V 10400 4050 50  0000 C CNN
F 2 "" V 10330 4050 30  0000 C CNN
F 3 "" H 10400 4050 30  0000 C CNN
	1    10400 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D8463
P 10400 3500
F 0 "R?" V 10480 3500 50  0000 C CNN
F 1 "R" V 10400 3500 50  0000 C CNN
F 2 "" V 10330 3500 30  0000 C CNN
F 3 "" H 10400 3500 30  0000 C CNN
	1    10400 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q?
U 1 1 561D8469
P 9050 1700
F 0 "Q?" H 9300 1775 50  0000 L CNN
F 1 "IRF9530" H 9300 1700 50  0000 L CNN
F 2 "TO-220" H 9300 1625 50  0000 L CIN
F 3 "" H 9050 1700 50  0000 L CNN
	1    9050 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q?
U 1 1 561D846F
P 8250 1200
F 0 "Q?" H 8450 1275 50  0000 L CNN
F 1 "BC849" H 8450 1200 50  0000 L CNN
F 2 "SOT-23" H 8450 1125 50  0000 L CIN
F 3 "" H 8250 1200 50  0000 L CNN
	1    8250 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561D8475
P 7450 2100
F 0 "D?" H 7450 2200 50  0000 C CNN
F 1 "LED" H 7450 2000 50  0000 C CNN
F 2 "" H 7450 2100 60  0000 C CNN
F 3 "" H 7450 2100 60  0000 C CNN
	1    7450 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561D847B
P 9500 3800
F 0 "D?" H 9500 3900 50  0000 C CNN
F 1 "LED" H 9500 3700 50  0000 C CNN
F 2 "" H 9500 3800 60  0000 C CNN
F 3 "" H 9500 3800 60  0000 C CNN
	1    9500 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D8481
P 9500 3300
F 0 "R?" V 9580 3300 50  0000 C CNN
F 1 "R" V 9500 3300 50  0000 C CNN
F 2 "" V 9430 3300 30  0000 C CNN
F 3 "" H 9500 3300 30  0000 C CNN
	1    9500 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D8487
P 7450 1600
F 0 "R?" V 7530 1600 50  0000 C CNN
F 1 "R" V 7450 1600 50  0000 C CNN
F 2 "" V 7380 1600 30  0000 C CNN
F 3 "" H 7450 1600 30  0000 C CNN
	1    7450 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D848D
P 9000 1050
F 0 "R?" V 9080 1050 50  0000 C CNN
F 1 "R" V 9000 1050 50  0000 C CNN
F 2 "" V 8930 1050 30  0000 C CNN
F 3 "" H 9000 1050 30  0000 C CNN
	1    9000 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561D8493
P 9000 800
F 0 "#PWR?" H 9000 650 50  0001 C CNN
F 1 "+15V" H 9000 940 50  0000 C CNN
F 2 "" H 9000 800 60  0000 C CNN
F 3 "" H 9000 800 60  0000 C CNN
	1    9000 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D8499
P 8350 1450
F 0 "#PWR?" H 8350 1200 50  0001 C CNN
F 1 "GND" H 8350 1300 50  0000 C CNN
F 2 "" H 8350 1450 60  0000 C CNN
F 3 "" H 8350 1450 60  0000 C CNN
	1    8350 1450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D849F
P 10050 1800
F 0 "R?" V 10130 1800 50  0000 C CNN
F 1 "R" V 10050 1800 50  0000 C CNN
F 2 "" V 9980 1800 30  0000 C CNN
F 3 "" H 10050 1800 30  0000 C CNN
	1    10050 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D84A5
P 9600 2600
F 0 "#PWR?" H 9600 2350 50  0001 C CNN
F 1 "GND" H 9600 2450 50  0000 C CNN
F 2 "" H 9600 2600 60  0000 C CNN
F 3 "" H 9600 2600 60  0000 C CNN
	1    9600 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84AB
P 7800 1200
F 0 "R?" V 7880 1200 50  0000 C CNN
F 1 "R" V 7800 1200 50  0000 C CNN
F 2 "" V 7730 1200 30  0000 C CNN
F 3 "" H 7800 1200 30  0000 C CNN
	1    7800 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84B1
P 9050 2400
F 0 "R?" V 9130 2400 50  0000 C CNN
F 1 "R" V 9050 2400 50  0000 C CNN
F 2 "" V 8980 2400 30  0000 C CNN
F 3 "" H 9050 2400 30  0000 C CNN
	1    9050 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D84B7
P 9500 4550
F 0 "#PWR?" H 9500 4300 50  0001 C CNN
F 1 "GND" H 9500 4400 50  0000 C CNN
F 2 "" H 9500 4550 60  0000 C CNN
F 3 "" H 9500 4550 60  0000 C CNN
	1    9500 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D84BD
P 7450 2300
F 0 "#PWR?" H 7450 2050 50  0001 C CNN
F 1 "GND" H 7450 2150 50  0000 C CNN
F 2 "" H 7450 2300 60  0000 C CNN
F 3 "" H 7450 2300 60  0000 C CNN
	1    7450 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84C3
P 8900 4250
F 0 "R?" V 8980 4250 50  0000 C CNN
F 1 "R" V 8900 4250 50  0000 C CNN
F 2 "" V 8830 4250 30  0000 C CNN
F 3 "" H 8900 4250 30  0000 C CNN
	1    8900 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U?
U 1 1 561D84C9
P 7700 3500
F 0 "U?" H 7850 3750 60  0000 L CNN
F 1 "LM358DUAL" H 7700 3000 60  0000 L CNN
F 2 "" H 7700 3500 60  0000 C CNN
F 3 "" H 7700 3500 60  0000 C CNN
	1    7700 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561D84CF
P 7700 2750
F 0 "#PWR?" H 7700 2600 50  0001 C CNN
F 1 "+15V" H 7700 2890 50  0000 C CNN
F 2 "" H 7700 2750 60  0000 C CNN
F 3 "" H 7700 2750 60  0000 C CNN
	1    7700 2750
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR?
U 1 1 561D84D5
P 7700 4400
F 0 "#PWR?" H 7700 4250 50  0001 C CNN
F 1 "-15V" H 7700 4540 50  0000 C CNN
F 2 "" H 7700 4400 60  0000 C CNN
F 3 "" H 7700 4400 60  0000 C CNN
	1    7700 4400
	-1   0    0    1   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84DB
P 7050 3150
F 0 "R?" V 7130 3150 50  0000 C CNN
F 1 "R" V 7050 3150 50  0000 C CNN
F 2 "" V 6980 3150 30  0000 C CNN
F 3 "" H 7050 3150 30  0000 C CNN
	1    7050 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84E1
P 6650 3400
F 0 "R?" V 6730 3400 50  0000 C CNN
F 1 "R" V 6650 3400 50  0000 C CNN
F 2 "" V 6580 3400 30  0000 C CNN
F 3 "" H 6650 3400 30  0000 C CNN
	1    6650 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D84E7
P 6400 4100
F 0 "#PWR?" H 6400 3850 50  0001 C CNN
F 1 "GND" H 6400 3950 50  0000 C CNN
F 2 "" H 6400 4100 60  0000 C CNN
F 3 "" H 6400 4100 60  0000 C CNN
	1    6400 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84ED
P 6850 3550
F 0 "R?" V 6930 3550 50  0000 C CNN
F 1 "R" V 6850 3550 50  0000 C CNN
F 2 "" V 6780 3550 30  0000 C CNN
F 3 "" H 6850 3550 30  0000 C CNN
	1    6850 3550
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84F3
P 8250 3150
F 0 "R?" V 8330 3150 50  0000 C CNN
F 1 "R" V 8250 3150 50  0000 C CNN
F 2 "" V 8180 3150 30  0000 C CNN
F 3 "" H 8250 3150 30  0000 C CNN
	1    8250 3150
	1    0    0    -1  
$EndComp
Text Label 6750 2900 0    40   ~ 0
shunt4_high
Text Label 6050 3550 0    40   ~ 0
shunt4_low
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D84FB
P 6900 4100
F 0 "R?" V 6980 4100 50  0000 C CNN
F 1 "R" V 6900 4100 50  0000 C CNN
F 2 "" V 6830 4100 30  0000 C CNN
F 3 "" H 6900 4100 30  0000 C CNN
	1    6900 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561D8501
P 6900 4600
F 0 "R?" V 6980 4600 50  0000 C CNN
F 1 "R" V 6900 4600 50  0000 C CNN
F 2 "" V 6830 4600 30  0000 C CNN
F 3 "" H 6900 4600 30  0000 C CNN
	1    6900 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D8507
P 6900 4900
F 0 "#PWR?" H 6900 4650 50  0001 C CNN
F 1 "GND" H 6900 4750 50  0000 C CNN
F 2 "" H 6900 4900 60  0000 C CNN
F 3 "" H 6900 4900 60  0000 C CNN
	1    6900 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 561D850D
P 6900 3850
F 0 "#PWR?" H 6900 3700 50  0001 C CNN
F 1 "VCC" H 6900 4000 50  0000 C CNN
F 2 "" H 6900 3850 60  0000 C CNN
F 3 "" H 6900 3850 60  0000 C CNN
	1    6900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 1850 7450 1900
Wire Wire Line
	7450 1350 7450 1200
Wire Wire Line
	9500 4050 9500 4000
Wire Wire Line
	9500 3600 9500 3550
Wire Wire Line
	9500 4450 9500 4550
Wire Wire Line
	9150 4250 9200 4250
Wire Wire Line
	6900 3400 7250 3400
Connection ~ 7050 3400
Wire Wire Line
	6400 3400 6400 4100
Wire Wire Line
	7100 3550 7250 3550
Wire Wire Line
	8250 2900 8250 2550
Wire Wire Line
	8250 2550 7200 2550
Wire Wire Line
	7200 2550 7200 3550
Connection ~ 7200 3550
Wire Wire Line
	8250 3400 8250 4100
Wire Wire Line
	8250 3500 8150 3500
Wire Wire Line
	9600 2550 9600 2600
Wire Wire Line
	9250 1800 9800 1800
Wire Wire Line
	9600 1800 9600 2150
Wire Wire Line
	7050 2900 6750 2900
Wire Wire Line
	6600 3550 6050 3550
Wire Wire Line
	8250 4100 7100 4100
Wire Wire Line
	7100 4100 7100 3700
Wire Wire Line
	7100 3700 7250 3700
Connection ~ 8250 3500
Wire Wire Line
	7250 3850 7200 3850
Wire Wire Line
	7200 3850 7200 4350
Wire Wire Line
	7200 4350 6900 4350
Wire Wire Line
	6900 4900 6900 4850
Wire Wire Line
	8150 3700 8500 3700
Wire Wire Line
	9750 1800 9750 1600
Wire Wire Line
	9750 1600 9950 1600
Connection ~ 9750 1800
Text Label 9750 1600 0    40   ~ 0
shunt4_high
Wire Wire Line
	10300 1800 10600 1800
Wire Wire Line
	10350 1800 10350 1600
Wire Wire Line
	10350 1600 10550 1600
Connection ~ 10350 1800
Text Label 10350 1600 0    40   ~ 0
shunt4_low
Connection ~ 9600 1800
Wire Wire Line
	9000 1300 9000 1500
Wire Wire Line
	9000 1400 8800 1400
Wire Wire Line
	8800 1400 8800 1000
Wire Wire Line
	8800 1000 8350 1000
Connection ~ 9000 1400
Wire Wire Line
	8350 1400 8350 1450
Wire Wire Line
	7200 1200 7550 1200
$Comp
L +15V #PWR?
U 1 1 561D8542
P 8650 1700
F 0 "#PWR?" H 8650 1550 50  0001 C CNN
F 1 "+15V" H 8650 1840 50  0000 C CNN
F 2 "" H 8650 1700 60  0000 C CNN
F 3 "" H 8650 1700 60  0000 C CNN
	1    8650 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1800 8650 1800
Wire Wire Line
	8650 1800 8650 1700
Connection ~ 7450 1200
Wire Wire Line
	8800 2400 8550 2400
Wire Wire Line
	10400 3800 10400 3750
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561D854D
P 10400 4350
F 0 "#PWR?" H 10400 4100 50  0001 C CNN
F 1 "GND" H 10400 4200 50  0000 C CNN
F 2 "" H 10400 4350 60  0000 C CNN
F 3 "" H 10400 4350 60  0000 C CNN
	1    10400 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 4350 10400 4300
Wire Wire Line
	10400 3250 10400 3100
Wire Wire Line
	10400 3100 10550 3100
Wire Wire Line
	10400 3800 10600 3800
Wire Wire Line
	8650 4250 8450 4250
$Comp
L VCC #PWR?
U 1 1 561D8558
P 9500 3050
F 0 "#PWR?" H 9500 2900 50  0001 C CNN
F 1 "VCC" H 9500 3200 50  0000 C CNN
F 2 "" H 9500 3050 60  0000 C CNN
F 3 "" H 9500 3050 60  0000 C CNN
	1    9500 3050
	1    0    0    -1  
$EndComp
Text Notes 6400 1050 2    60   ~ 0
AI      2\nDO     1\nPWM   2
Wire Notes Line
	5900 550  5900 5250
Wire Notes Line
	5900 5250 11000 5250
Wire Notes Line
	11000 5250 11000 550 
Wire Notes Line
	11000 550  5900 550 
Text Label 5950 700  0    80   ~ 0
BATTERY4
Text HLabel 7200 1200 0    40   Input ~ 0
chg_ctrl4
Text HLabel 8550 2400 0    40   Input ~ 0
dchg_ctrl4
Text HLabel 10600 1800 2    40   UnSpc ~ 0
bat4+
Text HLabel 8500 3700 2    40   Output ~ 0
i4
Text HLabel 8450 4250 0    40   Input ~ 0
dc_led4
Text HLabel 10600 3800 2    40   Output ~ 0
vbat4
Text HLabel 10550 3100 2    40   UnSpc ~ 0
bat4+
Text HLabel 1150 5600 0    80   UnSpc ~ 0
+15V
$Comp
L +15V #PWR?
U 1 1 561DB3B7
P 1400 5500
F 0 "#PWR?" H 1400 5350 50  0001 C CNN
F 1 "+15V" H 1400 5640 50  0000 C CNN
F 2 "" H 1400 5500 60  0000 C CNN
F 3 "" H 1400 5500 60  0000 C CNN
	1    1400 5500
	1    0    0    -1  
$EndComp
Text HLabel 1150 5800 0    80   UnSpc ~ 0
GND
Text HLabel 1150 6000 0    80   UnSpc ~ 0
-15V
Text HLabel 1150 6400 0    80   UnSpc ~ 0
VCC
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DB84A
P 1800 5900
F 0 "#PWR?" H 1800 5650 50  0001 C CNN
F 1 "GND" H 1800 5750 50  0000 C CNN
F 2 "" H 1800 5900 60  0000 C CNN
F 3 "" H 1800 5900 60  0000 C CNN
	1    1800 5900
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR?
U 1 1 561DB8D9
P 1600 6000
F 0 "#PWR?" H 1600 5850 50  0001 C CNN
F 1 "-15V" H 1600 6140 50  0000 C CNN
F 2 "" H 1600 6000 60  0000 C CNN
F 3 "" H 1600 6000 60  0000 C CNN
	1    1600 6000
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR?
U 1 1 561DB968
P 1300 6250
F 0 "#PWR?" H 1300 6100 50  0001 C CNN
F 1 "VCC" H 1300 6400 50  0000 C CNN
F 2 "" H 1300 6250 60  0000 C CNN
F 3 "" H 1300 6250 60  0000 C CNN
	1    1300 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 5600 1400 5600
Wire Wire Line
	1400 5600 1400 5500
Wire Wire Line
	1800 5900 1800 5800
Wire Wire Line
	1800 5800 1150 5800
Wire Wire Line
	1600 6000 1150 6000
Wire Wire Line
	1300 6250 1300 6400
Wire Wire Line
	1300 6400 1150 6400
$Comp
L CONN_01X02 P?
U 1 1 561E0FA1
P 3400 5950
F 0 "P?" H 3400 6100 50  0000 C CNN
F 1 "CONN_01X02" V 3500 5950 50  0000 C CNN
F 2 "" H 3400 5950 60  0000 C CNN
F 3 "" H 3400 5950 60  0000 C CNN
	1    3400 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 5900 2800 5900
Text HLabel 2800 5900 0    80   UnSpc ~ 0
bat3+
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E128A
P 3050 6150
F 0 "#PWR?" H 3050 5900 50  0001 C CNN
F 1 "GND" H 3050 6000 50  0000 C CNN
F 2 "" H 3050 6150 60  0000 C CNN
F 3 "" H 3050 6150 60  0000 C CNN
	1    3050 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6000 3050 6000
Wire Wire Line
	3050 6000 3050 6150
$Comp
L CONN_01X02 P?
U 1 1 561E13E4
P 3400 6800
F 0 "P?" H 3400 6950 50  0000 C CNN
F 1 "CONN_01X02" V 3500 6800 50  0000 C CNN
F 2 "" H 3400 6800 60  0000 C CNN
F 3 "" H 3400 6800 60  0000 C CNN
	1    3400 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6750 2800 6750
Text HLabel 2800 6750 0    80   UnSpc ~ 0
bat4+
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E13EC
P 3050 7000
F 0 "#PWR?" H 3050 6750 50  0001 C CNN
F 1 "GND" H 3050 6850 50  0000 C CNN
F 2 "" H 3050 7000 60  0000 C CNN
F 3 "" H 3050 7000 60  0000 C CNN
	1    3050 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6850 3050 6850
Wire Wire Line
	3050 6850 3050 7000
$Comp
L C_Small C?
U 1 1 561E409B
P 2100 4450
F 0 "C?" H 2110 4520 50  0000 L CNN
F 1 "0.1uF" H 2110 4370 50  0000 L CNN
F 2 "" H 2100 4450 60  0000 C CNN
F 3 "" H 2100 4450 60  0000 C CNN
	1    2100 4450
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E41E4
P 2100 4550
F 0 "#PWR?" H 2100 4300 50  0001 C CNN
F 1 "GND" H 2100 4400 50  0000 C CNN
F 2 "" H 2100 4550 60  0000 C CNN
F 3 "" H 2100 4550 60  0000 C CNN
	1    2100 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4200 2400 4450
Wire Wire Line
	2100 4350 2100 4300
Wire Wire Line
	2100 4300 2400 4300
Connection ~ 2400 4300
$Comp
L C_Small C?
U 1 1 561E4484
P 2100 2950
F 0 "C?" H 2110 3020 50  0000 L CNN
F 1 "0.1uF" H 2110 2870 50  0000 L CNN
F 2 "" H 2100 2950 60  0000 C CNN
F 3 "" H 2100 2950 60  0000 C CNN
	1    2100 2950
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E448A
P 2100 3050
F 0 "#PWR?" H 2100 2800 50  0001 C CNN
F 1 "GND" H 2100 2900 50  0000 C CNN
F 2 "" H 2100 3050 60  0000 C CNN
F 3 "" H 2100 3050 60  0000 C CNN
	1    2100 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2850 2400 3050
Wire Wire Line
	2100 2850 2300 2850
Wire Wire Line
	2300 2850 2300 2950
Wire Wire Line
	2300 2950 2400 2950
Connection ~ 2400 2950
$Comp
L C_Small C?
U 1 1 561E49EF
P 7400 2950
F 0 "C?" H 7410 3020 50  0000 L CNN
F 1 "0.1uF" H 7410 2870 50  0000 L CNN
F 2 "" H 7400 2950 60  0000 C CNN
F 3 "" H 7400 2950 60  0000 C CNN
	1    7400 2950
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E49F5
P 7400 3050
F 0 "#PWR?" H 7400 2800 50  0001 C CNN
F 1 "GND" H 7400 2900 50  0000 C CNN
F 2 "" H 7400 3050 60  0000 C CNN
F 3 "" H 7400 3050 60  0000 C CNN
	1    7400 3050
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 561E4C02
P 7400 4600
F 0 "C?" H 7410 4670 50  0000 L CNN
F 1 "0.1uF" H 7410 4520 50  0000 L CNN
F 2 "" H 7400 4600 60  0000 C CNN
F 3 "" H 7400 4600 60  0000 C CNN
	1    7400 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E4C08
P 7400 4700
F 0 "#PWR?" H 7400 4450 50  0001 C CNN
F 1 "GND" H 7400 4550 50  0000 C CNN
F 2 "" H 7400 4700 60  0000 C CNN
F 3 "" H 7400 4700 60  0000 C CNN
	1    7400 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2750 7700 3050
Wire Wire Line
	7400 2850 7700 2850
Connection ~ 7700 2850
Wire Wire Line
	7700 4200 7700 4400
Wire Wire Line
	7400 4500 7400 4300
Wire Wire Line
	7400 4300 7700 4300
Connection ~ 7700 4300
$EndSCHEMATC