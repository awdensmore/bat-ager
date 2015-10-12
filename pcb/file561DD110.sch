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
Sheet 4 4
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
U 1 1 561DD709
P 4150 4250
F 0 "Q?" H 4350 4325 50  0000 L CNN
F 1 "BC849" H 4350 4250 50  0000 L CNN
F 2 "SOT-23" H 4350 4175 50  0000 L CIN
F 3 "" H 4150 4250 50  0000 L CNN
	1    4150 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q?
U 1 1 561DD710
P 4250 2350
F 0 "Q?" H 4450 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 4450 2350 50  0000 L CNN
F 2 "TO-220" H 4450 2275 50  0000 L CIN
F 3 "" H 4250 2350 50  0000 L CNN
	1    4250 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD717
P 5150 4050
F 0 "R?" V 5230 4050 50  0000 C CNN
F 1 "R" V 5150 4050 50  0000 C CNN
F 2 "" V 5080 4050 30  0000 C CNN
F 3 "" H 5150 4050 30  0000 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD71E
P 5150 3500
F 0 "R?" V 5230 3500 50  0000 C CNN
F 1 "R" V 5150 3500 50  0000 C CNN
F 2 "" V 5080 3500 30  0000 C CNN
F 3 "" H 5150 3500 30  0000 C CNN
	1    5150 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q?
U 1 1 561DD725
P 3800 1700
F 0 "Q?" H 4050 1775 50  0000 L CNN
F 1 "IRF9530" H 4050 1700 50  0000 L CNN
F 2 "TO-220" H 4050 1625 50  0000 L CIN
F 3 "" H 3800 1700 50  0000 L CNN
	1    3800 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q?
U 1 1 561DD72C
P 3000 1200
F 0 "Q?" H 3200 1275 50  0000 L CNN
F 1 "BC849" H 3200 1200 50  0000 L CNN
F 2 "SOT-23" H 3200 1125 50  0000 L CIN
F 3 "" H 3000 1200 50  0000 L CNN
	1    3000 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561DD733
P 2200 2100
F 0 "D?" H 2200 2200 50  0000 C CNN
F 1 "LED" H 2200 2000 50  0000 C CNN
F 2 "" H 2200 2100 60  0000 C CNN
F 3 "" H 2200 2100 60  0000 C CNN
	1    2200 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D?
U 1 1 561DD73A
P 4250 3800
F 0 "D?" H 4250 3900 50  0000 C CNN
F 1 "LED" H 4250 3700 50  0000 C CNN
F 2 "" H 4250 3800 60  0000 C CNN
F 3 "" H 4250 3800 60  0000 C CNN
	1    4250 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD741
P 4250 3300
F 0 "R?" V 4330 3300 50  0000 C CNN
F 1 "R" V 4250 3300 50  0000 C CNN
F 2 "" V 4180 3300 30  0000 C CNN
F 3 "" H 4250 3300 30  0000 C CNN
	1    4250 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD748
P 2200 1600
F 0 "R?" V 2280 1600 50  0000 C CNN
F 1 "R" V 2200 1600 50  0000 C CNN
F 2 "" V 2130 1600 30  0000 C CNN
F 3 "" H 2200 1600 30  0000 C CNN
	1    2200 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD74F
P 3750 1050
F 0 "R?" V 3830 1050 50  0000 C CNN
F 1 "R" V 3750 1050 50  0000 C CNN
F 2 "" V 3680 1050 30  0000 C CNN
F 3 "" H 3750 1050 30  0000 C CNN
	1    3750 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561DD756
P 3750 800
F 0 "#PWR?" H 3750 650 50  0001 C CNN
F 1 "+15V" H 3750 940 50  0000 C CNN
F 2 "" H 3750 800 60  0000 C CNN
F 3 "" H 3750 800 60  0000 C CNN
	1    3750 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD75C
P 3100 1450
F 0 "#PWR?" H 3100 1200 50  0001 C CNN
F 1 "GND" H 3100 1300 50  0000 C CNN
F 2 "" H 3100 1450 60  0000 C CNN
F 3 "" H 3100 1450 60  0000 C CNN
	1    3100 1450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD762
P 4800 1800
F 0 "R?" V 4880 1800 50  0000 C CNN
F 1 "R" V 4800 1800 50  0000 C CNN
F 2 "" V 4730 1800 30  0000 C CNN
F 3 "" H 4800 1800 30  0000 C CNN
	1    4800 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD769
P 4350 2600
F 0 "#PWR?" H 4350 2350 50  0001 C CNN
F 1 "GND" H 4350 2450 50  0000 C CNN
F 2 "" H 4350 2600 60  0000 C CNN
F 3 "" H 4350 2600 60  0000 C CNN
	1    4350 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD76F
P 2550 1200
F 0 "R?" V 2630 1200 50  0000 C CNN
F 1 "R" V 2550 1200 50  0000 C CNN
F 2 "" V 2480 1200 30  0000 C CNN
F 3 "" H 2550 1200 30  0000 C CNN
	1    2550 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD776
P 3800 2400
F 0 "R?" V 3880 2400 50  0000 C CNN
F 1 "R" V 3800 2400 50  0000 C CNN
F 2 "" V 3730 2400 30  0000 C CNN
F 3 "" H 3800 2400 30  0000 C CNN
	1    3800 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD77D
P 4250 4550
F 0 "#PWR?" H 4250 4300 50  0001 C CNN
F 1 "GND" H 4250 4400 50  0000 C CNN
F 2 "" H 4250 4550 60  0000 C CNN
F 3 "" H 4250 4550 60  0000 C CNN
	1    4250 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD783
P 2200 2300
F 0 "#PWR?" H 2200 2050 50  0001 C CNN
F 1 "GND" H 2200 2150 50  0000 C CNN
F 2 "" H 2200 2300 60  0000 C CNN
F 3 "" H 2200 2300 60  0000 C CNN
	1    2200 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD789
P 3650 4250
F 0 "R?" V 3730 4250 50  0000 C CNN
F 1 "R" V 3650 4250 50  0000 C CNN
F 2 "" V 3580 4250 30  0000 C CNN
F 3 "" H 3650 4250 30  0000 C CNN
	1    3650 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U?
U 1 1 561DD790
P 2450 3500
F 0 "U?" H 2600 3750 60  0000 L CNN
F 1 "LM358DUAL" H 2450 3000 60  0000 L CNN
F 2 "" H 2450 3500 60  0000 C CNN
F 3 "" H 2450 3500 60  0000 C CNN
	1    2450 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR?
U 1 1 561DD797
P 2450 2750
F 0 "#PWR?" H 2450 2600 50  0001 C CNN
F 1 "+15V" H 2450 2890 50  0000 C CNN
F 2 "" H 2450 2750 60  0000 C CNN
F 3 "" H 2450 2750 60  0000 C CNN
	1    2450 2750
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR?
U 1 1 561DD79D
P 2450 4400
F 0 "#PWR?" H 2450 4250 50  0001 C CNN
F 1 "-15V" H 2450 4540 50  0000 C CNN
F 2 "" H 2450 4400 60  0000 C CNN
F 3 "" H 2450 4400 60  0000 C CNN
	1    2450 4400
	-1   0    0    1   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7A3
P 1800 3150
F 0 "R?" V 1880 3150 50  0000 C CNN
F 1 "R" V 1800 3150 50  0000 C CNN
F 2 "" V 1730 3150 30  0000 C CNN
F 3 "" H 1800 3150 30  0000 C CNN
	1    1800 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7AA
P 1400 3400
F 0 "R?" V 1480 3400 50  0000 C CNN
F 1 "R" V 1400 3400 50  0000 C CNN
F 2 "" V 1330 3400 30  0000 C CNN
F 3 "" H 1400 3400 30  0000 C CNN
	1    1400 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD7B1
P 1150 4100
F 0 "#PWR?" H 1150 3850 50  0001 C CNN
F 1 "GND" H 1150 3950 50  0000 C CNN
F 2 "" H 1150 4100 60  0000 C CNN
F 3 "" H 1150 4100 60  0000 C CNN
	1    1150 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7B7
P 1600 3550
F 0 "R?" V 1680 3550 50  0000 C CNN
F 1 "R" V 1600 3550 50  0000 C CNN
F 2 "" V 1530 3550 30  0000 C CNN
F 3 "" H 1600 3550 30  0000 C CNN
	1    1600 3550
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7BE
P 3000 3150
F 0 "R?" V 3080 3150 50  0000 C CNN
F 1 "R" V 3000 3150 50  0000 C CNN
F 2 "" V 2930 3150 30  0000 C CNN
F 3 "" H 3000 3150 30  0000 C CNN
	1    3000 3150
	1    0    0    -1  
$EndComp
Text Label 1500 2900 0    40   ~ 0
shunt5_high
Text Label 800  3550 0    40   ~ 0
shunt5_low
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7C7
P 1650 4100
F 0 "R?" V 1730 4100 50  0000 C CNN
F 1 "R" V 1650 4100 50  0000 C CNN
F 2 "" V 1580 4100 30  0000 C CNN
F 3 "" H 1650 4100 30  0000 C CNN
	1    1650 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R?
U 1 1 561DD7CE
P 1650 4600
F 0 "R?" V 1730 4600 50  0000 C CNN
F 1 "R" V 1650 4600 50  0000 C CNN
F 2 "" V 1580 4600 30  0000 C CNN
F 3 "" H 1650 4600 30  0000 C CNN
	1    1650 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD7D5
P 1650 4900
F 0 "#PWR?" H 1650 4650 50  0001 C CNN
F 1 "GND" H 1650 4750 50  0000 C CNN
F 2 "" H 1650 4900 60  0000 C CNN
F 3 "" H 1650 4900 60  0000 C CNN
	1    1650 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 561DD7DB
P 1650 3850
F 0 "#PWR?" H 1650 3700 50  0001 C CNN
F 1 "VCC" H 1650 4000 50  0000 C CNN
F 2 "" H 1650 3850 60  0000 C CNN
F 3 "" H 1650 3850 60  0000 C CNN
	1    1650 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1850 2200 1900
Wire Wire Line
	2200 1350 2200 1200
Wire Wire Line
	4250 4050 4250 4000
Wire Wire Line
	4250 3600 4250 3550
Wire Wire Line
	4250 4450 4250 4550
Wire Wire Line
	3900 4250 3950 4250
Wire Wire Line
	1650 3400 2000 3400
Connection ~ 1800 3400
Wire Wire Line
	1150 3400 1150 4100
Wire Wire Line
	1850 3550 2000 3550
Wire Wire Line
	3000 2900 3000 2550
Wire Wire Line
	3000 2550 1950 2550
Wire Wire Line
	1950 2550 1950 3550
Connection ~ 1950 3550
Wire Wire Line
	3000 3400 3000 4100
Wire Wire Line
	3000 3500 2900 3500
Wire Wire Line
	4350 2550 4350 2600
Wire Wire Line
	4000 1800 4550 1800
Wire Wire Line
	4350 1800 4350 2150
Wire Wire Line
	1800 2900 1500 2900
Wire Wire Line
	1350 3550 800  3550
Wire Wire Line
	3000 4100 1850 4100
Wire Wire Line
	1850 4100 1850 3700
Wire Wire Line
	1850 3700 2000 3700
Connection ~ 3000 3500
Wire Wire Line
	2000 3850 1950 3850
Wire Wire Line
	1950 3850 1950 4350
Wire Wire Line
	1950 4350 1650 4350
Wire Wire Line
	1650 4900 1650 4850
Wire Wire Line
	2900 3700 3250 3700
Wire Wire Line
	4500 1800 4500 1600
Wire Wire Line
	4500 1600 4700 1600
Connection ~ 4500 1800
Text Label 4500 1600 0    40   ~ 0
shunt5_high
Wire Wire Line
	5050 1800 5350 1800
Wire Wire Line
	5100 1800 5100 1600
Wire Wire Line
	5100 1600 5300 1600
Connection ~ 5100 1800
Text Label 5100 1600 0    40   ~ 0
shunt5_low
Connection ~ 4350 1800
Wire Wire Line
	3750 1300 3750 1500
Wire Wire Line
	3750 1400 3550 1400
Wire Wire Line
	3550 1400 3550 1000
Wire Wire Line
	3550 1000 3100 1000
Connection ~ 3750 1400
Wire Wire Line
	3100 1400 3100 1450
Wire Wire Line
	1950 1200 2300 1200
$Comp
L +15V #PWR?
U 1 1 561DD811
P 3400 1700
F 0 "#PWR?" H 3400 1550 50  0001 C CNN
F 1 "+15V" H 3400 1840 50  0000 C CNN
F 2 "" H 3400 1700 60  0000 C CNN
F 3 "" H 3400 1700 60  0000 C CNN
	1    3400 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1800 3400 1800
Wire Wire Line
	3400 1800 3400 1700
Connection ~ 2200 1200
Wire Wire Line
	3550 2400 3300 2400
Wire Wire Line
	5150 3800 5150 3750
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DD81F
P 5150 4350
F 0 "#PWR?" H 5150 4100 50  0001 C CNN
F 1 "GND" H 5150 4200 50  0000 C CNN
F 2 "" H 5150 4350 60  0000 C CNN
F 3 "" H 5150 4350 60  0000 C CNN
	1    5150 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4350 5150 4300
Wire Wire Line
	5150 3250 5150 3100
Wire Wire Line
	5150 3100 5300 3100
Wire Wire Line
	5150 3800 5350 3800
Wire Wire Line
	3400 4250 3200 4250
$Comp
L VCC #PWR?
U 1 1 561DD82D
P 4250 3050
F 0 "#PWR?" H 4250 2900 50  0001 C CNN
F 1 "VCC" H 4250 3200 50  0000 C CNN
F 2 "" H 4250 3050 60  0000 C CNN
F 3 "" H 4250 3050 60  0000 C CNN
	1    4250 3050
	1    0    0    -1  
$EndComp
Text Notes 1150 1000 2    60   ~ 0
AI      2\nDO     1\nPWM   2
Wire Notes Line
	650  550  650  5250
Wire Notes Line
	650  5250 5750 5250
Wire Notes Line
	5750 5250 5750 550 
Wire Notes Line
	5750 550  650  550 
Text Label 700  700  0    80   ~ 0
BATTERY5
Text HLabel 1950 1200 0    40   Input ~ 0
chg_ctrl5
Text HLabel 3300 2400 0    40   Input ~ 0
dchg_ctrl5
Text HLabel 3250 3700 2    40   Output ~ 0
i5
Text HLabel 5350 3800 2    40   Output ~ 0
vbat5
Text HLabel 5300 3100 2    40   UnSpc ~ 0
bat5+
Text HLabel 3200 4250 0    40   Input ~ 0
dc_led5
Text HLabel 5350 1800 2    40   UnSpc ~ 0
bat5+
Text HLabel 1050 5600 0    80   UnSpc ~ 0
+15V
$Comp
L +15V #PWR?
U 1 1 561DE12A
P 1200 5550
F 0 "#PWR?" H 1200 5400 50  0001 C CNN
F 1 "+15V" H 1200 5690 50  0000 C CNN
F 2 "" H 1200 5550 60  0000 C CNN
F 3 "" H 1200 5550 60  0000 C CNN
	1    1200 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5550 1200 5600
Wire Wire Line
	1200 5600 1050 5600
Text HLabel 1050 5750 0    80   UnSpc ~ 0
GND
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561DE23B
P 1200 5800
F 0 "#PWR?" H 1200 5550 50  0001 C CNN
F 1 "GND" H 1200 5650 50  0000 C CNN
F 2 "" H 1200 5800 60  0000 C CNN
F 3 "" H 1200 5800 60  0000 C CNN
	1    1200 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5800 1200 5750
Wire Wire Line
	1200 5750 1050 5750
Text HLabel 1050 6050 0    80   UnSpc ~ 0
-15V
$Comp
L -15V #PWR?
U 1 1 561DE3BD
P 1400 6050
F 0 "#PWR?" H 1400 5900 50  0001 C CNN
F 1 "-15V" H 1400 6190 50  0000 C CNN
F 2 "" H 1400 6050 60  0000 C CNN
F 3 "" H 1400 6050 60  0000 C CNN
	1    1400 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 6050 1050 6050
Text HLabel 1050 6450 0    80   UnSpc ~ 0
VCC
$Comp
L VCC #PWR?
U 1 1 561DE52B
P 1200 6350
F 0 "#PWR?" H 1200 6200 50  0001 C CNN
F 1 "VCC" H 1200 6500 50  0000 C CNN
F 2 "" H 1200 6350 60  0000 C CNN
F 3 "" H 1200 6350 60  0000 C CNN
	1    1200 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 6350 1200 6450
Wire Wire Line
	1200 6450 1050 6450
$Comp
L C_Small C?
U 1 1 561E5391
P 2150 4550
F 0 "C?" H 2160 4620 50  0000 L CNN
F 1 "0.1uF" H 2160 4470 50  0000 L CNN
F 2 "" H 2150 4550 60  0000 C CNN
F 3 "" H 2150 4550 60  0000 C CNN
	1    2150 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E5449
P 2150 4650
F 0 "#PWR?" H 2150 4400 50  0001 C CNN
F 1 "GND" H 2150 4500 50  0000 C CNN
F 2 "" H 2150 4650 60  0000 C CNN
F 3 "" H 2150 4650 60  0000 C CNN
	1    2150 4650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 561E54DA
P 2150 2950
F 0 "C?" H 2160 3020 50  0000 L CNN
F 1 "0.1uF" H 2160 2870 50  0000 L CNN
F 2 "" H 2150 2950 60  0000 C CNN
F 3 "" H 2150 2950 60  0000 C CNN
	1    2150 2950
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E54E0
P 2150 3050
F 0 "#PWR?" H 2150 2800 50  0001 C CNN
F 1 "GND" H 2150 2900 50  0000 C CNN
F 2 "" H 2150 3050 60  0000 C CNN
F 3 "" H 2150 3050 60  0000 C CNN
	1    2150 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2750 2450 3050
Wire Wire Line
	2150 2850 2450 2850
Connection ~ 2450 2850
Wire Wire Line
	2450 4200 2450 4400
Wire Wire Line
	2150 4450 2150 4300
Wire Wire Line
	2150 4300 2450 4300
Connection ~ 2450 4300
$Comp
L CONN_01X02 P?
U 1 1 561E596C
P 3250 5800
F 0 "P?" H 3250 5950 50  0000 C CNN
F 1 "CONN_01X02" V 3350 5800 50  0000 C CNN
F 2 "" H 3250 5800 60  0000 C CNN
F 3 "" H 3250 5800 60  0000 C CNN
	1    3250 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5750 2700 5750
Text HLabel 2700 5750 0    80   UnSpc ~ 0
bat5+
$Comp
L GND-RESCUE-BatAger #PWR?
U 1 1 561E5AC5
P 2950 5950
F 0 "#PWR?" H 2950 5700 50  0001 C CNN
F 1 "GND" H 2950 5800 50  0000 C CNN
F 2 "" H 2950 5950 60  0000 C CNN
F 3 "" H 2950 5950 60  0000 C CNN
	1    2950 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5850 2950 5850
Wire Wire Line
	2950 5850 2950 5950
$EndSCHEMATC
