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
L BC849 Q20
U 1 1 561DD709
P 4150 4250
F 0 "Q20" H 4350 4325 50  0000 L CNN
F 1 "BC849" H 4350 4250 50  0000 L CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 4350 4175 50  0001 L CIN
F 3 "" H 4150 4250 50  0000 L CNN
	1    4150 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q21
U 1 1 561DD710
P 4250 2350
F 0 "Q21" H 4450 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 4450 2350 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 4450 2275 50  0001 L CIN
F 3 "" H 4250 2350 50  0000 L CNN
	1    4250 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R81
U 1 1 561DD717
P 5150 4050
F 0 "R81" V 5230 4050 50  0000 C CNN
F 1 "R" V 5150 4050 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 5080 4050 30  0001 C CNN
F 3 "" H 5150 4050 30  0000 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R80
U 1 1 561DD71E
P 5150 3500
F 0 "R80" V 5230 3500 50  0000 C CNN
F 1 "R" V 5150 3500 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 5080 3500 30  0001 C CNN
F 3 "" H 5150 3500 30  0000 C CNN
	1    5150 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q19
U 1 1 561DD725
P 3800 1700
F 0 "Q19" H 4050 1775 50  0000 L CNN
F 1 "IRF9530" H 4050 1700 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 4050 1625 50  0001 L CIN
F 3 "" H 3800 1700 50  0000 L CNN
	1    3800 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q18
U 1 1 561DD72C
P 3000 1200
F 0 "Q18" H 3200 1275 50  0000 L CNN
F 1 "BC849" H 3200 1200 50  0000 L CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 3200 1125 50  0001 L CIN
F 3 "" H 3000 1200 50  0000 L CNN
	1    3000 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D9
U 1 1 561DD733
P 2200 2100
F 0 "D9" H 2200 2200 50  0000 C CNN
F 1 "LED" H 2200 2000 50  0000 C CNN
F 2 "LEDs:LED-0805" H 2200 2100 60  0001 C CNN
F 3 "" H 2200 2100 60  0000 C CNN
	1    2200 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D10
U 1 1 561DD73A
P 4250 3800
F 0 "D10" H 4250 3900 50  0000 C CNN
F 1 "LED" H 4250 3700 50  0000 C CNN
F 2 "LEDs:LED-0805" H 4250 3800 60  0001 C CNN
F 3 "" H 4250 3800 60  0000 C CNN
	1    4250 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R78
U 1 1 561DD741
P 4250 3300
F 0 "R78" V 4330 3300 50  0000 C CNN
F 1 "200" V 4250 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4180 3300 30  0001 C CNN
F 3 "" H 4250 3300 30  0000 C CNN
	1    4250 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R72
U 1 1 561DD748
P 2200 1600
F 0 "R72" V 2280 1600 50  0000 C CNN
F 1 "200" V 2200 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2130 1600 30  0001 C CNN
F 3 "" H 2200 1600 30  0000 C CNN
	1    2200 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R76
U 1 1 561DD74F
P 3750 1050
F 0 "R76" V 3830 1050 50  0000 C CNN
F 1 "1k" V 3750 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3680 1050 30  0001 C CNN
F 3 "" H 3750 1050 30  0000 C CNN
	1    3750 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR112
U 1 1 561DD756
P 3750 800
F 0 "#PWR112" H 3750 650 50  0001 C CNN
F 1 "+15V" H 3750 940 50  0000 C CNN
F 2 "" H 3750 800 60  0000 C CNN
F 3 "" H 3750 800 60  0000 C CNN
	1    3750 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR110
U 1 1 561DD75C
P 3100 1450
F 0 "#PWR110" H 3100 1200 50  0001 C CNN
F 1 "GND" H 3100 1300 50  0000 C CNN
F 2 "" H 3100 1450 60  0000 C CNN
F 3 "" H 3100 1450 60  0000 C CNN
	1    3100 1450
	1    0    0    -1  
$EndComp
$Comp
L R R79
U 1 1 561DD762
P 4800 1800
F 0 "R79" V 4880 1800 50  0000 C CNN
F 1 "0R056" V 4800 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 4730 1800 30  0001 C CNN
F 3 "" H 4800 1800 30  0000 C CNN
F 4 "RL2512 FK-07 0R056" V 4800 1800 60  0001 C CNN "Part #"
	1    4800 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR116
U 1 1 561DD769
P 4350 2600
F 0 "#PWR116" H 4350 2350 50  0001 C CNN
F 1 "GND" H 4350 2450 50  0000 C CNN
F 2 "" H 4350 2600 60  0000 C CNN
F 3 "" H 4350 2600 60  0000 C CNN
	1    4350 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R73
U 1 1 561DD76F
P 2550 1200
F 0 "R73" V 2630 1200 50  0000 C CNN
F 1 "1k" V 2550 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2480 1200 30  0001 C CNN
F 3 "" H 2550 1200 30  0000 C CNN
	1    2550 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R77
U 1 1 561DD776
P 3600 2400
F 0 "R77" V 3680 2400 50  0000 C CNN
F 1 "200" V 3600 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3530 2400 30  0001 C CNN
F 3 "" H 3600 2400 30  0000 C CNN
	1    3600 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR115
U 1 1 561DD77D
P 4250 4550
F 0 "#PWR115" H 4250 4300 50  0001 C CNN
F 1 "GND" H 4250 4400 50  0000 C CNN
F 2 "" H 4250 4550 60  0000 C CNN
F 3 "" H 4250 4550 60  0000 C CNN
	1    4250 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR106
U 1 1 561DD783
P 2200 2300
F 0 "#PWR106" H 2200 2050 50  0001 C CNN
F 1 "GND" H 2200 2150 50  0000 C CNN
F 2 "" H 2200 2300 60  0000 C CNN
F 3 "" H 2200 2300 60  0000 C CNN
	1    2200 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R75
U 1 1 561DD789
P 3650 4250
F 0 "R75" V 3730 4250 50  0000 C CNN
F 1 "1k" V 3650 4250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3580 4250 30  0001 C CNN
F 3 "" H 3650 4250 30  0000 C CNN
	1    3650 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U6
U 1 1 561DD790
P 2450 3500
F 0 "U6" H 2600 3750 60  0000 L CNN
F 1 "LM358DUAL" H 2450 3000 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2450 3500 60  0001 C CNN
F 3 "" H 2450 3500 60  0000 C CNN
	1    2450 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR107
U 1 1 561DD797
P 2450 2750
F 0 "#PWR107" H 2450 2600 50  0001 C CNN
F 1 "+15V" H 2450 2890 50  0000 C CNN
F 2 "" H 2450 2750 60  0000 C CNN
F 3 "" H 2450 2750 60  0000 C CNN
	1    2450 2750
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR108
U 1 1 561DD79D
P 2450 4400
F 0 "#PWR108" H 2450 4250 50  0001 C CNN
F 1 "-15V" H 2450 4540 50  0000 C CNN
F 2 "" H 2450 4400 60  0000 C CNN
F 3 "" H 2450 4400 60  0000 C CNN
	1    2450 4400
	-1   0    0    1   
$EndComp
$Comp
L R-RESCUE-BatAger R71
U 1 1 561DD7A3
P 1800 3150
F 0 "R71" V 1880 3150 50  0000 C CNN
F 1 "R" V 1800 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1730 3150 30  0001 C CNN
F 3 "" H 1800 3150 30  0000 C CNN
	1    1800 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R67
U 1 1 561DD7AA
P 1400 3400
F 0 "R67" V 1480 3400 50  0000 C CNN
F 1 "R" V 1400 3400 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1330 3400 30  0001 C CNN
F 3 "" H 1400 3400 30  0000 C CNN
	1    1400 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR97
U 1 1 561DD7B1
P 1150 4100
F 0 "#PWR97" H 1150 3850 50  0001 C CNN
F 1 "GND" H 1150 3950 50  0000 C CNN
F 2 "" H 1150 4100 60  0000 C CNN
F 3 "" H 1150 4100 60  0000 C CNN
	1    1150 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R68
U 1 1 561DD7B7
P 1600 3550
F 0 "R68" V 1680 3550 50  0000 C CNN
F 1 "R" V 1600 3550 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1530 3550 30  0001 C CNN
F 3 "" H 1600 3550 30  0000 C CNN
	1    1600 3550
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R74
U 1 1 561DD7BE
P 3000 3150
F 0 "R74" V 3080 3150 50  0000 C CNN
F 1 "R" V 3000 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 2930 3150 30  0001 C CNN
F 3 "" H 3000 3150 30  0000 C CNN
	1    3000 3150
	1    0    0    -1  
$EndComp
Text Label 1500 2900 0    40   ~ 0
shunt5_high
Text Label 800  3550 0    40   ~ 0
shunt5_low
$Comp
L R-RESCUE-BatAger R69
U 1 1 561DD7C7
P 1650 4100
F 0 "R69" V 1730 4100 50  0000 C CNN
F 1 "R" V 1650 4100 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1580 4100 30  0001 C CNN
F 3 "" H 1650 4100 30  0000 C CNN
	1    1650 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R70
U 1 1 561DD7CE
P 1650 4600
F 0 "R70" V 1730 4600 50  0000 C CNN
F 1 "R" V 1650 4600 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1580 4600 30  0001 C CNN
F 3 "" H 1650 4600 30  0000 C CNN
	1    1650 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR103
U 1 1 561DD7D5
P 1650 4900
F 0 "#PWR103" H 1650 4650 50  0001 C CNN
F 1 "GND" H 1650 4750 50  0000 C CNN
F 2 "" H 1650 4900 60  0000 C CNN
F 3 "" H 1650 4900 60  0000 C CNN
	1    1650 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR102
U 1 1 561DD7DB
P 1650 3850
F 0 "#PWR102" H 1650 3700 50  0001 C CNN
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
	4000 1800 4650 1800
Wire Wire Line
	4350 1800 4350 2150
Wire Wire Line
	1800 2900 1500 2900
Wire Wire Line
	1350 3550 800  3550
Wire Wire Line
	1850 3700 1850 4350
Wire Wire Line
	1850 3700 2000 3700
Connection ~ 3000 3500
Wire Wire Line
	2000 3850 1950 3850
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
	4950 1800 5350 1800
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
L +15V #PWR111
U 1 1 561DD811
P 3400 1700
F 0 "#PWR111" H 3400 1550 50  0001 C CNN
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
	3350 2400 3100 2400
Wire Wire Line
	5150 3800 5150 3750
$Comp
L GND-RESCUE-BatAger #PWR117
U 1 1 561DD81F
P 5150 4350
F 0 "#PWR117" H 5150 4100 50  0001 C CNN
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
L VCC #PWR114
U 1 1 561DD82D
P 4250 3050
F 0 "#PWR114" H 4250 2900 50  0001 C CNN
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
Text HLabel 3100 2400 0    40   Input ~ 0
dchg_ctrl5
Text HLabel 3250 3700 2    40   Output ~ 0
i5
Text HLabel 5350 3800 2    40   Output ~ 0
vbat5
Text HLabel 3200 4250 0    40   Input ~ 0
dc_led5
Text HLabel 1050 5600 0    80   UnSpc ~ 0
+15V
$Comp
L +15V #PWR98
U 1 1 561DE12A
P 1200 5550
F 0 "#PWR98" H 1200 5400 50  0001 C CNN
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
L GND-RESCUE-BatAger #PWR99
U 1 1 561DE23B
P 1200 5800
F 0 "#PWR99" H 1200 5550 50  0001 C CNN
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
L -15V #PWR101
U 1 1 561DE3BD
P 1400 6050
F 0 "#PWR101" H 1400 5900 50  0001 C CNN
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
L VCC #PWR100
U 1 1 561DE52B
P 1200 6350
F 0 "#PWR100" H 1200 6200 50  0001 C CNN
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
L C_Small C22
U 1 1 561E5391
P 2150 4550
F 0 "C22" H 2160 4620 50  0000 L CNN
F 1 "0.1uF" H 2160 4470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2150 4550 60  0001 C CNN
F 3 "" H 2150 4550 60  0000 C CNN
	1    2150 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR105
U 1 1 561E5449
P 2150 4650
F 0 "#PWR105" H 2150 4400 50  0001 C CNN
F 1 "GND" H 2150 4500 50  0000 C CNN
F 2 "" H 2150 4650 60  0000 C CNN
F 3 "" H 2150 4650 60  0000 C CNN
	1    2150 4650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C21
U 1 1 561E54DA
P 2150 2950
F 0 "C21" H 2160 3020 50  0000 L CNN
F 1 "0.1uF" H 2160 2870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2150 2950 60  0001 C CNN
F 3 "" H 2150 2950 60  0000 C CNN
	1    2150 2950
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR104
U 1 1 561E54E0
P 2150 3050
F 0 "#PWR104" H 2150 2800 50  0001 C CNN
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
L CONN_01X02 P12
U 1 1 561E596C
P 3250 5800
F 0 "P12" H 3250 5950 50  0000 C CNN
F 1 "CONN_01X02" V 3350 5800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3250 5800 60  0001 C CNN
F 3 "" H 3250 5800 60  0000 C CNN
	1    3250 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5750 2700 5750
$Comp
L GND-RESCUE-BatAger #PWR109
U 1 1 561E5AC5
P 2950 5950
F 0 "#PWR109" H 2950 5700 50  0001 C CNN
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
Text Label 5350 1800 2    40   ~ 0
bat5+
Text Label 5300 3100 2    40   ~ 0
bat5+
Text Label 2700 5750 0    80   ~ 0
bat5+
$Comp
L CONN_01X02 P16
U 1 1 561F6C0F
P 9350 2950
F 0 "P16" H 9350 3100 50  0000 C CNN
F 1 "FAN" V 9450 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 9350 2950 60  0001 C CNN
F 3 "" H 9350 2950 60  0000 C CNN
	1    9350 2950
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q22
U 1 1 561F6CDD
P 8850 3300
F 0 "Q22" H 9150 3350 50  0000 R CNN
F 1 "IRFR3607PBF" H 9500 3250 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 9050 3400 29  0001 C CNN
F 3 "" H 8850 3300 60  0000 C CNN
	1    8850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3000 8950 3000
Wire Wire Line
	8950 3000 8950 3100
$Comp
L GND #PWR120
U 1 1 561F6DC6
P 8950 3600
F 0 "#PWR120" H 8950 3350 50  0001 C CNN
F 1 "GND" H 8950 3450 50  0000 C CNN
F 2 "" H 8950 3600 60  0000 C CNN
F 3 "" H 8950 3600 60  0000 C CNN
	1    8950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3500 8950 3600
$Comp
L R R82
U 1 1 561F6E96
P 7550 3700
F 0 "R82" V 7630 3700 50  0000 C CNN
F 1 "200" V 7550 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7480 3700 30  0001 C CNN
F 3 "" H 7550 3700 30  0000 C CNN
	1    7550 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	8400 2900 9150 2900
Text Label 8800 2900 0    40   ~ 0
+15V
Text HLabel 7150 3700 0    40   Input ~ 0
fan_ctrl
Wire Wire Line
	1850 4350 1650 4350
Wire Wire Line
	3000 4100 1950 4100
Wire Wire Line
	1950 4100 1950 3850
$Comp
L BC849 Q24
U 1 1 562102C4
P 8400 3500
F 0 "Q24" H 8600 3575 50  0000 L CNN
F 1 "BC849" H 8600 3500 50  0000 L CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 8600 3425 50  0001 L CIN
F 3 "" H 8400 3500 50  0000 L CNN
	1    8400 3500
	1    0    0    -1  
$EndComp
$Comp
L R R84
U 1 1 56210485
P 8400 3050
F 0 "R84" V 8480 3050 50  0000 C CNN
F 1 "1k" V 8400 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8330 3050 30  0001 C CNN
F 3 "" H 8400 3050 30  0000 C CNN
	1    8400 3050
	-1   0    0    1   
$EndComp
$Comp
L BC849 Q23
U 1 1 56210668
P 7950 3700
F 0 "Q23" H 8150 3775 50  0000 L CNN
F 1 "BC849" H 8150 3700 50  0000 L CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 8150 3625 50  0001 L CIN
F 3 "" H 7950 3700 50  0000 L CNN
	1    7950 3700
	1    0    0    -1  
$EndComp
$Comp
L R R83
U 1 1 562106DC
P 8150 3300
F 0 "R83" V 8230 3300 50  0000 C CNN
F 1 "200" V 8150 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8080 3300 30  0001 C CNN
F 3 "" H 8150 3300 30  0000 C CNN
	1    8150 3300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR119
U 1 1 562109D3
P 8500 3750
F 0 "#PWR119" H 8500 3500 50  0001 C CNN
F 1 "GND" H 8500 3600 50  0000 C CNN
F 2 "" H 8500 3750 60  0000 C CNN
F 3 "" H 8500 3750 60  0000 C CNN
	1    8500 3750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR118
U 1 1 56210A3E
P 8050 3950
F 0 "#PWR118" H 8050 3700 50  0001 C CNN
F 1 "GND" H 8050 3800 50  0000 C CNN
F 2 "" H 8050 3950 60  0000 C CNN
F 3 "" H 8050 3950 60  0000 C CNN
	1    8050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 3300 8650 3300
Wire Wire Line
	8400 3300 8400 3200
Connection ~ 8500 3300
Wire Wire Line
	8200 3500 8050 3500
Wire Wire Line
	8150 3450 8150 3500
Connection ~ 8150 3500
Wire Wire Line
	7750 3700 7700 3700
Wire Wire Line
	8150 3150 8150 3000
Wire Wire Line
	8150 3000 7950 3000
Text Label 7950 3000 0    40   ~ 0
VCC
Wire Wire Line
	7150 3700 7400 3700
Wire Wire Line
	8050 3950 8050 3900
Wire Wire Line
	8500 3750 8500 3700
$Comp
L C_Small C27
U 1 1 56267435
P 3950 2550
F 0 "C27" H 3960 2620 50  0000 L CNN
F 1 "0.1uF" H 3960 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3950 2550 60  0001 C CNN
F 3 "" H 3950 2550 60  0000 C CNN
	1    3950 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2400 4050 2400
Wire Wire Line
	3950 2450 3950 2400
Connection ~ 3950 2400
$Comp
L GND-RESCUE-BatAger #PWR113
U 1 1 562675C2
P 3950 2700
F 0 "#PWR113" H 3950 2450 50  0001 C CNN
F 1 "GND" H 3950 2550 50  0000 C CNN
F 2 "" H 3950 2700 60  0000 C CNN
F 3 "" H 3950 2700 60  0000 C CNN
	1    3950 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2700 3950 2650
$EndSCHEMATC
