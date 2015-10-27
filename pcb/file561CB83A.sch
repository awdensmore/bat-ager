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
Sheet 2 4
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
L BC849 Q5
U 1 1 561CF055
P 4100 4250
F 0 "Q5" H 4300 4325 50  0000 L CNN
F 1 "BC849" H 4300 4250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4300 4175 50  0001 L CIN
F 3 "" H 4100 4250 50  0000 L CNN
	1    4100 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q4
U 1 1 561CF05C
P 4100 2350
F 0 "Q4" H 4300 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 4300 2350 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 4300 2275 50  0001 L CIN
F 3 "" H 4100 2350 50  0000 L CNN
	1    4100 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R21
U 1 1 561CF063
P 5050 4050
F 0 "R21" V 5130 4050 50  0000 C CNN
F 1 "R" V 5050 4050 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 4980 4050 30  0001 C CNN
F 3 "" H 5050 4050 30  0000 C CNN
	1    5050 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R20
U 1 1 561CF06A
P 5050 3500
F 0 "R20" V 5130 3500 50  0000 C CNN
F 1 "R" V 5050 3500 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 4980 3500 30  0001 C CNN
F 3 "" H 5050 3500 30  0000 C CNN
	1    5050 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q3
U 1 1 561CF071
P 3650 1700
F 0 "Q3" H 3900 1775 50  0000 L CNN
F 1 "IRF9530" H 3900 1700 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 3900 1625 50  0001 L CIN
F 3 "" H 3650 1700 50  0000 L CNN
	1    3650 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q2
U 1 1 561CF078
P 2850 1200
F 0 "Q2" H 3050 1275 50  0000 L CNN
F 1 "BC849" H 3050 1200 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 3050 1125 50  0001 L CIN
F 3 "" H 2850 1200 50  0000 L CNN
	1    2850 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D1
U 1 1 561CF07F
P 2050 2100
F 0 "D1" H 2050 2200 50  0000 C CNN
F 1 "LED" H 2050 2000 50  0000 C CNN
F 2 "LEDs:LED-0805" H 2050 2100 60  0001 C CNN
F 3 "" H 2050 2100 60  0000 C CNN
	1    2050 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D2
U 1 1 561CF086
P 4200 3800
F 0 "D2" H 4200 3900 50  0000 C CNN
F 1 "LED" H 4200 3700 50  0000 C CNN
F 2 "LEDs:LED-0805" H 4200 3800 60  0001 C CNN
F 3 "" H 4200 3800 60  0000 C CNN
	1    4200 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R18
U 1 1 561CF08D
P 4200 3300
F 0 "R18" V 4280 3300 50  0000 C CNN
F 1 "200" V 4200 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4130 3300 30  0001 C CNN
F 3 "" H 4200 3300 30  0000 C CNN
F 4 "RC0805JR-07 200R" V 4200 3300 60  0001 C CNN "Part #"
	1    4200 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R12
U 1 1 561CF094
P 2050 1600
F 0 "R12" V 2130 1600 50  0000 C CNN
F 1 "200" V 2050 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1980 1600 30  0001 C CNN
F 3 "" H 2050 1600 30  0000 C CNN
F 4 "RC0805JR-07 200R" V 2050 1600 60  0001 C CNN "Part #"
	1    2050 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R15
U 1 1 561CF09B
P 3600 1050
F 0 "R15" V 3680 1050 50  0000 C CNN
F 1 "1k" V 3600 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3530 1050 30  0001 C CNN
F 3 "" H 3600 1050 30  0000 C CNN
	1    3600 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR020
U 1 1 561CF0A2
P 3600 800
F 0 "#PWR020" H 3600 650 50  0001 C CNN
F 1 "+15V" H 3600 940 50  0000 C CNN
F 2 "" H 3600 800 60  0000 C CNN
F 3 "" H 3600 800 60  0000 C CNN
	1    3600 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR021
U 1 1 561CF0A8
P 2950 1450
F 0 "#PWR021" H 2950 1200 50  0001 C CNN
F 1 "GND" H 2950 1300 50  0000 C CNN
F 2 "" H 2950 1450 60  0000 C CNN
F 3 "" H 2950 1450 60  0000 C CNN
	1    2950 1450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R19
U 1 1 561CF0AE
P 4650 1800
F 0 "R19" V 4730 1800 50  0000 C CNN
F 1 "0R056" V 4650 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 4580 1800 30  0001 C CNN
F 3 "" H 4650 1800 30  0000 C CNN
	1    4650 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR022
U 1 1 561CF0B5
P 4200 2600
F 0 "#PWR022" H 4200 2350 50  0001 C CNN
F 1 "GND" H 4200 2450 50  0000 C CNN
F 2 "" H 4200 2600 60  0000 C CNN
F 3 "" H 4200 2600 60  0000 C CNN
	1    4200 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R13
U 1 1 561CF0BB
P 2400 1200
F 0 "R13" V 2480 1200 50  0000 C CNN
F 1 "1k" V 2400 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 1200 30  0001 C CNN
F 3 "" H 2400 1200 30  0000 C CNN
F 4 "RC0805JR-07 200R" V 2400 1200 60  0001 C CNN "Part #"
	1    2400 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R17
U 1 1 561CF0C2
P 3550 2400
F 0 "R17" V 3630 2400 50  0000 C CNN
F 1 "15k" V 3550 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3480 2400 30  0001 C CNN
F 3 "" H 3550 2400 30  0000 C CNN
F 4 "RC0805JR-07 200R" V 3550 2400 60  0001 C CNN "Part #"
	1    3550 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR023
U 1 1 561CF0C9
P 4200 4550
F 0 "#PWR023" H 4200 4300 50  0001 C CNN
F 1 "GND" H 4200 4400 50  0000 C CNN
F 2 "" H 4200 4550 60  0000 C CNN
F 3 "" H 4200 4550 60  0000 C CNN
	1    4200 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR024
U 1 1 561CF0CF
P 2050 2300
F 0 "#PWR024" H 2050 2050 50  0001 C CNN
F 1 "GND" H 2050 2150 50  0000 C CNN
F 2 "" H 2050 2300 60  0000 C CNN
F 3 "" H 2050 2300 60  0000 C CNN
	1    2050 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R16
U 1 1 561CF0D5
P 3600 4250
F 0 "R16" V 3680 4250 50  0000 C CNN
F 1 "1k" V 3600 4250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3530 4250 30  0001 C CNN
F 3 "" H 3600 4250 30  0000 C CNN
F 4 "RC0805JR-07 200R" V 3600 4250 60  0001 C CNN "Part #"
	1    3600 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U2
U 1 1 561CF0DC
P 2400 3500
F 0 "U2" H 2550 3750 60  0000 L CNN
F 1 "LM358DUAL" H 2400 3000 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2400 3500 60  0001 C CNN
F 3 "" H 2400 3500 60  0000 C CNN
	1    2400 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR025
U 1 1 561CF0E3
P 2400 3000
F 0 "#PWR025" H 2400 2850 50  0001 C CNN
F 1 "+15V" H 2400 3140 50  0000 C CNN
F 2 "" H 2400 3000 60  0000 C CNN
F 3 "" H 2400 3000 60  0000 C CNN
	1    2400 3000
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR32
U 1 1 561CF0E9
P 2400 4250
F 0 "#PWR32" H 2400 4100 50  0001 C CNN
F 1 "-15V" H 2400 4390 50  0000 C CNN
F 2 "" H 2400 4250 60  0000 C CNN
F 3 "" H 2400 4250 60  0000 C CNN
	1    2400 4250
	-1   0    0    1   
$EndComp
$Comp
L R R11
U 1 1 561CF0EF
P 1750 3150
F 0 "R11" V 1830 3150 50  0000 C CNN
F 1 "R" V 1750 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1680 3150 30  0001 C CNN
F 3 "" H 1750 3150 30  0000 C CNN
	1    1750 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R7
U 1 1 561CF0F6
P 1350 3400
F 0 "R7" V 1430 3400 50  0000 C CNN
F 1 "R" V 1350 3400 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1280 3400 30  0001 C CNN
F 3 "" H 1350 3400 30  0000 C CNN
	1    1350 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR026
U 1 1 561CF0FD
P 1100 4100
F 0 "#PWR026" H 1100 3850 50  0001 C CNN
F 1 "GND" H 1100 3950 50  0000 C CNN
F 2 "" H 1100 4100 60  0000 C CNN
F 3 "" H 1100 4100 60  0000 C CNN
	1    1100 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R8
U 1 1 561CF103
P 1550 3550
F 0 "R8" V 1630 3550 50  0000 C CNN
F 1 "R" V 1550 3550 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1480 3550 30  0001 C CNN
F 3 "" H 1550 3550 30  0000 C CNN
	1    1550 3550
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 561CF10A
P 2950 3150
F 0 "R14" V 3030 3150 50  0000 C CNN
F 1 "R" V 2950 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 2880 3150 30  0001 C CNN
F 3 "" H 2950 3150 30  0000 C CNN
	1    2950 3150
	1    0    0    -1  
$EndComp
Text Label 1450 2900 0    40   ~ 0
shunt1_high
$Comp
L R-RESCUE-BatAger R9
U 1 1 561CF113
P 1600 4100
F 0 "R9" V 1680 4100 50  0000 C CNN
F 1 "R" V 1600 4100 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1530 4100 30  0001 C CNN
F 3 "" H 1600 4100 30  0000 C CNN
	1    1600 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R10
U 1 1 561CF11A
P 1600 4600
F 0 "R10" V 1680 4600 50  0000 C CNN
F 1 "R" V 1600 4600 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 1530 4600 30  0001 C CNN
F 3 "" H 1600 4600 30  0000 C CNN
	1    1600 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR027
U 1 1 561CF121
P 1600 4900
F 0 "#PWR027" H 1600 4650 50  0001 C CNN
F 1 "GND" H 1600 4750 50  0000 C CNN
F 2 "" H 1600 4900 60  0000 C CNN
F 3 "" H 1600 4900 60  0000 C CNN
	1    1600 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR028
U 1 1 561CF127
P 1600 3850
F 0 "#PWR028" H 1600 3700 50  0001 C CNN
F 1 "VCC" H 1600 4000 50  0000 C CNN
F 2 "" H 1600 3850 60  0000 C CNN
F 3 "" H 1600 3850 60  0000 C CNN
	1    1600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1850 2050 1900
Wire Wire Line
	2050 1350 2050 1200
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
	2950 2800 2950 3000
Wire Wire Line
	2950 2800 1900 2800
Wire Wire Line
	1900 2800 1900 3550
Connection ~ 1900 3550
Wire Wire Line
	2950 3300 2950 4100
Wire Wire Line
	2950 3500 2850 3500
Wire Wire Line
	4200 2550 4200 2600
Wire Wire Line
	3850 1800 4400 1800
Wire Wire Line
	4200 1800 4200 2150
Wire Wire Line
	1750 2900 1450 2900
Wire Wire Line
	1300 3550 750  3550
Wire Wire Line
	2950 4100 1900 4100
Wire Wire Line
	1750 3700 1950 3700
Connection ~ 2950 3500
Wire Wire Line
	1950 3850 1900 3850
Wire Wire Line
	1600 4900 1600 4850
Wire Wire Line
	2850 3700 3200 3700
Wire Wire Line
	4350 1800 4350 1600
Wire Wire Line
	4350 1600 4550 1600
Connection ~ 4350 1800
Text Label 4350 1600 0    40   ~ 0
shunt1_high
Wire Wire Line
	4900 1800 5200 1800
Wire Wire Line
	4950 1800 4950 1600
Wire Wire Line
	4950 1600 5150 1600
Connection ~ 4950 1800
Text Label 4950 1600 0    40   ~ 0
shunt1_low
Connection ~ 4200 1800
Wire Wire Line
	3600 1300 3600 1500
Wire Wire Line
	3600 1400 3400 1400
Wire Wire Line
	3400 1400 3400 1000
Wire Wire Line
	3400 1000 2950 1000
Connection ~ 3600 1400
Wire Wire Line
	2950 1400 2950 1450
Wire Wire Line
	1800 1200 2150 1200
$Comp
L +15V #PWR029
U 1 1 561CF15D
P 3250 1700
F 0 "#PWR029" H 3250 1550 50  0001 C CNN
F 1 "+15V" H 3250 1840 50  0000 C CNN
F 2 "" H 3250 1700 60  0000 C CNN
F 3 "" H 3250 1700 60  0000 C CNN
	1    3250 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1800 3250 1800
Wire Wire Line
	3250 1800 3250 1700
Connection ~ 2050 1200
Wire Wire Line
	3300 2400 3050 2400
Wire Wire Line
	5050 3800 5050 3750
$Comp
L GND-RESCUE-BatAger #PWR030
U 1 1 561CF16B
P 5050 4350
F 0 "#PWR030" H 5050 4100 50  0001 C CNN
F 1 "GND" H 5050 4200 50  0000 C CNN
F 2 "" H 5050 4350 60  0000 C CNN
F 3 "" H 5050 4350 60  0000 C CNN
	1    5050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4350 5050 4300
Wire Wire Line
	5050 3250 5050 3100
Wire Wire Line
	5050 3100 5200 3100
Wire Wire Line
	5050 3800 5250 3800
Wire Wire Line
	3350 4250 3150 4250
$Comp
L VCC #PWR031
U 1 1 561CF179
P 4200 3050
F 0 "#PWR031" H 4200 2900 50  0001 C CNN
F 1 "VCC" H 4200 3200 50  0000 C CNN
F 2 "" H 4200 3050 60  0000 C CNN
F 3 "" H 4200 3050 60  0000 C CNN
	1    4200 3050
	1    0    0    -1  
$EndComp
Text Notes 1100 1100 2    60   ~ 0
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
BATTERY1
Text HLabel 1800 1200 0    80   Input ~ 0
chg_ctrl1
Text HLabel 3050 2400 0    80   Input ~ 0
dchg_ctrl1
Text Label 750  3550 0    40   ~ 0
shunt1_low
Text HLabel 3150 4250 0    80   Input ~ 0
dc_led1
Text HLabel 5250 3800 2    80   Output ~ 0
vbat1
Text HLabel 3200 3700 2    80   Output ~ 0
i1
Text HLabel 1100 5500 0    80   UnSpc ~ 0
+15V
Text HLabel 1100 5700 0    80   UnSpc ~ 0
GND
Text HLabel 1100 5900 0    80   UnSpc ~ 0
-15V
$Comp
L -15V #PWR22
U 1 1 561D048E
P 1200 5950
F 0 "#PWR22" H 1200 5800 50  0001 C CNN
F 1 "-15V" H 1200 6090 50  0000 C CNN
F 2 "" H 1200 5950 60  0000 C CNN
F 3 "" H 1200 5950 60  0000 C CNN
	1    1200 5950
	-1   0    0    1   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR032
U 1 1 561D04DB
P 1400 5750
F 0 "#PWR032" H 1400 5500 50  0001 C CNN
F 1 "GND" H 1400 5600 50  0000 C CNN
F 2 "" H 1400 5750 60  0000 C CNN
F 3 "" H 1400 5750 60  0000 C CNN
	1    1400 5750
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR033
U 1 1 561D05C2
P 1250 5450
F 0 "#PWR033" H 1250 5300 50  0001 C CNN
F 1 "+15V" H 1250 5590 50  0000 C CNN
F 2 "" H 1250 5450 60  0000 C CNN
F 3 "" H 1250 5450 60  0000 C CNN
	1    1250 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 5500 1250 5500
Wire Wire Line
	1250 5500 1250 5450
Wire Wire Line
	1400 5750 1400 5700
Wire Wire Line
	1400 5700 1100 5700
Wire Wire Line
	1200 5950 1200 5900
Wire Wire Line
	1200 5900 1100 5900
$Comp
L BC849 Q9
U 1 1 561D0DC9
P 9350 4250
F 0 "Q9" H 9550 4325 50  0000 L CNN
F 1 "BC849" H 9550 4250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 9550 4175 50  0001 L CIN
F 3 "" H 9350 4250 50  0000 L CNN
	1    9350 4250
	1    0    0    -1  
$EndComp
$Comp
L BTS141 Q8
U 1 1 561D0DCF
P 9350 2350
F 0 "Q8" H 9550 2425 50  0000 L CNN
F 1 "CSD18536KCS" H 9550 2350 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 9550 2275 50  0001 L CIN
F 3 "" H 9350 2350 50  0000 L CNN
	1    9350 2350
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R36
U 1 1 561D0DD5
P 10300 4050
F 0 "R36" V 10380 4050 50  0000 C CNN
F 1 "R" V 10300 4050 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 10230 4050 30  0001 C CNN
F 3 "" H 10300 4050 30  0000 C CNN
	1    10300 4050
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R35
U 1 1 561D0DDB
P 10300 3500
F 0 "R35" V 10380 3500 50  0000 C CNN
F 1 "R" V 10300 3500 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 10230 3500 30  0001 C CNN
F 3 "" H 10300 3500 30  0000 C CNN
	1    10300 3500
	1    0    0    -1  
$EndComp
$Comp
L IRF9530 Q7
U 1 1 561D0DE1
P 8900 1700
F 0 "Q7" H 9150 1775 50  0000 L CNN
F 1 "IRF9530" H 9150 1700 50  0000 L CNN
F 2 "Custom Footprints:TO-220" H 9150 1625 50  0001 L CIN
F 3 "" H 8900 1700 50  0000 L CNN
	1    8900 1700
	0    1    1    0   
$EndComp
$Comp
L BC849 Q6
U 1 1 561D0DE7
P 8100 1200
F 0 "Q6" H 8300 1275 50  0000 L CNN
F 1 "BC849" H 8300 1200 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8300 1125 50  0001 L CIN
F 3 "" H 8100 1200 50  0000 L CNN
	1    8100 1200
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BatAger D3
U 1 1 561D0DED
P 7300 2100
F 0 "D3" H 7300 2200 50  0000 C CNN
F 1 "LED" H 7300 2000 50  0000 C CNN
F 2 "LEDs:LED-0805" H 7300 2100 60  0001 C CNN
F 3 "" H 7300 2100 60  0000 C CNN
	1    7300 2100
	0    -1   -1   0   
$EndComp
$Comp
L LED-RESCUE-BatAger D4
U 1 1 561D0DF3
P 9450 3800
F 0 "D4" H 9450 3900 50  0000 C CNN
F 1 "LED" H 9450 3700 50  0000 C CNN
F 2 "LEDs:LED-0805" H 9450 3800 60  0001 C CNN
F 3 "" H 9450 3800 60  0000 C CNN
	1    9450 3800
	0    -1   -1   0   
$EndComp
$Comp
L R-RESCUE-BatAger R33
U 1 1 561D0DF9
P 9450 3300
F 0 "R33" V 9530 3300 50  0000 C CNN
F 1 "200" V 9450 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9380 3300 30  0001 C CNN
F 3 "" H 9450 3300 30  0000 C CNN
	1    9450 3300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R27
U 1 1 561D0DFF
P 7300 1600
F 0 "R27" V 7380 1600 50  0000 C CNN
F 1 "200" V 7300 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7230 1600 30  0001 C CNN
F 3 "" H 7300 1600 30  0000 C CNN
	1    7300 1600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R30
U 1 1 561D0E05
P 8850 1050
F 0 "R30" V 8930 1050 50  0000 C CNN
F 1 "1k" V 8850 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8780 1050 30  0001 C CNN
F 3 "" H 8850 1050 30  0000 C CNN
	1    8850 1050
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR034
U 1 1 561D0E0B
P 8850 800
F 0 "#PWR034" H 8850 650 50  0001 C CNN
F 1 "+15V" H 8850 940 50  0000 C CNN
F 2 "" H 8850 800 60  0000 C CNN
F 3 "" H 8850 800 60  0000 C CNN
	1    8850 800 
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR035
U 1 1 561D0E11
P 8200 1450
F 0 "#PWR035" H 8200 1200 50  0001 C CNN
F 1 "GND" H 8200 1300 50  0000 C CNN
F 2 "" H 8200 1450 60  0000 C CNN
F 3 "" H 8200 1450 60  0000 C CNN
	1    8200 1450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R34
U 1 1 561D0E17
P 9900 1800
F 0 "R34" V 9980 1800 50  0000 C CNN
F 1 "0R056" V 9900 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 9830 1800 30  0001 C CNN
F 3 "" H 9900 1800 30  0000 C CNN
	1    9900 1800
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR036
U 1 1 561D0E1D
P 9450 2600
F 0 "#PWR036" H 9450 2350 50  0001 C CNN
F 1 "GND" H 9450 2450 50  0000 C CNN
F 2 "" H 9450 2600 60  0000 C CNN
F 3 "" H 9450 2600 60  0000 C CNN
	1    9450 2600
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R28
U 1 1 561D0E23
P 7650 1200
F 0 "R28" V 7730 1200 50  0000 C CNN
F 1 "1k" V 7650 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7580 1200 30  0001 C CNN
F 3 "" H 7650 1200 30  0000 C CNN
	1    7650 1200
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R32
U 1 1 561D0E29
P 8700 2400
F 0 "R32" V 8780 2400 50  0000 C CNN
F 1 "15k" V 8700 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8630 2400 30  0001 C CNN
F 3 "" H 8700 2400 30  0000 C CNN
	1    8700 2400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR037
U 1 1 561D0E2F
P 9450 4550
F 0 "#PWR037" H 9450 4300 50  0001 C CNN
F 1 "GND" H 9450 4400 50  0000 C CNN
F 2 "" H 9450 4550 60  0000 C CNN
F 3 "" H 9450 4550 60  0000 C CNN
	1    9450 4550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR038
U 1 1 561D0E35
P 7300 2300
F 0 "#PWR038" H 7300 2050 50  0001 C CNN
F 1 "GND" H 7300 2150 50  0000 C CNN
F 2 "" H 7300 2300 60  0000 C CNN
F 3 "" H 7300 2300 60  0000 C CNN
	1    7300 2300
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R31
U 1 1 561D0E3B
P 8850 4250
F 0 "R31" V 8930 4250 50  0000 C CNN
F 1 "1k" V 8850 4250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8780 4250 30  0001 C CNN
F 3 "" H 8850 4250 30  0000 C CNN
	1    8850 4250
	0    -1   -1   0   
$EndComp
$Comp
L LM358Dual U3
U 1 1 561D0E41
P 7650 3500
F 0 "U3" H 7800 3750 60  0000 L CNN
F 1 "LM358DUAL" H 7650 3000 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7650 3500 60  0001 C CNN
F 3 "" H 7650 3500 60  0000 C CNN
	1    7650 3500
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR039
U 1 1 561D0E47
P 7650 2950
F 0 "#PWR039" H 7650 2800 50  0001 C CNN
F 1 "+15V" H 7650 3090 50  0000 C CNN
F 2 "" H 7650 2950 60  0000 C CNN
F 3 "" H 7650 2950 60  0000 C CNN
	1    7650 2950
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR50
U 1 1 561D0E4D
P 7650 4350
F 0 "#PWR50" H 7650 4200 50  0001 C CNN
F 1 "-15V" H 7650 4490 50  0000 C CNN
F 2 "" H 7650 4350 60  0000 C CNN
F 3 "" H 7650 4350 60  0000 C CNN
	1    7650 4350
	-1   0    0    1   
$EndComp
$Comp
L R R26
U 1 1 561D0E53
P 7000 3150
F 0 "R26" V 7080 3150 50  0000 C CNN
F 1 "R" V 7000 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 6930 3150 30  0001 C CNN
F 3 "" H 7000 3150 30  0000 C CNN
	1    7000 3150
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R22
U 1 1 561D0E59
P 6600 3400
F 0 "R22" V 6680 3400 50  0000 C CNN
F 1 "R" V 6600 3400 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 6530 3400 30  0001 C CNN
F 3 "" H 6600 3400 30  0000 C CNN
	1    6600 3400
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR040
U 1 1 561D0E5F
P 6350 4100
F 0 "#PWR040" H 6350 3850 50  0001 C CNN
F 1 "GND" H 6350 3950 50  0000 C CNN
F 2 "" H 6350 4100 60  0000 C CNN
F 3 "" H 6350 4100 60  0000 C CNN
	1    6350 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R23
U 1 1 561D0E65
P 6800 3550
F 0 "R23" V 6880 3550 50  0000 C CNN
F 1 "R" V 6800 3550 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 6730 3550 30  0001 C CNN
F 3 "" H 6800 3550 30  0000 C CNN
	1    6800 3550
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-BatAger R29
U 1 1 561D0E6B
P 8200 3150
F 0 "R29" V 8280 3150 50  0000 C CNN
F 1 "R" V 8200 3150 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 8130 3150 30  0001 C CNN
F 3 "" H 8200 3150 30  0000 C CNN
	1    8200 3150
	1    0    0    -1  
$EndComp
Text Label 6700 2900 0    40   ~ 0
shunt2_high
$Comp
L R-RESCUE-BatAger R24
U 1 1 561D0E72
P 6850 4100
F 0 "R24" V 6930 4100 50  0000 C CNN
F 1 "R" V 6850 4100 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 6780 4100 30  0001 C CNN
F 3 "" H 6850 4100 30  0000 C CNN
	1    6850 4100
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-BatAger R25
U 1 1 561D0E78
P 6850 4600
F 0 "R25" V 6930 4600 50  0000 C CNN
F 1 "R" V 6850 4600 50  0000 C CNN
F 2 "Custom Footprints:Resistor_Horizontal_RM10mm" V 6780 4600 30  0001 C CNN
F 3 "" H 6850 4600 30  0000 C CNN
	1    6850 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR041
U 1 1 561D0E7E
P 6850 4900
F 0 "#PWR041" H 6850 4650 50  0001 C CNN
F 1 "GND" H 6850 4750 50  0000 C CNN
F 2 "" H 6850 4900 60  0000 C CNN
F 3 "" H 6850 4900 60  0000 C CNN
	1    6850 4900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR042
U 1 1 561D0E84
P 6850 3850
F 0 "#PWR042" H 6850 3700 50  0001 C CNN
F 1 "VCC" H 6850 4000 50  0000 C CNN
F 2 "" H 6850 3850 60  0000 C CNN
F 3 "" H 6850 3850 60  0000 C CNN
	1    6850 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 1850 7300 1900
Wire Wire Line
	7300 1350 7300 1200
Wire Wire Line
	9450 4050 9450 4000
Wire Wire Line
	9450 3600 9450 3550
Wire Wire Line
	9450 4450 9450 4550
Wire Wire Line
	9100 4250 9150 4250
Wire Wire Line
	6850 3400 7200 3400
Connection ~ 7000 3400
Wire Wire Line
	6350 3400 6350 4100
Wire Wire Line
	7050 3550 7200 3550
Wire Wire Line
	8200 2900 8200 2650
Wire Wire Line
	8200 2650 7150 2650
Wire Wire Line
	7150 2650 7150 3550
Connection ~ 7150 3550
Wire Wire Line
	8200 3400 8200 4100
Wire Wire Line
	8200 3500 8100 3500
Wire Wire Line
	9450 2550 9450 2600
Wire Wire Line
	9100 1800 9650 1800
Wire Wire Line
	9450 1800 9450 2150
Wire Wire Line
	7000 2900 6700 2900
Wire Wire Line
	6550 3550 6000 3550
Wire Wire Line
	7050 3700 7050 4350
Wire Wire Line
	7050 3700 7200 3700
Connection ~ 8200 3500
Wire Wire Line
	7200 3850 7150 3850
Wire Wire Line
	6850 4900 6850 4850
Wire Wire Line
	8100 3700 8450 3700
Wire Wire Line
	9600 1800 9600 1600
Wire Wire Line
	9600 1600 9800 1600
Connection ~ 9600 1800
Text Label 9600 1600 0    40   ~ 0
shunt2_high
Wire Wire Line
	10150 1800 10450 1800
Wire Wire Line
	10200 1800 10200 1600
Wire Wire Line
	10200 1600 10400 1600
Connection ~ 10200 1800
Text Label 10200 1600 0    40   ~ 0
shunt2_low
Connection ~ 9450 1800
Wire Wire Line
	8850 1300 8850 1500
Wire Wire Line
	8850 1400 8650 1400
Wire Wire Line
	8650 1400 8650 1000
Wire Wire Line
	8650 1000 8200 1000
Connection ~ 8850 1400
Wire Wire Line
	8200 1400 8200 1450
Wire Wire Line
	7050 1200 7400 1200
$Comp
L +15V #PWR043
U 1 1 561D0EB9
P 8500 1700
F 0 "#PWR043" H 8500 1550 50  0001 C CNN
F 1 "+15V" H 8500 1840 50  0000 C CNN
F 2 "" H 8500 1700 60  0000 C CNN
F 3 "" H 8500 1700 60  0000 C CNN
	1    8500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1800 8500 1800
Wire Wire Line
	8500 1800 8500 1700
Connection ~ 7300 1200
Wire Wire Line
	8450 2400 8200 2400
Wire Wire Line
	10300 3800 10300 3750
$Comp
L GND-RESCUE-BatAger #PWR044
U 1 1 561D0EC4
P 10300 4350
F 0 "#PWR044" H 10300 4100 50  0001 C CNN
F 1 "GND" H 10300 4200 50  0000 C CNN
F 2 "" H 10300 4350 60  0000 C CNN
F 3 "" H 10300 4350 60  0000 C CNN
	1    10300 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 4350 10300 4300
Wire Wire Line
	10300 3250 10300 3100
Wire Wire Line
	10300 3100 10450 3100
Wire Wire Line
	10300 3800 10500 3800
Wire Wire Line
	8600 4250 8400 4250
$Comp
L VCC #PWR045
U 1 1 561D0ECF
P 9450 3050
F 0 "#PWR045" H 9450 2900 50  0001 C CNN
F 1 "VCC" H 9450 3200 50  0000 C CNN
F 2 "" H 9450 3050 60  0000 C CNN
F 3 "" H 9450 3050 60  0000 C CNN
	1    9450 3050
	1    0    0    -1  
$EndComp
Text Notes 6350 1050 2    60   ~ 0
AI      2\nDO     1\nPWM   2
Wire Notes Line
	5850 550  5850 5250
Wire Notes Line
	5850 5250 10950 5250
Wire Notes Line
	10950 5250 10950 550 
Wire Notes Line
	10950 550  5850 550 
Text Label 5900 700  0    80   ~ 0
BATTERY2
Text HLabel 7050 1200 0    80   Input ~ 0
chg_ctrl2
Text HLabel 8200 2400 0    80   Input ~ 0
dchg_ctrl2
Text Label 6000 3550 0    40   ~ 0
shunt2_low
Text HLabel 8400 4250 0    80   Input ~ 0
dc_led2
Text HLabel 10500 3800 2    80   Output ~ 0
vbat2
Text HLabel 8450 3700 2    80   Output ~ 0
i2
Text HLabel 1100 6300 0    80   UnSpc ~ 0
VCC
$Comp
L VCC #PWR046
U 1 1 561D92A2
P 1400 6200
F 0 "#PWR046" H 1400 6050 50  0001 C CNN
F 1 "VCC" H 1400 6350 50  0000 C CNN
F 2 "" H 1400 6200 60  0000 C CNN
F 3 "" H 1400 6200 60  0000 C CNN
	1    1400 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 6200 1400 6300
Wire Wire Line
	1400 6300 1100 6300
$Comp
L CONN_01X02 P8
U 1 1 561DF67D
P 3200 5950
F 0 "P8" H 3200 6100 50  0000 C CNN
F 1 "CONN_01X02" V 3300 5950 50  0000 C CNN
F 2 "Custom Footprints:PINHEAD1-2_200mil" H 3200 5950 60  0001 C CNN
F 3 "" H 3200 5950 60  0000 C CNN
	1    3200 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5900 2650 5900
$Comp
L GND-RESCUE-BatAger #PWR047
U 1 1 561DF8C7
P 2850 6100
F 0 "#PWR047" H 2850 5850 50  0001 C CNN
F 1 "GND" H 2850 5950 50  0000 C CNN
F 2 "" H 2850 6100 60  0000 C CNN
F 3 "" H 2850 6100 60  0000 C CNN
	1    2850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6000 2850 6000
Wire Wire Line
	2850 6000 2850 6100
$Comp
L CONN_01X02 P9
U 1 1 561E186A
P 3200 6750
F 0 "P9" H 3200 6900 50  0000 C CNN
F 1 "CONN_01X02" V 3300 6750 50  0000 C CNN
F 2 "Custom Footprints:PINHEAD1-2_200mil" H 3200 6750 60  0001 C CNN
F 3 "" H 3200 6750 60  0000 C CNN
	1    3200 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6700 2650 6700
$Comp
L GND-RESCUE-BatAger #PWR048
U 1 1 561E1871
P 2850 6900
F 0 "#PWR048" H 2850 6650 50  0001 C CNN
F 1 "GND" H 2850 6750 50  0000 C CNN
F 2 "" H 2850 6900 60  0000 C CNN
F 3 "" H 2850 6900 60  0000 C CNN
	1    2850 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6800 2850 6800
Wire Wire Line
	2850 6800 2850 6900
$Comp
L C_Small C14
U 1 1 561E1F35
P 2150 4400
F 0 "C14" H 2160 4470 50  0000 L CNN
F 1 "0.1uF" H 2160 4320 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2150 4400 60  0001 C CNN
F 3 "" H 2150 4400 60  0000 C CNN
	1    2150 4400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C13
U 1 1 561E2121
P 2050 3050
F 0 "C13" H 2060 3120 50  0000 L CNN
F 1 "0.1uF" H 2060 2970 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2050 3050 60  0001 C CNN
F 3 "" H 2050 3050 60  0000 C CNN
	1    2050 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2950 2250 2950
Wire Wire Line
	2400 3000 2400 3050
Wire Wire Line
	2250 2950 2250 3050
Wire Wire Line
	2250 3050 2400 3050
Connection ~ 2400 3050
Wire Wire Line
	2150 4300 2150 4200
Wire Wire Line
	2150 4200 2400 4200
Wire Wire Line
	2400 4200 2400 4250
Connection ~ 2400 4200
$Comp
L GND-RESCUE-BatAger #PWR049
U 1 1 561E28B4
P 2050 3150
F 0 "#PWR049" H 2050 2900 50  0001 C CNN
F 1 "GND" H 2050 3000 50  0000 C CNN
F 2 "" H 2050 3150 60  0000 C CNN
F 3 "" H 2050 3150 60  0000 C CNN
	1    2050 3150
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR050
U 1 1 561E294F
P 2150 4500
F 0 "#PWR050" H 2150 4250 50  0001 C CNN
F 1 "GND" H 2150 4350 50  0000 C CNN
F 2 "" H 2150 4500 60  0000 C CNN
F 3 "" H 2150 4500 60  0000 C CNN
	1    2150 4500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C16
U 1 1 561E30A6
P 7350 4500
F 0 "C16" H 7360 4570 50  0000 L CNN
F 1 "0.1uF" H 7360 4420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7350 4500 60  0001 C CNN
F 3 "" H 7350 4500 60  0000 C CNN
	1    7350 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 4400 7350 4300
$Comp
L GND-RESCUE-BatAger #PWR051
U 1 1 561E30AD
P 7350 4600
F 0 "#PWR051" H 7350 4350 50  0001 C CNN
F 1 "GND" H 7350 4450 50  0000 C CNN
F 2 "" H 7350 4600 60  0000 C CNN
F 3 "" H 7350 4600 60  0000 C CNN
	1    7350 4600
	1    0    0    -1  
$EndComp
$Comp
L C_Small C15
U 1 1 561E3496
P 7350 3000
F 0 "C15" H 7360 3070 50  0000 L CNN
F 1 "0.1uF" H 7360 2920 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7350 3000 60  0001 C CNN
F 3 "" H 7350 3000 60  0000 C CNN
	1    7350 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2900 7350 2800
$Comp
L GND-RESCUE-BatAger #PWR052
U 1 1 561E349D
P 7350 3100
F 0 "#PWR052" H 7350 2850 50  0001 C CNN
F 1 "GND" H 7350 2950 50  0000 C CNN
F 2 "" H 7350 3100 60  0000 C CNN
F 3 "" H 7350 3100 60  0000 C CNN
	1    7350 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 2950 7500 2950
Wire Wire Line
	7500 2950 7500 2800
Wire Wire Line
	7500 2800 7350 2800
Wire Wire Line
	7650 2950 7650 3050
Wire Wire Line
	7650 4200 7650 4350
Wire Wire Line
	7350 4300 7650 4300
Connection ~ 7650 4300
Text Label 5200 3100 2    80   ~ 0
bat1+
Text Label 2650 5900 0    80   ~ 0
bat1+
Text Label 2650 6700 0    80   ~ 0
bat2+
Text Label 5200 1800 0    80   ~ 0
bat1+
Text Label 10450 1800 0    80   ~ 0
bat2+
Text Label 10450 3100 0    80   ~ 0
bat2+
Wire Wire Line
	1750 3000 1750 2900
Wire Wire Line
	1750 3300 1750 3400
Wire Wire Line
	7000 3000 7000 2900
Wire Wire Line
	7000 3300 7000 3400
Wire Wire Line
	1750 3700 1750 4350
Wire Wire Line
	1750 4350 1600 4350
Wire Wire Line
	1900 3850 1900 4100
Wire Wire Line
	7050 4350 6850 4350
Wire Wire Line
	8200 4100 7150 4100
Wire Wire Line
	7150 4100 7150 3850
$Comp
L C_Small C23
U 1 1 56265748
P 3800 2550
F 0 "C23" H 3810 2620 50  0000 L CNN
F 1 "1uF" H 3810 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3800 2550 60  0001 C CNN
F 3 "" H 3800 2550 60  0000 C CNN
	1    3800 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2450 3800 2400
Wire Wire Line
	3800 2400 3900 2400
$Comp
L GND-RESCUE-BatAger #PWR053
U 1 1 562659DA
P 3800 2700
F 0 "#PWR053" H 3800 2450 50  0001 C CNN
F 1 "GND" H 3800 2550 50  0000 C CNN
F 2 "" H 3800 2700 60  0000 C CNN
F 3 "" H 3800 2700 60  0000 C CNN
	1    3800 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2700 3800 2650
$Comp
L C_Small C24
U 1 1 56265F55
P 9000 2550
F 0 "C24" H 9010 2620 50  0000 L CNN
F 1 "1uF" H 9010 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9000 2550 60  0001 C CNN
F 3 "" H 9000 2550 60  0000 C CNN
	1    9000 2550
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-BatAger #PWR054
U 1 1 562660BC
P 9000 2700
F 0 "#PWR054" H 9000 2450 50  0001 C CNN
F 1 "GND" H 9000 2550 50  0000 C CNN
F 2 "" H 9000 2700 60  0000 C CNN
F 3 "" H 9000 2700 60  0000 C CNN
	1    9000 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2400 9150 2400
Wire Wire Line
	9000 2450 9000 2400
Connection ~ 9000 2400
Wire Wire Line
	9000 2650 9000 2700
$EndSCHEMATC
