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
Sheet 1 4
Title ""
Date "12 oct 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 9050 1050 1550 1450
U 561CB83B
F0 "Sheet561CB83A" 80
F1 "file561CB83A.sch" 80
F2 "chg_ctrl1" I R 10600 1150 80 
F3 "dchg_ctrl1" I R 10600 1300 80 
F4 "dc_led1" I R 10600 1450 80 
F5 "vbat1" O R 10600 1600 80 
F6 "i1" O R 10600 1900 80 
F7 "+15V" U R 10600 2050 80 
F8 "GND" U R 10600 2200 80 
F9 "-15V" U R 10600 2350 80 
F10 "chg_ctrl2" I L 9050 1150 80 
F11 "dchg_ctrl2" I L 9050 1300 80 
F12 "dc_led2" I L 9050 1450 80 
F13 "vbat2" O L 9050 1600 80 
F14 "i2" O L 9050 1900 80 
F15 "VCC" U L 9050 2050 80 
$EndSheet
$Sheet
S 9200 2950 1400 1400
U 561D4A91
F0 "Sheet561D4A90" 80
F1 "file561D4A90.sch" 80
F2 "chg_ctrl3" I R 10600 3050 80 
F3 "dchg_ctrl3" I R 10600 3200 80 
F4 "i3" O R 10600 3500 80 
F5 "dc_led3" I R 10600 3650 80 
F6 "vbat3" O R 10600 3800 80 
F7 "chg_ctrl4" I R 10600 3950 80 
F8 "dchg_ctrl4" I R 10600 4100 80 
F9 "i4" O L 9200 3050 80 
F10 "dc_led4" I L 9200 3200 80 
F11 "vbat4" O L 9200 3350 80 
F12 "+15V" U L 9200 3500 80 
F13 "GND" U L 9200 3650 80 
F14 "-15V" U L 9200 3800 80 
F15 "VCC" U L 9200 3950 80 
$EndSheet
$Comp
L ATMEGA2560-A IC1
U 1 1 561D57F5
P 4350 3800
F 0 "IC1" H 3200 6600 40  0000 L BNN
F 1 "ATMEGA2560-A" H 5050 950 40  0000 L BNN
F 2 "Housings_QFP:TQFP-100_14x14mm_Pitch0.5mm" H 4350 3800 30  0000 C CIN
F 3 "" H 4350 3800 60  0000 C CNN
	1    4350 3800
	1    0    0    -1  
$EndComp
$Sheet
S 9200 4600 1400 1450
U 561DD111
F0 "Sheet561DD110" 80
F1 "file561DD110.sch" 80
F2 "chg_ctrl5" I R 10600 4700 80 
F3 "dchg_ctrl5" I R 10600 4850 80 
F4 "i5" O R 10600 5000 80 
F5 "vbat5" O R 10600 5150 80 
F6 "dc_led5" I R 10600 5450 80 
F7 "+15V" U R 10600 5600 80 
F8 "GND" U R 10600 5750 80 
F9 "-15V" U R 10600 5900 80 
F10 "VCC" U L 9200 4700 80 
$EndSheet
$Comp
L +15V #PWR01
U 1 1 561DFF64
P 7600 4550
F 0 "#PWR01" H 7600 4400 50  0001 C CNN
F 1 "+15V" H 7600 4690 50  0000 C CNN
F 2 "" H 7600 4550 60  0000 C CNN
F 3 "" H 7600 4550 60  0000 C CNN
	1    7600 4550
	1    0    0    -1  
$EndComp
$Comp
L -15V #PWR16
U 1 1 561E0397
P 7000 5500
F 0 "#PWR16" H 7000 5350 50  0001 C CNN
F 1 "-15V" H 7000 5640 50  0000 C CNN
F 2 "" H 7000 5500 60  0000 C CNN
F 3 "" H 7000 5500 60  0000 C CNN
	1    7000 5500
	-1   0    0    1   
$EndComp
$Comp
L CP C11
U 1 1 561E6331
P 7200 5350
F 0 "C11" H 7225 5450 50  0000 L CNN
F 1 "150uF" H 7225 5250 50  0000 L CNN
F 2 "Capacitors_Elko_ThroughHole:Elko_vert_11.5x8mm_RM3.5" H 7238 5200 30  0001 C CNN
F 3 "" H 7200 5350 60  0000 C CNN
	1    7200 5350
	1    0    0    -1  
$EndComp
$Comp
L CP C12
U 1 1 561E67B1
P 7850 4800
F 0 "C12" H 7875 4900 50  0000 L CNN
F 1 "150uF" H 7875 4700 50  0000 L CNN
F 2 "Capacitors_Elko_ThroughHole:Elko_vert_11.5x8mm_RM3.5" H 7888 4650 30  0001 C CNN
F 3 "" H 7850 4800 60  0000 C CNN
	1    7850 4800
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 561CB9BC
P 2750 1650
F 0 "Y1" H 2750 1800 50  0000 C CNN
F 1 "16MHz" H 2750 1500 50  0000 C CNN
F 2 "Crystals:Crystal_HC48-U_Vertical" H 2750 1650 60  0001 C CNN
F 3 "" H 2750 1650 60  0000 C CNN
	1    2750 1650
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 561CBB8A
P 2400 1650
F 0 "R3" V 2480 1650 50  0000 C CNN
F 1 "1M" V 2400 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 1650 30  0001 C CNN
F 3 "" H 2400 1650 30  0000 C CNN
	1    2400 1650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C5
U 1 1 561CBD37
P 2100 1500
F 0 "C5" H 2110 1570 50  0000 L CNN
F 1 "22pF" H 2110 1420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2100 1500 60  0001 C CNN
F 3 "" H 2100 1500 60  0000 C CNN
	1    2100 1500
	0    1    1    0   
$EndComp
$Comp
L C_Small C6
U 1 1 561CBE75
P 2100 1800
F 0 "C6" H 2110 1870 50  0000 L CNN
F 1 "22pF" H 2110 1720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2100 1800 60  0001 C CNN
F 3 "" H 2100 1800 60  0000 C CNN
	1    2100 1800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 561CC021
P 1850 1650
F 0 "#PWR02" H 1850 1400 50  0001 C CNN
F 1 "GND" H 1850 1500 50  0000 C CNN
F 2 "" H 1850 1650 60  0000 C CNN
F 3 "" H 1850 1650 60  0000 C CNN
	1    1850 1650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 561CDB6D
P 4350 6800
F 0 "#PWR03" H 4350 6550 50  0001 C CNN
F 1 "GND" H 4350 6650 50  0000 C CNN
F 2 "" H 4350 6800 60  0000 C CNN
F 3 "" H 4350 6800 60  0000 C CNN
	1    4350 6800
	1    0    0    -1  
$EndComp
$Comp
L C_Small C8
U 1 1 561CE064
P 4800 650
F 0 "C8" H 4810 720 50  0000 L CNN
F 1 "100nF" H 4810 570 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4800 650 60  0001 C CNN
F 3 "" H 4800 650 60  0000 C CNN
	1    4800 650 
	-1   0    0    1   
$EndComp
$Comp
L C_Small C9
U 1 1 561CE514
P 5100 650
F 0 "C9" H 5110 720 50  0000 L CNN
F 1 "100nF" H 5110 570 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5100 650 60  0001 C CNN
F 3 "" H 5100 650 60  0000 C CNN
	1    5100 650 
	-1   0    0    1   
$EndComp
$Comp
L C_Small C10
U 1 1 561CE646
P 5350 650
F 0 "C10" H 5360 720 50  0000 L CNN
F 1 "100nF" H 5360 570 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5350 650 60  0001 C CNN
F 3 "" H 5350 650 60  0000 C CNN
	1    5350 650 
	-1   0    0    1   
$EndComp
Wire Wire Line
	7600 5200 7800 5200
Wire Wire Line
	7600 4550 7600 5200
Wire Wire Line
	7000 5500 7800 5500
Wire Wire Line
	7800 5900 7650 5900
Wire Wire Line
	7650 5900 7650 6000
Connection ~ 7200 5500
Wire Wire Line
	7200 5200 7200 5050
Wire Wire Line
	7200 5050 6950 5050
Wire Wire Line
	6950 5050 6950 5100
Wire Wire Line
	7850 4650 7600 4650
Connection ~ 7600 4650
Wire Wire Line
	2200 1500 3050 1500
Connection ~ 2750 1500
Connection ~ 2400 1500
Wire Wire Line
	2200 1800 3050 1800
Connection ~ 2400 1800
Connection ~ 2750 1800
Wire Wire Line
	2000 1500 2000 1800
Wire Wire Line
	1850 1650 2000 1650
Connection ~ 2000 1650
Wire Wire Line
	4200 6700 4500 6700
Connection ~ 4300 6700
Connection ~ 4400 6700
Wire Wire Line
	4350 6800 4350 6700
Connection ~ 4350 6700
$Comp
L GND #PWR04
U 1 1 561CF0F9
P 5100 800
F 0 "#PWR04" H 5100 550 50  0001 C CNN
F 1 "GND" H 5100 650 50  0000 C CNN
F 2 "" H 5100 800 60  0000 C CNN
F 3 "" H 5100 800 60  0000 C CNN
	1    5100 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 800  5100 750 
Wire Wire Line
	4800 750  5350 750 
Connection ~ 5100 750 
Wire Wire Line
	4200 550  5350 550 
Connection ~ 5100 550 
Wire Wire Line
	4200 900  4200 550 
Connection ~ 4800 550 
Wire Wire Line
	4000 900  4500 900 
Connection ~ 4200 900 
Connection ~ 4100 900 
$Comp
L SW_PUSH_4PIN-RESCUE-BatAger SW1
U 1 1 561CB857
P -950 1600
F 0 "SW1" H -800 1710 50  0000 C CNN
F 1 "SW_PUSH_4PIN" H -950 1520 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH_SMALL" H -950 1600 60  0001 C CNN
F 3 "" H -950 1600 60  0000 C CNN
	1    -950 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 561CBC69
P -1350 1750
F 0 "#PWR05" H -1350 1500 50  0001 C CNN
F 1 "GND" H -1350 1600 50  0000 C CNN
F 2 "" H -1350 1750 60  0000 C CNN
F 3 "" H -1350 1750 60  0000 C CNN
	1    -1350 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1350 1600 -1250 1600
Wire Wire Line
	-1350 1600 -1350 1750
Wire Wire Line
	-1250 1650 -1300 1650
Wire Wire Line
	-1300 1650 -1300 1600
Connection ~ -1300 1600
Wire Wire Line
	-650 1650 -600 1650
Wire Wire Line
	-600 1650 -600 1600
Wire Wire Line
	-650 1600 -450 1600
Connection ~ -600 1600
Text Label -450 1600 0    40   ~ 0
RESET
Wire Wire Line
	3050 1200 2750 1200
Text Label 2750 1200 0    40   ~ 0
RESET
$Comp
L C_Small C7
U 1 1 561CDDDE
P 2800 6500
F 0 "C7" H 2810 6570 50  0000 L CNN
F 1 "100nF" H 2810 6420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2800 6500 60  0001 C CNN
F 3 "" H 2800 6500 60  0000 C CNN
	1    2800 6500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 561CDF2D
P 2800 6600
F 0 "#PWR06" H 2800 6350 50  0001 C CNN
F 1 "GND" H 2800 6450 50  0000 C CNN
F 2 "" H 2800 6600 60  0000 C CNN
F 3 "" H 2800 6600 60  0000 C CNN
	1    2800 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6400 2800 6400
$Comp
L MC33269 U1
U 1 1 561CEF30
P -1050 4300
F 0 "U1" H -1250 4500 40  0000 C CNN
F 1 "MC33269" H -1050 4500 40  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H -1050 4400 30  0001 C CIN
F 3 "" H -1050 4300 60  0000 C CNN
	1    -1050 4300
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR07
U 1 1 561D012C
P -300 4150
F 0 "#PWR07" H -300 4000 50  0001 C CNN
F 1 "+15V" H -300 4290 50  0000 C CNN
F 2 "" H -300 4150 60  0000 C CNN
F 3 "" H -300 4150 60  0000 C CNN
	1    -300 4150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 561D0A41
P -300 4350
F 0 "C4" H -290 4420 50  0000 L CNN
F 1 "100nF" H -290 4270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -300 4350 60  0001 C CNN
F 3 "" H -300 4350 60  0000 C CNN
	1    -300 4350
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C2
U 1 1 561D0DF4
P -550 4350
F 0 "C2" H -540 4420 50  0000 L CNN
F 1 "47uF" H -540 4270 50  0000 L CNN
F 2 "Capacitors_Elko_ThroughHole:Elko_vert_11.2x6.3mm_RM2.5" H -550 4350 60  0001 C CNN
F 3 "" H -550 4350 60  0000 C CNN
	1    -550 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	-650 4250 -300 4250
Connection ~ -550 4250
Wire Wire Line
	-300 4250 -300 4150
$Comp
L GND #PWR08
U 1 1 561D1799
P -400 4500
F 0 "#PWR08" H -400 4250 50  0001 C CNN
F 1 "GND" H -400 4350 50  0000 C CNN
F 2 "" H -400 4500 60  0000 C CNN
F 3 "" H -400 4500 60  0000 C CNN
	1    -400 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	-550 4450 -300 4450
Wire Wire Line
	-400 4500 -400 4450
Connection ~ -400 4450
$Comp
L GND #PWR09
U 1 1 561D1A5E
P -1600 4300
F 0 "#PWR09" H -1600 4050 50  0001 C CNN
F 1 "GND" H -1600 4150 50  0000 C CNN
F 2 "" H -1600 4300 60  0000 C CNN
F 3 "" H -1600 4300 60  0000 C CNN
	1    -1600 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1450 4250 -1600 4250
Wire Wire Line
	-1600 4250 -1600 4300
$Comp
L VCC #PWR010
U 1 1 561D1CE2
P -1400 4650
F 0 "#PWR010" H -1400 4500 50  0001 C CNN
F 1 "VCC" H -1400 4800 50  0000 C CNN
F 2 "" H -1400 4650 60  0000 C CNN
F 3 "" H -1400 4650 60  0000 C CNN
	1    -1400 4650
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C1
U 1 1 561D1F46
P -1200 4750
F 0 "C1" H -1190 4820 50  0000 L CNN
F 1 "47uF" H -1190 4670 50  0000 L CNN
F 2 "Capacitors_Elko_ThroughHole:Elko_vert_11.2x6.3mm_RM2.5" H -1200 4750 60  0001 C CNN
F 3 "" H -1200 4750 60  0000 C CNN
	1    -1200 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1050 4650 -1050 4550
Wire Wire Line
	-1400 4650 -1050 4650
Connection ~ -1200 4650
$Comp
L GND #PWR011
U 1 1 561D236D
P -1200 4900
F 0 "#PWR011" H -1200 4650 50  0001 C CNN
F 1 "GND" H -1200 4750 50  0000 C CNN
F 2 "" H -1200 4900 60  0000 C CNN
F 3 "" H -1200 4900 60  0000 C CNN
	1    -1200 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1200 4900 -1200 4850
$Comp
L Q_NMOS_GDS Q1
U 1 1 561D2DE7
P -1250 3100
F 0 "Q1" H -950 3150 50  0000 R CNN
F 1 "IRFR3607PBF" H -600 3050 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H -1050 3200 29  0001 C CNN
F 3 "" H -1250 3100 60  0000 C CNN
	1    -1250 3100
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 561D374D
P -1250 2650
F 0 "R1" V -1170 2650 50  0000 C CNN
F 1 "1K" V -1250 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -1320 2650 30  0001 C CNN
F 3 "" H -1250 2650 30  0000 C CNN
	1    -1250 2650
	1    0    0    -1  
$EndComp
$Comp
L +15V #PWR012
U 1 1 561D38B8
P -1250 2400
F 0 "#PWR012" H -1250 2250 50  0001 C CNN
F 1 "+15V" H -1250 2540 50  0000 C CNN
F 2 "" H -1250 2400 60  0000 C CNN
F 3 "" H -1250 2400 60  0000 C CNN
	1    -1250 2400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR013
U 1 1 561D44D6
P 7650 6000
F 0 "#PWR013" H 7650 5800 50  0001 C CNN
F 1 "GNDPWR" H 7650 5870 50  0000 C CNN
F 2 "" H 7650 5950 60  0000 C CNN
F 3 "" H 7650 5950 60  0000 C CNN
	1    7650 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 561D4D27
P -950 3250
F 0 "#PWR014" H -950 3000 50  0001 C CNN
F 1 "GND" H -950 3100 50  0000 C CNN
F 2 "" H -950 3250 60  0000 C CNN
F 3 "" H -950 3250 60  0000 C CNN
	1    -950 3250
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR015
U 1 1 561D4EED
P -1550 3250
F 0 "#PWR015" H -1550 3050 50  0001 C CNN
F 1 "GNDPWR" H -1550 3120 50  0000 C CNN
F 2 "" H -1550 3200 60  0000 C CNN
F 3 "" H -1550 3200 60  0000 C CNN
	1    -1550 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1250 2900 -1250 2800
Wire Wire Line
	-1250 2500 -1250 2400
Wire Wire Line
	-1450 3200 -1550 3200
Wire Wire Line
	-1550 3200 -1550 3250
Wire Wire Line
	-1050 3200 -950 3200
Wire Wire Line
	-950 3200 -950 3250
$Comp
L GND #PWR016
U 1 1 561D6318
P 6950 5100
F 0 "#PWR016" H 6950 4850 50  0001 C CNN
F 1 "GND" H 6950 4950 50  0000 C CNN
F 2 "" H 6950 5100 60  0000 C CNN
F 3 "" H 6950 5100 60  0000 C CNN
	1    6950 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 561D645E
P 7850 4950
F 0 "#PWR017" H 7850 4700 50  0001 C CNN
F 1 "GND" H 7850 4800 50  0000 C CNN
F 2 "" H 7850 4950 60  0000 C CNN
F 3 "" H 7850 4950 60  0000 C CNN
	1    7850 4950
	1    0    0    -1  
$EndComp
$Comp
L AVR-ISP-6 CON1
U 1 1 561D79AA
P 7400 2200
F 0 "CON1" H 7295 2440 50  0000 C CNN
F 1 "AVR-ISP-6" H 7135 1970 50  0000 L BNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" V 6880 2240 50  0001 C CNN
F 3 "" H 7375 2200 60  0000 C CNN
	1    7400 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2300 5900 2300
Wire Wire Line
	5650 2400 5900 2400
Wire Wire Line
	5650 2200 5900 2200
Text Label 5800 2200 0    40   ~ 0
SCK
Text Label 5800 2300 0    40   ~ 0
MOSI
Text Label 5800 2400 0    40   ~ 0
MISO
Wire Wire Line
	7250 2100 6950 2100
Wire Wire Line
	7250 2200 6950 2200
Wire Wire Line
	7500 2100 7800 2100
Wire Wire Line
	7500 2200 7800 2200
Wire Wire Line
	7500 2300 7700 2300
Wire Wire Line
	7700 2300 7700 2400
Text Label 6950 2100 0    40   ~ 0
MISO
Text Label 6950 2200 0    40   ~ 0
SCK
Text Label -1350 4650 0    40   ~ 0
VCC
Text Label 7800 2100 0    40   ~ 0
VCC
Text Label 7800 2200 0    40   ~ 0
MOSI
$Comp
L GND #PWR018
U 1 1 561DA91D
P 7700 2400
F 0 "#PWR018" H 7700 2150 50  0001 C CNN
F 1 "GND" H 7700 2250 50  0000 C CNN
F 2 "" H 7700 2400 60  0000 C CNN
F 3 "" H 7700 2400 60  0000 C CNN
	1    7700 2400
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P1
U 1 1 561DB426
P 7900 1050
F 0 "P1" H 7900 1400 50  0000 C CNN
F 1 "FTDI" V 8000 1050 50  0000 C CNN
F 2 "Connect:PINHEAD1-6" H 7900 1050 60  0001 C CNN
F 3 "" H 7900 1050 60  0000 C CNN
	1    7900 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 800  7500 800 
Text Label 7500 800  0    40   ~ 0
GND
Text Label 4850 750  0    40   ~ 0
GND
Wire Wire Line
	10600 1150 10850 1150
Text Label 10750 1150 0    40   ~ 0
CC1
$Comp
L R_Small R5
U 1 1 561E1943
P 7500 1100
F 0 "R5" V 7550 1150 50  0000 L CNN
F 1 "1K" V 7530 1060 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7500 1100 60  0001 C CNN
F 3 "" H 7500 1100 60  0000 C CNN
	1    7500 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1100 7600 1100
$Comp
L R_Small R6
U 1 1 561E2071
P 7500 1200
F 0 "R6" V 7550 1250 50  0000 L CNN
F 1 "1K" V 7530 1160 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7500 1200 60  0001 C CNN
F 3 "" H 7500 1200 60  0000 C CNN
	1    7500 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1200 7600 1200
Wire Wire Line
	7400 1100 7200 1100
Wire Wire Line
	7400 1200 7200 1200
Text Label 7200 1100 0    40   ~ 0
TX0
Text Label 7200 1200 0    40   ~ 0
RX0
$Comp
L R_Small R4
U 1 1 561E2CCA
P 7500 1000
F 0 "R4" V 7550 1050 50  0000 L CNN
F 1 "0" V 7530 960 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7500 1000 60  0001 C CNN
F 3 "" H 7500 1000 60  0000 C CNN
	1    7500 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1000 7600 1000
Wire Wire Line
	7400 1000 7200 1000
Text Label 7200 1000 0    40   ~ 0
VCC
$Comp
L C_Small C3
U 1 1 561E3D19
P -500 1700
F 0 "C3" H -490 1770 50  0000 L CNN
F 1 "22pF" H -490 1620 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -500 1700 60  0001 C CNN
F 3 "" H -500 1700 60  0000 C CNN
	1    -500 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 561E3E82
P -500 1800
F 0 "#PWR019" H -500 1550 50  0001 C CNN
F 1 "GND" H -500 1650 50  0000 C CNN
F 2 "" H -500 1800 60  0000 C CNN
F 3 "" H -500 1800 60  0000 C CNN
	1    -500 1800
	1    0    0    -1  
$EndComp
Connection ~ -500 1600
$Comp
L R_Small R2
U 1 1 561E4303
P -550 1400
F 0 "R2" H -520 1420 50  0000 L CNN
F 1 "10k" H -520 1360 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H -550 1400 60  0001 C CNN
F 3 "" H -550 1400 60  0000 C CNN
	1    -550 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	-550 1500 -550 1600
Connection ~ -550 1600
Wire Wire Line
	-550 1300 -550 1200
Wire Wire Line
	-550 1200 -450 1200
Text Label -450 1200 0    40   ~ 0
VCC
Wire Wire Line
	5650 4800 5900 4800
Wire Wire Line
	5650 4900 5900 4900
Text Label 5800 4800 0    40   ~ 0
RX0
Text Label 5800 4900 0    40   ~ 0
TX0
Wire Wire Line
	5650 5200 5900 5200
Text Label 5800 5200 0    40   ~ 0
CC1
Wire Wire Line
	10600 1300 10850 1300
Text Label 10750 1300 0    40   ~ 0
DC1
Wire Wire Line
	5650 5300 5900 5300
Text Label 5800 5300 0    40   ~ 0
DC1
Wire Wire Line
	10600 1450 10850 1450
Text Label 10750 1450 0    40   ~ 0
DCL1
Wire Wire Line
	5650 1200 5900 1200
Text Label 5800 1200 0    40   ~ 0
DCL1
Wire Wire Line
	10600 1600 10850 1600
Text Label 10750 1600 0    40   ~ 0
VBAT1
Wire Wire Line
	5650 5700 5900 5700
Text Label 5800 5700 0    40   ~ 0
VBAT1
Wire Wire Line
	10600 1900 10850 1900
Text Label 10750 1900 0    40   ~ 0
I1
Wire Wire Line
	5650 5800 5900 5800
Text Label 5800 5800 0    40   ~ 0
I1
Text Label 7600 4600 0    40   ~ 0
+15V
Text Label 7500 5500 0    40   ~ 0
-15V
Wire Wire Line
	10600 2050 10850 2050
Wire Wire Line
	10600 2200 10850 2200
Wire Wire Line
	10600 2350 10850 2350
Text Label 10750 2050 0    40   ~ 0
+15V
Text Label 10750 2200 0    40   ~ 0
GND
Text Label 10750 2350 0    40   ~ 0
-15V
Wire Wire Line
	9200 3500 8900 3500
Wire Wire Line
	9200 3650 8900 3650
Wire Wire Line
	9200 3800 8900 3800
Wire Wire Line
	9200 3950 8900 3950
Wire Wire Line
	9050 2050 8800 2050
Text Label 8900 3500 0    40   ~ 0
+15V
Text Label 8900 3650 0    40   ~ 0
GND
Text Label 8900 3800 0    40   ~ 0
-15V
Text Label 8900 3950 0    40   ~ 0
VCC
Text Label 8800 2050 0    40   ~ 0
VCC
Wire Wire Line
	9200 4700 8900 4700
Text Label 8900 4700 0    40   ~ 0
VCC
Wire Wire Line
	10600 5600 10850 5600
Wire Wire Line
	10600 5750 10850 5750
Wire Wire Line
	10600 5900 10850 5900
Text Label 10750 5600 0    40   ~ 0
+15V
Text Label 10750 5750 0    40   ~ 0
GND
Text Label 10750 5900 0    40   ~ 0
-15V
Wire Wire Line
	9050 1150 8800 1150
Wire Wire Line
	9050 1300 8800 1300
Wire Wire Line
	9050 1450 8800 1450
Wire Wire Line
	9050 1600 8800 1600
Wire Wire Line
	9050 1900 8800 1900
Wire Wire Line
	3050 6200 2800 6200
Text Label 2800 6200 0    40   ~ 0
CC2
Text Label 8800 1150 0    40   ~ 0
CC2
Text Label 8800 1300 0    40   ~ 0
DC2
Wire Wire Line
	5650 5100 5900 5100
Text Label 5800 5100 0    40   ~ 0
DC2
Text Label 8800 1450 0    40   ~ 0
DCL2
Wire Wire Line
	5650 1300 5900 1300
Text Label 5800 1300 0    40   ~ 0
DCL2
Text Label 8800 1600 0    40   ~ 0
VBAT2
Wire Wire Line
	5650 5900 5900 5900
Wire Wire Line
	5650 6000 5900 6000
Text Label 5800 5900 0    40   ~ 0
VBAT2
Text Label 5800 6000 0    40   ~ 0
I2
Text Label 8800 1900 0    40   ~ 0
I2
Wire Wire Line
	10600 3050 10850 3050
Wire Wire Line
	10600 3200 10850 3200
Wire Wire Line
	10600 3500 10850 3500
Wire Wire Line
	10600 3650 10850 3650
Wire Wire Line
	10600 3800 10850 3800
Wire Wire Line
	10600 3950 10850 3950
Wire Wire Line
	10600 4100 10850 4100
Wire Wire Line
	9200 3050 8900 3050
Wire Wire Line
	9200 3200 8900 3200
Wire Wire Line
	9200 3350 8900 3350
Text Label 10850 3050 0    40   ~ 0
CC3
Text Label 10850 3200 0    40   ~ 0
DC3
Text Label 10850 3650 0    40   ~ 0
DCL3
Text Label 10850 3800 0    40   ~ 0
VBAT3
Text Label 10850 3500 0    40   ~ 0
I3
Text Label 10850 3950 0    40   ~ 0
CC4
Text Label 10850 4100 0    40   ~ 0
DC4
Text Label 8900 3200 0    40   ~ 0
DCL4
Text Label 8900 3350 0    40   ~ 0
VBAT4
Text Label 8900 3050 0    40   ~ 0
I4
Wire Wire Line
	3050 5100 2750 5100
Wire Wire Line
	3050 5200 2750 5200
Text Label 2750 5100 0    40   ~ 0
CC3
Text Label 2750 5200 0    40   ~ 0
DC3
Text Notes 5950 5200 0    40   ~ 0
D2
Text Notes 5950 5300 0    40   ~ 0
D3
Text Notes 5950 4800 0    40   ~ 0
D0
Text Notes 5950 4900 0    40   ~ 0
D1
Text Notes 2650 6200 0    40   ~ 0
D4
Text Notes 5950 5100 0    40   ~ 0
D5
Text Notes 6000 1200 0    40   ~ 0
D22
Text Notes 6000 1300 0    40   ~ 0
D23
Text Notes 2600 5100 0    40   ~ 0
D6
Text Notes 2600 5200 0    40   ~ 0
D7
Text Notes 6000 5700 0    40   ~ 0
A0
Text Notes 6000 5800 0    40   ~ 0
A1
Text Notes 6000 5900 0    40   ~ 0
A2
Text Notes 6000 6000 0    40   ~ 0
A3
Wire Wire Line
	5650 6100 5900 6100
Text Label 5800 6100 0    40   ~ 0
VBAT3
Wire Wire Line
	5650 6200 5900 6200
Text Label 5800 6200 0    40   ~ 0
I3
Text Notes 6000 6100 0    40   ~ 0
A4
Text Notes 6000 6200 0    40   ~ 0
A5
Wire Wire Line
	5650 1400 5900 1400
Text Label 5800 1400 0    40   ~ 0
DCL3
Text Notes 6000 1400 0    40   ~ 0
D24
Wire Wire Line
	3050 5300 2750 5300
Text Label 2750 5300 0    40   ~ 0
CC4
Wire Wire Line
	3050 5400 2750 5400
Text Label 2750 5400 0    40   ~ 0
DC4
Text Notes 2600 5300 0    40   ~ 0
D8
Text Notes 2600 5400 0    40   ~ 0
D9
Wire Wire Line
	5650 1500 5900 1500
Text Label 5800 1500 0    40   ~ 0
DCL4
Text Notes 6000 1500 0    40   ~ 0
D25
Wire Wire Line
	5650 6300 5900 6300
Wire Wire Line
	5650 6400 5900 6400
Text Label 5800 6300 0    40   ~ 0
VBAT4
Text Label 5800 6400 0    40   ~ 0
I4
Text Notes 6000 6300 0    40   ~ 0
A6
Text Notes 6000 6400 0    40   ~ 0
A7
Wire Wire Line
	10600 4700 10850 4700
Wire Wire Line
	10600 4850 10850 4850
Wire Wire Line
	10600 5000 10850 5000
Text Label 10850 4700 0    40   ~ 0
CC5
Text Label 10850 4850 0    40   ~ 0
DC5
Wire Wire Line
	5650 2500 5900 2500
Wire Wire Line
	5650 2600 5900 2600
Text Label 5800 2500 0    40   ~ 0
CC5
Text Label 5800 2600 0    40   ~ 0
DC5
Text Notes 6000 2500 0    40   ~ 0
D10
Text Notes 6000 2600 0    40   ~ 0
D11
Wire Wire Line
	10600 5150 10850 5150
Text Label 10850 5150 0    40   ~ 0
VBAT5
Text Label 10850 5000 0    40   ~ 0
I5
Wire Wire Line
	3050 3000 2800 3000
Text Label 2800 3000 0    40   ~ 0
VBAT5
Wire Wire Line
	3050 3100 2800 3100
Text Label 2800 3100 0    40   ~ 0
I5
Text Notes 2650 3000 0    40   ~ 0
A8
Text Notes 2650 3100 0    40   ~ 0
A9
Wire Wire Line
	10600 5450 10850 5450
Text Label 10850 5450 0    40   ~ 0
DCL5
Wire Wire Line
	5650 1600 5900 1600
Text Label 5800 1600 0    40   ~ 0
DCL5
Text Notes 6000 1600 0    40   ~ 0
D26
Text Notes 6000 2400 0    40   ~ 0
D50
Text Notes 6000 2300 0    40   ~ 0
D51
Text Notes 6000 2200 0    40   ~ 0
D52
$Comp
L CONN_01X05 P3
U 1 1 56202634
P 12900 2150
F 0 "P3" H 12900 2450 50  0000 C CNN
F 1 "BAT1_TP" V 13000 2150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 12900 2150 60  0001 C CNN
F 3 "" H 12900 2150 60  0000 C CNN
	1    12900 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	12700 1950 12350 1950
Wire Wire Line
	12700 2050 12350 2050
Wire Wire Line
	12700 2150 12350 2150
Wire Wire Line
	12700 2250 12350 2250
Wire Wire Line
	12700 2350 12350 2350
Text Label 12350 1950 0    40   ~ 0
CC1
Text Label 12350 2050 0    40   ~ 0
DC1
Text Label 12350 2150 0    40   ~ 0
DCL1
Text Label 12350 2250 0    40   ~ 0
VBAT1
Text Label 12350 2350 0    40   ~ 0
I1
$Comp
L CONN_01X05 P4
U 1 1 56203610
P 12900 2800
F 0 "P4" H 12900 3100 50  0000 C CNN
F 1 "BAT3_TP" V 13000 2800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 12900 2800 60  0001 C CNN
F 3 "" H 12900 2800 60  0000 C CNN
	1    12900 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	12700 2600 12350 2600
Wire Wire Line
	12700 2700 12350 2700
Wire Wire Line
	12700 2800 12350 2800
Wire Wire Line
	12700 2900 12350 2900
Wire Wire Line
	12700 3000 12350 3000
Text Label 12350 2600 0    40   ~ 0
CC3
Text Label 12350 2700 0    40   ~ 0
DC3
Text Label 12350 2800 0    40   ~ 0
DCL3
Text Label 12350 2900 0    40   ~ 0
VBAT3
Text Label 12350 3000 0    40   ~ 0
I3
$Comp
L CONN_01X05 P5
U 1 1 5620374F
P 12900 3450
F 0 "P5" H 12900 3750 50  0000 C CNN
F 1 "BAT5_TP" V 13000 3450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 12900 3450 60  0001 C CNN
F 3 "" H 12900 3450 60  0000 C CNN
	1    12900 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	12700 3250 12350 3250
Wire Wire Line
	12700 3350 12350 3350
Wire Wire Line
	12700 3450 12350 3450
Wire Wire Line
	12700 3550 12350 3550
Wire Wire Line
	12700 3650 12350 3650
Text Label 12350 3250 0    40   ~ 0
CC5
Text Label 12350 3350 0    40   ~ 0
DC5
Text Label 12350 3450 0    40   ~ 0
DCL5
Text Label 12350 3550 0    40   ~ 0
VBAT5
Text Label 12350 3650 0    40   ~ 0
I5
$Comp
L CONN_01X05 P6
U 1 1 56203892
P 13800 2150
F 0 "P6" H 13800 2450 50  0000 C CNN
F 1 "BAT2_TP" V 13900 2150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 13800 2150 60  0001 C CNN
F 3 "" H 13800 2150 60  0000 C CNN
	1    13800 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	13600 1950 13250 1950
Wire Wire Line
	13600 2050 13250 2050
Wire Wire Line
	13600 2150 13250 2150
Wire Wire Line
	13600 2250 13250 2250
Wire Wire Line
	13600 2350 13250 2350
Text Label 13250 1950 0    40   ~ 0
CC2
Text Label 13250 2050 0    40   ~ 0
DC2
Text Label 13250 2150 0    40   ~ 0
DCL2
Text Label 13250 2250 0    40   ~ 0
VBAT2
Text Label 13250 2350 0    40   ~ 0
I2
$Comp
L CONN_01X05 P7
U 1 1 562039D6
P 13800 2800
F 0 "P7" H 13800 3100 50  0000 C CNN
F 1 "BAT4_TP" V 13900 2800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 13800 2800 60  0001 C CNN
F 3 "" H 13800 2800 60  0000 C CNN
	1    13800 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	13600 2600 13250 2600
Wire Wire Line
	13600 2700 13250 2700
Wire Wire Line
	13600 2800 13250 2800
Wire Wire Line
	13600 2900 13250 2900
Wire Wire Line
	13600 3000 13250 3000
Text Label 13250 2600 0    40   ~ 0
CC4
Text Label 13250 2700 0    40   ~ 0
DC4
Text Label 13250 2800 0    40   ~ 0
DCL4
Text Label 13250 2900 0    40   ~ 0
VBAT4
Text Label 13250 3000 0    40   ~ 0
I4
Text Label 4250 550  0    40   ~ 0
VCC
$Comp
L CONN_01X01 P2
U 1 1 56218B4B
P 8000 5200
F 0 "P2" H 8000 5300 50  0000 C CNN
F 1 "+15V" V 8100 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 8000 5200 60  0001 C CNN
F 3 "" H 8000 5200 60  0000 C CNN
	1    8000 5200
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P13
U 1 1 56218E68
P 8000 5500
F 0 "P13" H 8000 5600 50  0000 C CNN
F 1 "-15V" V 8100 5500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 8000 5500 60  0001 C CNN
F 3 "" H 8000 5500 60  0000 C CNN
	1    8000 5500
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P14
U 1 1 56219049
P 8000 5900
F 0 "P14" H 8000 6000 50  0000 C CNN
F 1 "GND" V 8100 5900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 8000 5900 60  0001 C CNN
F 3 "" H 8000 5900 60  0000 C CNN
	1    8000 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4200 5900 4200
Wire Wire Line
	5650 4100 5900 4100
Text Label 5800 4100 0    40   ~ 0
RX1
Text Label 5800 4200 0    40   ~ 0
TX1
Text Notes 5950 4200 0    40   ~ 0
D18
Text Notes 5950 4100 0    40   ~ 0
D19
Wire Wire Line
	3050 4800 2750 4800
Wire Wire Line
	3050 4900 2750 4900
Text Label 2750 4800 0    40   ~ 0
RX2
Text Label 2750 4900 0    40   ~ 0
TX2
Text Notes 2600 4900 0    40   ~ 0
D16
Text Notes 2600 4800 0    40   ~ 0
D17
Wire Wire Line
	3050 3900 2750 3900
Wire Wire Line
	3050 4000 2750 4000
Text Label 2750 3900 0    40   ~ 0
RX3
Text Label 2750 4000 0    40   ~ 0
TX3
Text Notes 2600 4000 0    40   ~ 0
D14
Text Notes 2600 3900 0    40   ~ 0
D15
$Comp
L CONN_01X06 P15
U 1 1 562255B5
P 7450 3550
F 0 "P15" H 7450 3900 50  0000 C CNN
F 1 "AUX" V 7550 3550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7450 3550 60  0001 C CNN
F 3 "" H 7450 3550 60  0000 C CNN
	1    7450 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3300 7050 3300
Wire Wire Line
	7250 3400 7050 3400
Wire Wire Line
	7250 3500 7050 3500
Wire Wire Line
	7250 3600 7050 3600
Wire Wire Line
	7250 3700 7050 3700
Wire Wire Line
	7250 3800 7050 3800
Text Label 7050 3300 0    40   ~ 0
TX1
Text Label 7050 3400 0    40   ~ 0
RX1
Text Label 7050 3500 0    40   ~ 0
TX2
Text Label 7050 3600 0    40   ~ 0
RX2
Text Label 7050 3700 0    40   ~ 0
TX3
Text Label 7050 3800 0    40   ~ 0
RX3
Wire Wire Line
	7250 2300 6950 2300
Text Label 6950 2300 0    40   ~ 0
RESET
$EndSCHEMATC
