EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L MCU_Module:Arduino_Nano_v2.x A1
U 1 1 5DE6D56C
P 1400 1800
F 0 "A1" H 1400 711 50  0000 C CNN
F 1 "Arduino_Nano_v2.x" H 1400 620 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 1550 850 50  0001 L CNN
F 3 "https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf" H 1400 800 50  0001 C CNN
	1    1400 1800
	1    0    0    -1  
$EndComp
$Comp
L RelayDeltaphi:HFD2 K1
U 1 1 5DE6E08D
P 3950 1700
F 0 "K1" H 3975 2415 50  0000 C CNN
F 1 "HFD2" H 3975 2324 50  0000 C CNN
F 2 "RelayDeltaphi:HFD" V 4150 1700 50  0001 C CNN
F 3 "" V 4150 1700 50  0001 C CNN
	1    3950 1700
	1    0    0    -1  
$EndComp
$Comp
L RelayDeltaphi:HFD2-L2 K2
U 1 1 5DE6EA0C
P 3950 3100
F 0 "K2" H 3975 3815 50  0000 C CNN
F 1 "HFD2-L2" H 3975 3724 50  0000 C CNN
F 2 "RelayDeltaphi:HFD2-L2" V 4150 3100 50  0001 C CNN
F 3 "" V 4150 3100 50  0001 C CNN
	1    3950 3100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003A U1
U 1 1 5DE6F7B1
P 2650 2200
F 0 "U1" H 2650 2867 50  0000 C CNN
F 1 "ULN2003A" H 2650 2776 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 2700 1650 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 2750 2000 50  0001 C CNN
	1    2650 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 5DE7097C
P 6100 1550
F 0 "J2" H 6180 1542 50  0000 L CNN
F 1 "Screw_Terminal_01x04" H 6180 1451 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00003_1x04_P5.00mm_Horizontal" H 6100 1550 50  0001 C CNN
F 3 "~" H 6100 1550 50  0001 C CNN
	1    6100 1550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 5DE71705
P 4750 4250
F 0 "J1" H 4830 4292 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 4830 4201 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00002_1x03_P5.00mm_Horizontal" H 4750 4250 50  0001 C CNN
F 3 "~" H 4750 4250 50  0001 C CNN
	1    4750 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Bridge_+-AA D1
U 1 1 5DE72946
P 5150 1100
F 0 "D1" H 5494 1146 50  0000 L CNN
F 1 "D_Bridge_+-AA" H 5494 1055 50  0000 L CNN
F 2 "Diode_THT:Diode_Bridge_19.0x3.5x10.0mm_P5.0mm" H 5150 1100 50  0001 C CNN
F 3 "~" H 5150 1100 50  0001 C CNN
	1    5150 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Bridge_+-AA D2
U 1 1 5DE750E7
P 5150 2050
F 0 "D2" H 5494 2096 50  0000 L CNN
F 1 "D_Bridge_+-AA" H 5494 2005 50  0000 L CNN
F 2 "Diode_THT:Diode_Bridge_19.0x3.5x10.0mm_P5.0mm" H 5150 2050 50  0001 C CNN
F 3 "~" H 5150 2050 50  0001 C CNN
	1    5150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1450 5900 800 
Wire Wire Line
	5900 800  5150 800 
Wire Wire Line
	5900 1550 5150 1550
Wire Wire Line
	5150 1550 5150 1400
Wire Wire Line
	5900 1650 5150 1650
Wire Wire Line
	5150 1650 5150 1750
Wire Wire Line
	5900 1750 5900 2350
Wire Wire Line
	5900 2350 5150 2350
$Comp
L power:GND #PWR0101
U 1 1 5DE76479
P 4850 1300
F 0 "#PWR0101" H 4850 1050 50  0001 C CNN
F 1 "GND" H 4855 1127 50  0000 C CNN
F 2 "" H 4850 1300 50  0001 C CNN
F 3 "" H 4850 1300 50  0001 C CNN
	1    4850 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1300 4850 1100
$Comp
L power:GND #PWR0102
U 1 1 5DE76DEF
P 1400 3100
F 0 "#PWR0102" H 1400 2850 50  0001 C CNN
F 1 "GND" H 1405 2927 50  0000 C CNN
F 2 "" H 1400 3100 50  0001 C CNN
F 3 "" H 1400 3100 50  0001 C CNN
	1    1400 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3100 1400 2800
Wire Wire Line
	1500 2800 1400 2800
Connection ~ 1400 2800
$Comp
L power:VCC #PWR0103
U 1 1 5DE77C6F
P 1600 700
F 0 "#PWR0103" H 1600 550 50  0001 C CNN
F 1 "VCC" H 1617 873 50  0000 C CNN
F 2 "" H 1600 700 50  0001 C CNN
F 3 "" H 1600 700 50  0001 C CNN
	1    1600 700 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0105
U 1 1 5DE7904F
P 4350 1050
F 0 "#PWR0105" H 4350 900 50  0001 C CNN
F 1 "VCC" H 4367 1223 50  0000 C CNN
F 2 "" H 4350 1050 50  0001 C CNN
F 3 "" H 4350 1050 50  0001 C CNN
	1    4350 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1050 4350 1300
$Comp
L power:VCC #PWR0106
U 1 1 5DE79979
P 4350 2550
F 0 "#PWR0106" H 4350 2400 50  0001 C CNN
F 1 "VCC" H 4367 2723 50  0000 C CNN
F 2 "" H 4350 2550 50  0001 C CNN
F 3 "" H 4350 2550 50  0001 C CNN
	1    4350 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2550 4350 2700
Connection ~ 4350 2700
Wire Wire Line
	4350 2700 4350 2950
Wire Wire Line
	3050 2400 3300 2400
Wire Wire Line
	3300 2400 3300 2950
Wire Wire Line
	3300 2950 3600 2950
Wire Wire Line
	3050 2300 3400 2300
Wire Wire Line
	3400 2300 3400 2700
Wire Wire Line
	3400 2700 3600 2700
Wire Wire Line
	3050 2500 3200 2500
Wire Wire Line
	3200 2500 3200 1300
Wire Wire Line
	3200 1300 3600 1300
Wire Wire Line
	4350 1800 4500 1800
Wire Wire Line
	3600 1800 3600 1650
Wire Wire Line
	3600 1650 4350 1650
Wire Wire Line
	4350 1650 4350 1800
Connection ~ 4350 1800
Wire Wire Line
	4500 4350 4500 1800
Wire Wire Line
	4350 3200 4450 3200
Wire Wire Line
	4450 3200 4450 4250
Wire Wire Line
	3400 3200 3400 4150
Wire Wire Line
	3600 2200 3600 2300
Wire Wire Line
	3600 2300 4350 2300
Wire Wire Line
	4350 2300 4350 2200
Wire Wire Line
	4350 2200 4600 2200
Wire Wire Line
	4600 2200 4600 2050
Wire Wire Line
	4600 2050 4850 2050
Connection ~ 4350 2200
Wire Wire Line
	4600 2200 4600 3400
Wire Wire Line
	4600 3400 4350 3400
Connection ~ 4600 2200
Wire Wire Line
	4600 3400 4600 3700
Wire Wire Line
	4600 3700 3600 3700
Wire Wire Line
	3600 3700 3600 3600
Connection ~ 4600 3400
Wire Wire Line
	5450 2050 5450 3600
Wire Wire Line
	5450 3600 4350 3600
Wire Wire Line
	3600 3200 3400 3200
Wire Wire Line
	5450 3600 5450 3800
Wire Wire Line
	5450 3800 3500 3800
Wire Wire Line
	3500 3800 3500 3400
Wire Wire Line
	3500 3400 3600 3400
Connection ~ 5450 3600
$Comp
L Device:CP C1
U 1 1 5DE9E80E
P 2650 950
F 0 "C1" H 2768 996 50  0000 L CNN
F 1 "470u" H 2768 905 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm" H 2688 800 50  0001 C CNN
F 3 "~" H 2650 950 50  0001 C CNN
	1    2650 950 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0107
U 1 1 5DE9F2B1
P 2650 700
F 0 "#PWR0107" H 2650 550 50  0001 C CNN
F 1 "VCC" H 2667 873 50  0000 C CNN
F 2 "" H 2650 700 50  0001 C CNN
F 3 "" H 2650 700 50  0001 C CNN
	1    2650 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 700  2650 800 
$Comp
L power:GND #PWR0108
U 1 1 5DEA0E7E
P 2650 1200
F 0 "#PWR0108" H 2650 950 50  0001 C CNN
F 1 "GND" H 2655 1027 50  0000 C CNN
F 2 "" H 2650 1200 50  0001 C CNN
F 3 "" H 2650 1200 50  0001 C CNN
	1    2650 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1100 2650 1200
$Comp
L power:GND #PWR0109
U 1 1 5DEAA8F6
P 2650 2900
F 0 "#PWR0109" H 2650 2650 50  0001 C CNN
F 1 "GND" H 2655 2727 50  0000 C CNN
F 2 "" H 2650 2900 50  0001 C CNN
F 3 "" H 2650 2900 50  0001 C CNN
	1    2650 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2900 2650 2800
$Comp
L power:GND #PWR0110
U 1 1 5DEAC98B
P 3300 1800
F 0 "#PWR0110" H 3300 1550 50  0001 C CNN
F 1 "GND" H 3305 1627 50  0000 C CNN
F 2 "" H 3300 1800 50  0001 C CNN
F 3 "" H 3300 1800 50  0001 C CNN
	1    3300 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1800 3300 1800
Wire Wire Line
	900  1900 850  1900
Wire Wire Line
	850  1900 850  3400
Wire Wire Line
	850  3400 2100 3400
Wire Wire Line
	2100 3400 2100 2400
Wire Wire Line
	2100 2400 2250 2400
Wire Wire Line
	900  1800 800  1800
Wire Wire Line
	800  1800 800  3450
Wire Wire Line
	800  3450 2150 3450
Wire Wire Line
	2150 3450 2150 2500
Wire Wire Line
	2150 2500 2250 2500
Wire Wire Line
	900  1700 750  1700
Wire Wire Line
	750  1700 750  3500
Wire Wire Line
	750  3500 2200 3500
Wire Wire Line
	2200 3500 2200 2600
Wire Wire Line
	2200 2600 2250 2600
Wire Wire Line
	3400 4150 4550 4150
Wire Wire Line
	4450 4250 4550 4250
Wire Wire Line
	4500 4350 4550 4350
Wire Wire Line
	1600 700  1600 800 
Wire Wire Line
	1300 800  1300 500 
Wire Wire Line
	1300 500  5450 500 
Wire Wire Line
	5450 500  5450 1100
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5DECEEFE
P 6900 900
F 0 "H1" H 7000 949 50  0000 L CNN
F 1 "MountingHole_Pad" H 7000 858 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6900 900 50  0001 C CNN
F 3 "~" H 6900 900 50  0001 C CNN
	1    6900 900 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5DECF596
P 6900 1200
F 0 "H2" H 7000 1249 50  0000 L CNN
F 1 "MountingHole_Pad" H 7000 1158 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6900 1200 50  0001 C CNN
F 3 "~" H 6900 1200 50  0001 C CNN
	1    6900 1200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5DECF7BE
P 6900 1500
F 0 "H3" H 7000 1549 50  0000 L CNN
F 1 "MountingHole_Pad" H 7000 1458 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6900 1500 50  0001 C CNN
F 3 "~" H 6900 1500 50  0001 C CNN
	1    6900 1500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5DECFA24
P 6900 1800
F 0 "H4" H 7000 1849 50  0000 L CNN
F 1 "MountingHole_Pad" H 7000 1758 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6900 1800 50  0001 C CNN
F 3 "~" H 6900 1800 50  0001 C CNN
	1    6900 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5DECFD19
P 6700 2050
F 0 "#PWR0104" H 6700 1800 50  0001 C CNN
F 1 "GND" H 6705 1877 50  0000 C CNN
F 2 "" H 6700 2050 50  0001 C CNN
F 3 "" H 6700 2050 50  0001 C CNN
	1    6700 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2050 6700 1900
Wire Wire Line
	6700 1000 6900 1000
Wire Wire Line
	6900 1300 6700 1300
Connection ~ 6700 1300
Wire Wire Line
	6700 1300 6700 1000
Wire Wire Line
	6900 1600 6700 1600
Connection ~ 6700 1600
Wire Wire Line
	6700 1600 6700 1300
Wire Wire Line
	6900 1900 6700 1900
Connection ~ 6700 1900
Wire Wire Line
	6700 1900 6700 1600
$EndSCHEMATC
