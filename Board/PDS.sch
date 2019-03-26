EESchema Schematic File Version 4
LIBS:PDS-cache
EELAYER 26 0
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
L Switch:SW_Push_Dual SW1
U 1 1 5C475541
P 1600 5500
F 0 "SW1" H 1600 5785 50  0000 C CNN
F 1 "SW_Push_Dual" H 1600 5694 50  0000 C CNN
F 2 "KUT_Switch:SW_SPST_SKRPACE010" H 1600 5700 50  0001 C CNN
F 3 "" H 1600 5700 50  0001 C CNN
	1    1600 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5500 1950 5500
$Comp
L Device:C_Small C7
U 1 1 5C4759EA
P 1950 5600
F 0 "C7" H 2042 5646 50  0000 L CNN
F 1 "0.1u" H 2042 5555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1950 5600 50  0001 C CNN
F 3 "~" H 1950 5600 50  0001 C CNN
	1    1950 5600
	1    0    0    -1  
$EndComp
Connection ~ 1950 5500
Wire Wire Line
	1950 5500 2200 5500
$Comp
L power:GND #PWR0101
U 1 1 5C47647A
P 1950 5750
F 0 "#PWR0101" H 1950 5500 50  0001 C CNN
F 1 "GND" H 1955 5577 50  0000 C CNN
F 2 "" H 1950 5750 50  0001 C CNN
F 3 "" H 1950 5750 50  0001 C CNN
	1    1950 5750
	1    0    0    -1  
$EndComp
NoConn ~ 1400 5500
$Comp
L power:GND #PWR0102
U 1 1 5C477481
P 1800 5750
F 0 "#PWR0102" H 1800 5500 50  0001 C CNN
F 1 "GND" H 1805 5577 50  0000 C CNN
F 2 "" H 1800 5750 50  0001 C CNN
F 3 "" H 1800 5750 50  0001 C CNN
	1    1800 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C47749E
P 1400 5750
F 0 "#PWR0103" H 1400 5500 50  0001 C CNN
F 1 "GND" H 1405 5577 50  0000 C CNN
F 2 "" H 1400 5750 50  0001 C CNN
F 3 "" H 1400 5750 50  0001 C CNN
	1    1400 5750
	1    0    0    -1  
$EndComp
Text Label 2200 5500 2    50   ~ 0
RESET
$Comp
L power:GND #PWR0104
U 1 1 5C4782C4
P 2650 7300
F 0 "#PWR0104" H 2650 7050 50  0001 C CNN
F 1 "GND" H 2655 7127 50  0000 C CNN
F 2 "" H 2650 7300 50  0001 C CNN
F 3 "" H 2650 7300 50  0001 C CNN
	1    2650 7300
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F042K6Tx U1
U 1 1 5C47AE79
P 2700 6200
F 0 "U1" H 2350 7050 50  0000 C CNN
F 1 "STM32F042K6Tx" H 2600 6550 50  0000 C CNN
F 2 "Package_QFP:LQFP-32_7x7mm_P0.8mm" H 2300 5300 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00105814.pdf" H 2700 6200 50  0001 C CNN
	1    2700 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 7000 2200 7000
Wire Wire Line
	2650 7300 2650 7250
Wire Wire Line
	2650 7250 2600 7250
Wire Wire Line
	2600 7250 2600 7200
Wire Wire Line
	2650 7250 2700 7250
Wire Wire Line
	2700 7250 2700 7200
Connection ~ 2650 7250
$Comp
L power:GND #PWR0105
U 1 1 5C47C0EA
P 2100 7300
F 0 "#PWR0105" H 2100 7050 50  0001 C CNN
F 1 "GND" H 2105 7127 50  0000 C CNN
F 2 "" H 2100 7300 50  0001 C CNN
F 3 "" H 2100 7300 50  0001 C CNN
	1    2100 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 7000 2100 7300
Wire Wire Line
	1400 5750 1400 5700
Wire Wire Line
	1800 5750 1800 5700
Wire Wire Line
	1950 5750 1950 5700
$Comp
L power:+3.3V #PWR0106
U 1 1 5C47C5D1
P 2600 4650
F 0 "#PWR0106" H 2600 4500 50  0001 C CNN
F 1 "+3.3V" H 2615 4823 50  0000 C CNN
F 2 "" H 2600 4650 50  0001 C CNN
F 3 "" H 2600 4650 50  0001 C CNN
	1    2600 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5C47C62D
P 3000 4950
F 0 "C4" V 2950 4900 50  0000 R CNN
F 1 "0.1u" V 2950 5000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3000 4950 50  0001 C CNN
F 3 "~" H 3000 4950 50  0001 C CNN
	1    3000 4950
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C47C7C1
P 3000 4800
F 0 "C3" V 2950 4750 50  0000 R CNN
F 1 "4.7u" V 2950 4850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3000 4800 50  0001 C CNN
F 3 "~" H 3000 4800 50  0001 C CNN
	1    3000 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 4650 2600 4800
Connection ~ 2600 4800
Wire Wire Line
	2600 4800 2600 4950
Wire Wire Line
	2600 4950 2900 4950
$Comp
L Device:C_Small C5
U 1 1 5C47D057
P 3000 5100
F 0 "C5" V 2950 5050 50  0000 R CNN
F 1 "0.01u" V 2950 5150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3000 5100 50  0001 C CNN
F 3 "~" H 3000 5100 50  0001 C CNN
	1    3000 5100
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5C47DC17
P 3000 5250
F 0 "C6" V 2950 5200 50  0000 R CNN
F 1 "0.1u" V 2950 5300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3000 5250 50  0001 C CNN
F 3 "~" H 3000 5250 50  0001 C CNN
	1    3000 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 5100 2700 5100
Wire Wire Line
	2700 5100 2700 5300
Wire Wire Line
	2900 5250 2800 5250
Wire Wire Line
	2800 5250 2800 5300
Wire Wire Line
	2600 4800 2900 4800
Wire Wire Line
	2600 4950 2600 5100
Connection ~ 2600 4950
Wire Wire Line
	2700 5100 2600 5100
Connection ~ 2700 5100
Connection ~ 2600 5100
Wire Wire Line
	2600 5100 2600 5250
Wire Wire Line
	2800 5250 2600 5250
Connection ~ 2800 5250
Connection ~ 2600 5250
Wire Wire Line
	2600 5250 2600 5300
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5C47FE4C
P 550 1250
F 0 "J1" H 656 1428 50  0000 C CNN
F 1 "PWR" H 656 1337 50  0000 C CNN
F 2 "KUT_Connector:DF3A-2P-2DSA" H 550 1250 50  0001 C CNN
F 3 "~" H 550 1250 50  0001 C CNN
	1    550  1250
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR0107
U 1 1 5C47FF22
P 850 1150
F 0 "#PWR0107" H 850 1000 50  0001 C CNN
F 1 "+6V" H 865 1323 50  0000 C CNN
F 2 "" H 850 1150 50  0001 C CNN
F 3 "" H 850 1150 50  0001 C CNN
	1    850  1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1150 850  1250
Wire Wire Line
	850  1250 750  1250
$Comp
L power:GND #PWR0108
U 1 1 5C480291
P 850 1450
F 0 "#PWR0108" H 850 1200 50  0001 C CNN
F 1 "GND" H 855 1277 50  0000 C CNN
F 2 "" H 850 1450 50  0001 C CNN
F 3 "" H 850 1450 50  0001 C CNN
	1    850  1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1450 850  1350
Wire Wire Line
	850  1350 750  1350
$Comp
L KUT_PowerIC:NJM12888 VR1
U 1 1 5C480A6A
P 2100 2750
F 0 "VR1" H 2100 3137 60  0000 C CNN
F 1 "NJM12888" H 2100 3031 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 2100 2350 60  0001 C CNN
F 3 "http://www.njr.co.jp/products/semicon/PDF/NJM12888_J.pdf" H 2100 2250 60  0001 C CNN
	1    2100 2750
	1    0    0    -1  
$EndComp
NoConn ~ 2500 2850
$Comp
L power:+6V #PWR0109
U 1 1 5C480E33
P 1600 2550
F 0 "#PWR0109" H 1600 2400 50  0001 C CNN
F 1 "+6V" H 1615 2723 50  0000 C CNN
F 2 "" H 1600 2550 50  0001 C CNN
F 3 "" H 1600 2550 50  0001 C CNN
	1    1600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2550 1600 2600
Wire Wire Line
	1600 2650 1700 2650
Wire Wire Line
	1600 2650 1600 2850
Wire Wire Line
	1600 2850 1700 2850
Connection ~ 1600 2650
$Comp
L Device:C_Small C1
U 1 1 5C48179D
P 1600 2950
F 0 "C1" H 1692 2996 50  0000 L CNN
F 1 "0.1u" H 1692 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1600 2950 50  0001 C CNN
F 3 "~" H 1600 2950 50  0001 C CNN
	1    1600 2950
	1    0    0    -1  
$EndComp
Connection ~ 1600 2850
$Comp
L Device:C_Small C2
U 1 1 5C481809
P 2600 2950
F 0 "C2" H 2692 2996 50  0000 L CNN
F 1 "1u" H 2692 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2600 2950 50  0001 C CNN
F 3 "~" H 2600 2950 50  0001 C CNN
	1    2600 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5C4818C2
P 1600 3050
F 0 "#PWR0110" H 1600 2800 50  0001 C CNN
F 1 "GND" H 1605 2877 50  0000 C CNN
F 2 "" H 1600 3050 50  0001 C CNN
F 3 "" H 1600 3050 50  0001 C CNN
	1    1600 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5C4818EA
P 2100 3050
F 0 "#PWR0111" H 2100 2800 50  0001 C CNN
F 1 "GND" H 2105 2877 50  0000 C CNN
F 2 "" H 2100 3050 50  0001 C CNN
F 3 "" H 2100 3050 50  0001 C CNN
	1    2100 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5C48190B
P 2600 3050
F 0 "#PWR0112" H 2600 2800 50  0001 C CNN
F 1 "GND" H 2605 2877 50  0000 C CNN
F 2 "" H 2600 3050 50  0001 C CNN
F 3 "" H 2600 3050 50  0001 C CNN
	1    2600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2650 2600 2650
Wire Wire Line
	2600 2650 2600 2850
Wire Wire Line
	2600 2550 2600 2650
Connection ~ 2600 2650
$Comp
L power:+3.3V #PWR0113
U 1 1 5C4824D6
P 2600 2550
F 0 "#PWR0113" H 2600 2400 50  0001 C CNN
F 1 "+3.3V" H 2615 2723 50  0000 C CNN
F 2 "" H 2600 2550 50  0001 C CNN
F 3 "" H 2600 2550 50  0001 C CNN
	1    2600 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male J2
U 1 1 5C482668
P 2600 1300
F 0 "J2" H 2706 1678 50  0000 C CNN
F 1 "Port" H 2706 1587 50  0000 C CNN
F 2 "KUT_Connector:DF3A-5P-2DSA" H 2600 1300 50  0001 C CNN
F 3 "~" H 2600 1300 50  0001 C CNN
	1    2600 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR0114
U 1 1 5C4826D2
P 2900 1000
F 0 "#PWR0114" H 2900 850 50  0001 C CNN
F 1 "+6V" H 2915 1173 50  0000 C CNN
F 2 "" H 2900 1000 50  0001 C CNN
F 3 "" H 2900 1000 50  0001 C CNN
	1    2900 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1000 2900 1100
$Comp
L power:GND #PWR0115
U 1 1 5C48344F
P 2900 1600
F 0 "#PWR0115" H 2900 1350 50  0001 C CNN
F 1 "GND" H 2905 1427 50  0000 C CNN
F 2 "" H 2900 1600 50  0001 C CNN
F 3 "" H 2900 1600 50  0001 C CNN
	1    2900 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1200 2900 1200
Wire Wire Line
	2900 1200 2900 1600
Wire Wire Line
	2800 1300 2950 1300
Wire Wire Line
	2800 1500 2950 1500
Text Notes 2600 1400 2    50   ~ 0
TRIG2
Text Notes 2600 1300 2    50   ~ 0
TRIG
Text Notes 2600 1500 2    50   ~ 0
FPIN
Text GLabel 2950 1500 2    50   Input ~ 0
FPIN
Text GLabel 2950 1300 2    50   Input ~ 0
TRIG
Text Notes 2600 1200 2    50   ~ 0
GND
Text Notes 2600 1100 2    50   ~ 0
VCC
Text GLabel 2200 6500 0    50   Input ~ 0
TRIG
Text GLabel 2200 6700 0    50   Input ~ 0
FPIN
Text GLabel 3200 6800 2    50   Input ~ 0
SWDIO
Text GLabel 3200 6900 2    50   Input ~ 0
SWCLK
Text GLabel 3200 6400 2    50   Input ~ 0
Debug_TX
Text GLabel 3200 6500 2    50   Input ~ 0
Debug_RX
$Comp
L Connector:Conn_01x03_Male J3
U 1 1 5C48743D
P 1200 1300
F 0 "J3" H 1306 1578 50  0000 C CNN
F 1 "Actuator_Port" H 1306 1487 50  0000 C CNN
F 2 "KUT_Connector:DF3A-3P-2DSA" H 1200 1300 50  0001 C CNN
F 3 "~" H 1200 1300 50  0001 C CNN
	1    1200 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0116
U 1 1 5C48751A
P 1600 1100
F 0 "#PWR0116" H 1600 950 50  0001 C CNN
F 1 "VCC" H 1617 1273 50  0000 C CNN
F 2 "" H 1600 1100 50  0001 C CNN
F 3 "" H 1600 1100 50  0001 C CNN
	1    1600 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1100 1600 1200
$Comp
L power:+12V #PWR0117
U 1 1 5C4886D8
P 1800 1100
F 0 "#PWR0117" H 1800 950 50  0001 C CNN
F 1 "+12V" H 1815 1273 50  0000 C CNN
F 2 "" H 1800 1100 50  0001 C CNN
F 3 "" H 1800 1100 50  0001 C CNN
	1    1800 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1100 1800 1300
$Comp
L power:GNDPWR #PWR0118
U 1 1 5C48912B
P 1800 1500
F 0 "#PWR0118" H 1800 1300 50  0001 C CNN
F 1 "GNDPWR" H 1804 1346 50  0000 C CNN
F 2 "" H 1800 1450 50  0001 C CNN
F 3 "" H 1800 1450 50  0001 C CNN
	1    1800 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1400 1400 1400
Wire Wire Line
	1800 1400 1800 1450
Text GLabel 3200 6000 2    50   Input ~ 0
SIG1
Text GLabel 4100 3100 0    50   Input ~ 0
SIG1
$Comp
L Device:R_Small R8
U 1 1 5C48BC97
P 4350 3350
F 0 "R8" H 4409 3396 50  0000 L CNN
F 1 "1k" H 4409 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4350 3350 50  0001 C CNN
F 3 "~" H 4350 3350 50  0001 C CNN
	1    4350 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5C48D3DA
P 4150 3350
F 0 "R4" H 4209 3396 50  0000 L CNN
F 1 "1k" H 4209 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4150 3350 50  0001 C CNN
F 3 "~" H 4150 3350 50  0001 C CNN
	1    4150 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D4
U 1 1 5C48ED16
P 4150 3550
F 0 "D4" V 4196 3482 50  0000 R CNN
F 1 "RED" V 4105 3482 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 4150 3550 50  0001 C CNN
F 3 "~" V 4150 3550 50  0001 C CNN
	1    4150 3550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5C48EDF4
P 4150 3650
F 0 "#PWR0119" H 4150 3400 50  0001 C CNN
F 1 "GND" H 4155 3477 50  0000 C CNN
F 2 "" H 4150 3650 50  0001 C CNN
F 3 "" H 4150 3650 50  0001 C CNN
	1    4150 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5C48EE28
P 4350 3650
F 0 "#PWR0120" H 4350 3400 50  0001 C CNN
F 1 "GND" H 4355 3477 50  0000 C CNN
F 2 "" H 4350 3650 50  0001 C CNN
F 3 "" H 4350 3650 50  0001 C CNN
	1    4350 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3650 4350 3450
$Comp
L power:GNDPWR #PWR0122
U 1 1 5C49588B
P 7300 4650
F 0 "#PWR0122" H 7300 4450 50  0001 C CNN
F 1 "GNDPWR" H 7304 4496 50  0000 C CNN
F 2 "" H 7300 4600 50  0001 C CNN
F 3 "" H 7300 4600 50  0001 C CNN
	1    7300 4650
	1    0    0    -1  
$EndComp
Text GLabel 3200 6100 2    50   Input ~ 0
SIG2
Text GLabel 7700 3600 2    50   Input ~ 0
Servo1
$Comp
L power:+3.3V #PWR0125
U 1 1 5C4B7F52
P 5850 2900
F 0 "#PWR0125" H 5850 2750 50  0001 C CNN
F 1 "+3.3V" H 5865 3073 50  0000 C CNN
F 2 "" H 5850 2900 50  0001 C CNN
F 3 "" H 5850 2900 50  0001 C CNN
	1    5850 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5C4B7F58
P 5850 3100
F 0 "R12" H 5909 3146 50  0000 L CNN
F 1 "1k" H 5909 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5850 3100 50  0001 C CNN
F 3 "~" H 5850 3100 50  0001 C CNN
	1    5850 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2900 5850 3000
$Comp
L power:+3.3V #PWR0126
U 1 1 5C4B7F60
P 6050 2900
F 0 "#PWR0126" H 6050 2750 50  0001 C CNN
F 1 "+3.3V" H 6065 3073 50  0000 C CNN
F 2 "" H 6050 2900 50  0001 C CNN
F 3 "" H 6050 2900 50  0001 C CNN
	1    6050 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5C4B7F66
P 6050 3100
F 0 "R13" H 6109 3146 50  0000 L CNN
F 1 "1k" H 6109 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6050 3100 50  0001 C CNN
F 3 "~" H 6050 3100 50  0001 C CNN
	1    6050 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2900 6050 3000
$Comp
L KUT_Device:Q_NMOS_DUAL_COMMON_S Q1
U 1 1 5C4DA449
P 5450 3500
F 0 "Q1" V 5100 3350 50  0000 C CNN
F 1 "DUAL" V 5700 3400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5500 3600 50  0001 C CNN
F 3 "" H 5250 3500 50  0001 C CNN
	1    5450 3500
	0    1    1    0   
$EndComp
Text GLabel 4100 3200 0    50   Input ~ 0
SIG2
$Comp
L Device:R_Small R9
U 1 1 5C4F353A
P 4750 3350
F 0 "R9" H 4809 3396 50  0000 L CNN
F 1 "1k" H 4809 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4750 3350 50  0001 C CNN
F 3 "~" H 4750 3350 50  0001 C CNN
	1    4750 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5C4F3541
P 4550 3350
F 0 "R5" H 4609 3396 50  0000 L CNN
F 1 "1k" H 4609 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4550 3350 50  0001 C CNN
F 3 "~" H 4550 3350 50  0001 C CNN
	1    4550 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 5C4F3548
P 4550 3550
F 0 "D5" V 4596 3482 50  0000 R CNN
F 1 "RED" V 4505 3482 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 4550 3550 50  0001 C CNN
F 3 "~" V 4550 3550 50  0001 C CNN
	1    4550 3550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5C4F354F
P 4550 3650
F 0 "#PWR0127" H 4550 3400 50  0001 C CNN
F 1 "GND" H 4555 3477 50  0000 C CNN
F 2 "" H 4550 3650 50  0001 C CNN
F 3 "" H 4550 3650 50  0001 C CNN
	1    4550 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5C4F3555
P 4750 3650
F 0 "#PWR0128" H 4750 3400 50  0001 C CNN
F 1 "GND" H 4755 3477 50  0000 C CNN
F 2 "" H 4750 3650 50  0001 C CNN
F 3 "" H 4750 3650 50  0001 C CNN
	1    4750 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3650 4750 3450
Wire Wire Line
	4100 3100 4150 3100
Wire Wire Line
	4150 3100 4150 3250
Wire Wire Line
	4150 3100 4350 3100
Wire Wire Line
	4350 3100 4350 3250
Connection ~ 4150 3100
Wire Wire Line
	4350 3100 5500 3100
Connection ~ 4350 3100
Wire Wire Line
	4100 3200 4550 3200
Wire Wire Line
	4550 3200 4550 3250
Wire Wire Line
	4550 3200 4750 3200
Wire Wire Line
	4750 3200 4750 3250
Connection ~ 4550 3200
Wire Wire Line
	4750 3200 5000 3200
Wire Wire Line
	5000 3200 5000 3900
Wire Wire Line
	5000 3900 5500 3900
Connection ~ 4750 3200
$Comp
L power:GND #PWR0133
U 1 1 5C50135F
P 5150 3650
F 0 "#PWR0133" H 5150 3400 50  0001 C CNN
F 1 "GND" H 5155 3477 50  0000 C CNN
F 2 "" H 5150 3650 50  0001 C CNN
F 3 "" H 5150 3650 50  0001 C CNN
	1    5150 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3650 5150 3500
$Comp
L power:VCC #PWR0135
U 1 1 5C58236B
P 7200 2800
F 0 "#PWR0135" H 7200 2650 50  0001 C CNN
F 1 "VCC" H 7217 2973 50  0000 C CNN
F 2 "" H 7200 2800 50  0001 C CNN
F 3 "" H 7200 2800 50  0001 C CNN
	1    7200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4600 7300 4600
Wire Wire Line
	7300 4600 7300 4650
Wire Wire Line
	7300 4600 7400 4600
Connection ~ 7300 4600
Wire Wire Line
	7100 3400 7200 3400
$Comp
L Device:R_Small R16
U 1 1 5C61B6BC
P 7200 4350
F 0 "R16" H 7259 4396 50  0000 L CNN
F 1 "1k" H 7259 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7200 4350 50  0001 C CNN
F 3 "~" H 7200 4350 50  0001 C CNN
	1    7200 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R17
U 1 1 5C61B6C3
P 7400 4350
F 0 "R17" H 7459 4396 50  0000 L CNN
F 1 "1k" H 7459 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7400 4350 50  0001 C CNN
F 3 "~" H 7400 4350 50  0001 C CNN
	1    7400 4350
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0138
U 1 1 5C6E38CD
P 7450 2800
F 0 "#PWR0138" H 7450 2650 50  0001 C CNN
F 1 "+12V" H 7465 2973 50  0000 C CNN
F 2 "" H 7450 2800 50  0001 C CNN
F 3 "" H 7450 2800 50  0001 C CNN
	1    7450 2800
	1    0    0    -1  
$EndComp
Text GLabel 8200 3600 2    50   Input ~ 0
AirVCC1
Wire Wire Line
	8150 3500 8150 3600
Wire Wire Line
	8200 3600 8150 3600
$Comp
L power:+12V #PWR0141
U 1 1 5C797939
P 8150 3500
F 0 "#PWR0141" H 8150 3350 50  0001 C CNN
F 1 "+12V" H 8165 3673 50  0000 C CNN
F 2 "" H 8150 3500 50  0001 C CNN
F 3 "" H 8150 3500 50  0001 C CNN
	1    8150 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J4
U 1 1 5C7D3DD3
P 4250 1300
F 0 "J4" H 4356 1578 50  0000 C CNN
F 1 "Servo1" H 4356 1487 50  0000 C CNN
F 2 "KUT_Connector:DF3A-3P-2DSA" H 4250 1300 50  0001 C CNN
F 3 "~" H 4250 1300 50  0001 C CNN
	1    4250 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0143
U 1 1 5C7D3F8C
P 4500 1100
F 0 "#PWR0143" H 4500 950 50  0001 C CNN
F 1 "VCC" H 4517 1273 50  0000 C CNN
F 2 "" H 4500 1100 50  0001 C CNN
F 3 "" H 4500 1100 50  0001 C CNN
	1    4500 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1100 4500 1300
Wire Wire Line
	4500 1300 4450 1300
Wire Wire Line
	4550 1500 4550 1200
Wire Wire Line
	4550 1200 4450 1200
Wire Wire Line
	4450 1400 4600 1400
Text GLabel 4600 1400 2    50   Input ~ 0
Servo1
$Comp
L Connector:Conn_01x02_Male J5
U 1 1 5C80B6E4
P 5200 1250
F 0 "J5" H 5306 1428 50  0000 C CNN
F 1 "Air1" H 5306 1337 50  0000 C CNN
F 2 "KUT_Connector:DF3A-2P-2DSA" H 5200 1250 50  0001 C CNN
F 3 "~" H 5200 1250 50  0001 C CNN
	1    5200 1250
	1    0    0    -1  
$EndComp
Text GLabel 8200 3800 2    50   Input ~ 0
AirGND1
Text GLabel 5400 1250 2    50   Input ~ 0
AirVCC1
Text GLabel 5400 1350 2    50   Input ~ 0
AirGND1
$Comp
L Device:R_Small R22
U 1 1 5C831ABF
P 8150 3700
F 0 "R22" H 8209 3746 50  0000 L CNN
F 1 "1k" H 8209 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8150 3700 50  0001 C CNN
F 3 "~" H 8150 3700 50  0001 C CNN
	1    8150 3700
	-1   0    0    1   
$EndComp
Connection ~ 8150 3600
Wire Wire Line
	8150 3800 8200 3800
Wire Notes Line
	550  2000 7150 2000
Wire Notes Line
	7150 2000 7150 1850
Wire Notes Line
	550  3550 3700 3550
Wire Notes Line
	3700 550  3700 7050
NoConn ~ 2200 6000
NoConn ~ 2200 6100
NoConn ~ 3200 5800
NoConn ~ 3200 5900
NoConn ~ 3200 6200
NoConn ~ 3200 6700
NoConn ~ 3200 7000
$Comp
L Device:R_Small R1
U 1 1 5C8BDC17
P 2950 2700
F 0 "R1" H 3009 2746 50  0000 L CNN
F 1 "1k" H 3009 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2950 2700 50  0001 C CNN
F 3 "~" H 2950 2700 50  0001 C CNN
	1    2950 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 5C8BDCDA
P 2950 2900
F 0 "D1" V 2996 2832 50  0000 R CNN
F 1 "LED" V 2905 2832 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 2950 2900 50  0001 C CNN
F 3 "~" V 2950 2900 50  0001 C CNN
	1    2950 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0147
U 1 1 5C8BDDFD
P 2950 3050
F 0 "#PWR0147" H 2950 2800 50  0001 C CNN
F 1 "GND" H 2955 2877 50  0000 C CNN
F 2 "" H 2950 3050 50  0001 C CNN
F 3 "" H 2950 3050 50  0001 C CNN
	1    2950 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0148
U 1 1 5C8BDE68
P 2950 2550
F 0 "#PWR0148" H 2950 2400 50  0001 C CNN
F 1 "+3.3V" H 2965 2723 50  0000 C CNN
F 2 "" H 2950 2550 50  0001 C CNN
F 3 "" H 2950 2550 50  0001 C CNN
	1    2950 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2550 2950 2600
Wire Wire Line
	2950 3000 2950 3050
Wire Wire Line
	2900 1100 2800 1100
$Comp
L power:GND #PWR0149
U 1 1 5C8EC72C
P 3300 5250
F 0 "#PWR0149" H 3300 5000 50  0001 C CNN
F 1 "GND" H 3305 5077 50  0000 C CNN
F 2 "" H 3300 5250 50  0001 C CNN
F 3 "" H 3300 5250 50  0001 C CNN
	1    3300 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 5100 3300 5100
Wire Wire Line
	3300 5100 3300 5250
Wire Wire Line
	3300 5100 3300 4950
Wire Wire Line
	3300 4950 3100 4950
Connection ~ 3300 5100
Wire Wire Line
	3300 4950 3300 4800
Wire Wire Line
	3300 4800 3100 4800
Connection ~ 3300 4950
$Comp
L Device:R_Small R2
U 1 1 5C918C40
P 1250 6850
F 0 "R2" H 1309 6896 50  0000 L CNN
F 1 "1k" H 1309 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1250 6850 50  0001 C CNN
F 3 "~" H 1250 6850 50  0001 C CNN
	1    1250 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D2
U 1 1 5C918C47
P 1250 7050
F 0 "D2" V 1296 6982 50  0000 R CNN
F 1 "LED" V 1205 6982 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 1250 7050 50  0001 C CNN
F 3 "~" V 1250 7050 50  0001 C CNN
	1    1250 7050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0150
U 1 1 5C918C4E
P 1250 7200
F 0 "#PWR0150" H 1250 6950 50  0001 C CNN
F 1 "GND" H 1255 7027 50  0000 C CNN
F 2 "" H 1250 7200 50  0001 C CNN
F 3 "" H 1250 7200 50  0001 C CNN
	1    1250 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 7150 1250 7200
$Comp
L Device:R_Small R3
U 1 1 5C9205A9
P 1550 6850
F 0 "R3" H 1609 6896 50  0000 L CNN
F 1 "1k" H 1609 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1550 6850 50  0001 C CNN
F 3 "~" H 1550 6850 50  0001 C CNN
	1    1550 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 5C9205B0
P 1550 7050
F 0 "D3" V 1596 6982 50  0000 R CNN
F 1 "LED" V 1505 6982 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 1550 7050 50  0001 C CNN
F 3 "~" V 1550 7050 50  0001 C CNN
	1    1550 7050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0151
U 1 1 5C9205B7
P 1550 7200
F 0 "#PWR0151" H 1550 6950 50  0001 C CNN
F 1 "GND" H 1555 7027 50  0000 C CNN
F 2 "" H 1550 7200 50  0001 C CNN
F 3 "" H 1550 7200 50  0001 C CNN
	1    1550 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 7150 1550 7200
Text Label 1250 6550 0    50   ~ 0
LED1
Text Label 1550 6550 0    50   ~ 0
LED2
Wire Wire Line
	1550 6550 1550 6750
Wire Wire Line
	1250 6550 1250 6750
$Comp
L Connector:Conn_01x06_Male J8
U 1 1 5C945477
P 1000 4600
F 0 "J8" H 1106 4978 50  0000 C CNN
F 1 "STLink+" H 1106 4887 50  0000 C CNN
F 2 "KUT_Connector:DF11-6DP-2DSA2" H 1000 4600 50  0001 C CNN
F 3 "~" H 1000 4600 50  0001 C CNN
	1    1000 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0152
U 1 1 5C945746
P 1300 5000
F 0 "#PWR0152" H 1300 4750 50  0001 C CNN
F 1 "GND" H 1305 4827 50  0000 C CNN
F 2 "" H 1300 5000 50  0001 C CNN
F 3 "" H 1300 5000 50  0001 C CNN
	1    1300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5000 1300 4400
Wire Wire Line
	1300 4400 1200 4400
Text Label 1350 4500 0    50   ~ 0
RESET
Text GLabel 1350 4600 2    50   Input ~ 0
SWDIO
Text GLabel 1350 4700 2    50   Input ~ 0
SWCLK
Text GLabel 1350 4800 2    50   Input ~ 0
Debug_TX
Text GLabel 1350 4900 2    50   Input ~ 0
Debug_RX
Wire Wire Line
	1350 4600 1200 4600
Wire Wire Line
	1200 4500 1350 4500
Wire Wire Line
	1350 4700 1200 4700
Wire Wire Line
	1350 4800 1200 4800
Wire Wire Line
	1350 4900 1200 4900
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5C9794DA
P 1600 2600
F 0 "#FLG0101" H 1600 2675 50  0001 C CNN
F 1 "PWR_FLAG" V 1600 2728 50  0000 L CNN
F 2 "" H 1600 2600 50  0001 C CNN
F 3 "~" H 1600 2600 50  0001 C CNN
	1    1600 2600
	0    -1   -1   0   
$EndComp
Connection ~ 1600 2600
Wire Wire Line
	1600 2600 1600 2650
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5C979808
P 1600 3050
F 0 "#FLG0102" H 1600 3125 50  0001 C CNN
F 1 "PWR_FLAG" V 1600 3178 50  0000 L CNN
F 2 "" H 1600 3050 50  0001 C CNN
F 3 "~" H 1600 3050 50  0001 C CNN
	1    1600 3050
	0    -1   -1   0   
$EndComp
Connection ~ 1600 3050
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5C979BA7
P 2950 2550
F 0 "#FLG0103" H 2950 2625 50  0001 C CNN
F 1 "PWR_FLAG" V 2950 2678 50  0000 L CNN
F 2 "" H 2950 2550 50  0001 C CNN
F 3 "~" H 2950 2550 50  0001 C CNN
	1    2950 2550
	0    1    1    0   
$EndComp
Connection ~ 2950 2550
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5C97A054
P 1800 1100
F 0 "#FLG0104" H 1800 1175 50  0001 C CNN
F 1 "PWR_FLAG" V 1800 1228 50  0000 L CNN
F 2 "" H 1800 1100 50  0001 C CNN
F 3 "~" H 1800 1100 50  0001 C CNN
	1    1800 1100
	0    1    1    0   
$EndComp
Connection ~ 1800 1100
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 5C97A177
P 1800 1450
F 0 "#FLG0105" H 1800 1525 50  0001 C CNN
F 1 "PWR_FLAG" V 1800 1578 50  0000 L CNN
F 2 "" H 1800 1450 50  0001 C CNN
F 3 "~" H 1800 1450 50  0001 C CNN
	1    1800 1450
	0    1    1    0   
$EndComp
Connection ~ 1800 1450
Wire Wire Line
	1800 1450 1800 1500
$Comp
L power:PWR_FLAG #FLG0106
U 1 1 5C97A1F4
P 4500 1100
F 0 "#FLG0106" H 4500 1175 50  0001 C CNN
F 1 "PWR_FLAG" V 4500 1228 50  0000 L CNN
F 2 "" H 4500 1100 50  0001 C CNN
F 3 "~" H 4500 1100 50  0001 C CNN
	1    4500 1100
	0    1    1    0   
$EndComp
Connection ~ 4500 1100
Text Label 3200 5700 0    50   ~ 0
LED1
Text Label 3200 5600 0    50   ~ 0
LED2
Wire Wire Line
	1600 1200 1400 1200
Wire Wire Line
	1400 1300 1800 1300
$Comp
L Device:R_Small R24
U 1 1 5C14B617
P 3450 1200
F 0 "R24" H 3509 1246 50  0000 L CNN
F 1 "1k" H 3509 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3450 1200 50  0001 C CNN
F 3 "~" H 3450 1200 50  0001 C CNN
	1    3450 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D8
U 1 1 5C14B61E
P 3450 1400
F 0 "D8" V 3496 1332 50  0000 R CNN
F 1 "LED" V 3405 1332 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 3450 1400 50  0001 C CNN
F 3 "~" V 3450 1400 50  0001 C CNN
	1    3450 1400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0153
U 1 1 5C14B625
P 3450 1550
F 0 "#PWR0153" H 3450 1300 50  0001 C CNN
F 1 "GND" H 3455 1377 50  0000 C CNN
F 2 "" H 3450 1550 50  0001 C CNN
F 3 "" H 3450 1550 50  0001 C CNN
	1    3450 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1500 3450 1550
Text GLabel 3350 1000 0    50   Input ~ 0
FPIN
Wire Wire Line
	3350 1000 3450 1000
Wire Wire Line
	3450 1000 3450 1100
Wire Wire Line
	5750 3400 5800 3400
Wire Wire Line
	7200 2800 7200 3400
Wire Wire Line
	7200 4450 7200 4600
Wire Wire Line
	7400 4450 7400 4600
Text Notes 7150 2600 0    50   ~ 0
6V
Text GLabel 2950 1400 2    50   Input ~ 0
TRIG2
Wire Wire Line
	2950 1400 2800 1400
Text GLabel 2200 6600 0    50   Input ~ 0
TRIG2
$Comp
L KUT_DriveIC:TLP621-2 DCN1
U 1 1 5C54404F
P 6800 3700
F 0 "DCN1" H 6800 4393 60  0000 C CNN
F 1 "ACPL-227" H 6800 4287 60  0000 C CNN
F 2 "Package_SO:SO-8_5.3x6.2mm_P1.27mm" H 6800 4181 60  0000 C CNN
F 3 "" H 6850 3650 60  0000 C CNN
	1    6800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3200 5850 3400
Wire Wire Line
	5850 3400 6500 3400
Wire Wire Line
	5800 3400 5800 3600
Wire Wire Line
	5800 3600 6500 3600
Wire Wire Line
	6050 3800 6500 3800
Wire Wire Line
	6050 3200 6050 3800
Wire Wire Line
	5750 4000 6500 4000
Wire Wire Line
	5750 3600 5750 4000
Wire Wire Line
	7450 2800 7450 3800
Wire Wire Line
	7450 3800 7100 3800
Wire Wire Line
	7100 3600 7200 3600
Wire Wire Line
	7200 3600 7200 4250
Connection ~ 7200 3600
Wire Wire Line
	7200 3600 7700 3600
Wire Wire Line
	7100 4000 7400 4000
Wire Wire Line
	7400 4000 7400 4250
$Comp
L Device:Q_NMOS_GSD Q2
U 1 1 5C5EEBF4
P 8050 4000
F 0 "Q2" H 8255 4046 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 8255 3955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8250 4100 50  0001 C CNN
F 3 "~" H 8050 4000 50  0001 C CNN
	1    8050 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4000 7850 4000
Connection ~ 7400 4000
$Comp
L power:GNDPWR #PWR0121
U 1 1 5C5FB808
P 8150 4650
F 0 "#PWR0121" H 8150 4450 50  0001 C CNN
F 1 "GNDPWR" H 8154 4496 50  0000 C CNN
F 2 "" H 8150 4600 50  0001 C CNN
F 3 "" H 8150 4600 50  0001 C CNN
	1    8150 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4650 8150 4200
Connection ~ 8150 3800
Connection ~ 3300 5250
Wire Wire Line
	3100 5250 3300 5250
Text Notes 2100 7000 2    50   ~ 0
BOOT0
NoConn ~ 2200 6300
NoConn ~ 2200 6400
NoConn ~ 2200 6800
NoConn ~ 2200 6900
NoConn ~ 3200 6300
NoConn ~ 3200 5500
$Comp
L power:GNDPWR #PWR0123
U 1 1 5C64FCD6
P 4550 1500
F 0 "#PWR0123" H 4550 1300 50  0001 C CNN
F 1 "GNDPWR" H 4554 1346 50  0000 C CNN
F 2 "" H 4550 1450 50  0001 C CNN
F 3 "" H 4550 1450 50  0001 C CNN
	1    4550 1500
	1    0    0    -1  
$EndComp
NoConn ~ 3200 6600
$EndSCHEMATC
