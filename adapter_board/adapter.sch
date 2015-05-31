EESchema Schematic File Version 2
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
LIBS:special
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
LIBS:logo
LIBS:adapter-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
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
L CONN_01X12 P101
U 1 1 5516CB27
P 2700 4000
F 0 "P101" H 2700 4650 50  0000 C CNN
F 1 "CONN_01X12" V 2800 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x12" H 2700 4000 60  0001 C CNN
F 3 "" H 2700 4000 60  0000 C CNN
	1    2700 4000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X12 P103
U 1 1 5516CB8E
P 3600 4000
F 0 "P103" H 3600 4650 50  0000 C CNN
F 1 "CONN_01X12" V 3700 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x12" H 3600 4000 60  0001 C CNN
F 3 "" H 3600 4000 60  0000 C CNN
	1    3600 4000
	-1   0    0    1   
$EndComp
Text Notes 3200 4250 1    60   ~ 0
Pro Mini 3V3
$Comp
L CONN_01X06 P102
U 1 1 5516CBE2
P 3150 3300
F 0 "P102" H 3150 3650 50  0000 C CNN
F 1 "CONN_01X06" V 3250 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 3150 3300 60  0001 C CNN
F 3 "" H 3150 3300 60  0000 C CNN
	1    3150 3300
	0    1    1    0   
$EndComp
Text Label 3300 2850 1    60   ~ 0
GND
Text Label 3200 2850 1    60   ~ 0
Vcc
Text Label 3100 2850 1    60   ~ 0
RXI
Text Label 3000 2850 1    60   ~ 0
TXO
Text Label 2250 3750 0    60   ~ 0
GND
Text Label 4050 3550 2    60   ~ 0
GND
Text Label 2250 3450 0    60   ~ 0
TXO
Text Label 2250 3550 0    60   ~ 0
RXI
Text Label 2250 3850 0    60   ~ 0
RS
Text Label 2250 3950 0    60   ~ 0
HTR0
Text Label 2250 4150 0    60   ~ 0
FAN0
Text Label 2250 4250 0    60   ~ 0
HTR2
Text Label 2250 4550 0    60   ~ 0
HTR1
Text Label 4050 4250 2    60   ~ 0
SCLK
Text Label 4050 4350 2    60   ~ 0
MISO
Text Label 4050 4450 2    60   ~ 0
MOSI
Text Label 4050 4050 2    60   ~ 0
SEL0
Text Label 4050 3950 2    60   ~ 0
SEL1
Text Label 4050 3850 2    60   ~ 0
SEL2
Text Label 2900 2850 1    60   ~ 0
DTR
$Comp
L CONN_01X02 P108
U 1 1 5516CFEE
P 1950 1750
F 0 "P108" H 1950 1900 50  0000 C CNN
F 1 "CONN_01X02" V 2100 1750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 1950 1750 60  0001 C CNN
F 3 "" H 1950 1750 60  0000 C CNN
	1    1950 1750
	-1   0    0    1   
$EndComp
Text Label 2350 1700 2    60   ~ 0
12V
Text Label 2350 1800 2    60   ~ 0
GND
Text Label 3650 3300 0    60   ~ 0
5V
$Comp
L CONN_01X06 P110
U 1 1 5516D2AE
P 5300 3050
F 0 "P110" H 5300 3400 50  0000 C CNN
F 1 "CONN_01X06" V 5400 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 5300 3050 60  0001 C CNN
F 3 "" H 5300 3050 60  0000 C CNN
	1    5300 3050
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P113
U 1 1 5516D31F
P 6300 3050
F 0 "P113" H 6300 3400 50  0000 C CNN
F 1 "CONN_01X06" V 6400 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 6300 3050 60  0001 C CNN
F 3 "" H 6300 3050 60  0000 C CNN
	1    6300 3050
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P105
U 1 1 5516D3D5
P 3600 5650
F 0 "P105" H 3600 5800 50  0000 C CNN
F 1 "CONN_01X02" V 3750 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3600 5650 60  0001 C CNN
F 3 "" H 3600 5650 60  0000 C CNN
	1    3600 5650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P106
U 1 1 5516D458
P 3600 6350
F 0 "P106" H 3600 6500 50  0000 C CNN
F 1 "CONN_01X02" V 3750 6350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3600 6350 60  0001 C CNN
F 3 "" H 3600 6350 60  0000 C CNN
	1    3600 6350
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P107
U 1 1 5516D47F
P 3600 7150
F 0 "P107" H 3600 7300 50  0000 C CNN
F 1 "CONN_01X02" V 3750 7150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3600 7150 60  0001 C CNN
F 3 "" H 3600 7150 60  0000 C CNN
	1    3600 7150
	1    0    0    -1  
$EndComp
$Sheet
S 1900 5450 800  500 
U 5516D580
F0 "PWM0" 60
F1 "PWM.sch" 60
F2 "GND" I L 1900 5850 60 
F3 "12V" I L 1900 5550 60 
F4 "INV_PWM" I L 1900 5700 60 
F5 "Drain" O R 2700 5700 60 
$EndSheet
$Sheet
S 1900 6150 800  500 
U 5516E183
F0 "PWM1" 60
F1 "PWM.sch" 60
F2 "GND" I L 1900 6550 60 
F3 "12V" I L 1900 6250 60 
F4 "INV_PWM" I L 1900 6400 60 
F5 "Drain" O R 2700 6400 60 
$EndSheet
$Sheet
S 1900 6950 800  500 
U 5516E3F1
F0 "PWM2" 60
F1 "PWM.sch" 60
F2 "GND" I L 1900 7350 60 
F3 "12V" I L 1900 7050 60 
F4 "INV_PWM" I L 1900 7200 60 
F5 "Drain" O R 2700 7200 60 
$EndSheet
Text Label 4550 5750 2    60   ~ 0
HTR0
Text Label 1450 7200 2    60   ~ 0
FAN0
Text Label 1450 5700 2    60   ~ 0
HTR1
Text Label 1750 5100 2    60   ~ 0
12V
Text Label 3200 5100 2    60   ~ 0
12V
Text Notes 4300 5650 2    60   ~ 0
HEATER 0
Text Notes 4200 6400 2    60   ~ 0
FAN 0
Text Notes 4350 7200 2    60   ~ 0
HEATER 1
Text Notes 5500 2800 2    60   ~ 0
SENSOR1
Text Label 4050 4550 2    60   ~ 0
HTR3
$Comp
L CONN_01X02 P111
U 1 1 5546C0CC
P 6700 5700
F 0 "P111" H 6700 5850 50  0000 C CNN
F 1 "CONN_01X02" V 6850 5700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6700 5700 60  0001 C CNN
F 3 "" H 6700 5700 60  0000 C CNN
	1    6700 5700
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P112
U 1 1 5546C0D2
P 6700 6400
F 0 "P112" H 6700 6550 50  0000 C CNN
F 1 "CONN_01X02" V 6850 6400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6700 6400 60  0001 C CNN
F 3 "" H 6700 6400 60  0000 C CNN
	1    6700 6400
	1    0    0    -1  
$EndComp
$Sheet
S 5000 5500 800  500 
U 5546C0DE
F0 "PWM3" 60
F1 "PWM.sch" 60
F2 "GND" I L 5000 5900 60 
F3 "12V" I L 5000 5600 60 
F4 "INV_PWM" I L 5000 5750 60 
F5 "Drain" O R 5800 5750 60 
$EndSheet
$Sheet
S 5000 6200 800  500 
U 5546C0E4
F0 "PWM4" 60
F1 "PWM.sch" 60
F2 "GND" I L 5000 6600 60 
F3 "12V" I L 5000 6300 60 
F4 "INV_PWM" I L 5000 6450 60 
F5 "Drain" O R 5800 6450 60 
$EndSheet
Text Label 4900 6950 0    60   ~ 0
GND
Text Label 1450 6400 2    60   ~ 0
HTR2
Text Label 4550 6450 2    60   ~ 0
HTR3
Text Notes 7400 5700 2    60   ~ 0
HEATER 2
Text Notes 7400 6450 2    60   ~ 0
HEATER 3
Text Notes 3350 4800 3    60   ~ 0
A4, A5
Text Notes 7850 2850 2    60   ~ 0
Display 0
Text Notes 8100 3300 3    60   ~ 0
D1
Text Notes 8000 3300 3    60   ~ 0
D2
Text Notes 7900 3300 3    60   ~ 0
D3
Text Notes 7800 3300 3    60   ~ 0
D4
Text Notes 7700 3300 3    60   ~ 0
EN
Wire Wire Line
	3300 3100 3300 2850
Wire Wire Line
	3200 3100 3200 2850
Wire Wire Line
	3100 3100 3100 2850
Wire Wire Line
	3000 3100 3000 2850
Wire Wire Line
	2900 3100 2900 2850
Wire Wire Line
	2500 3450 2250 3450
Wire Wire Line
	2500 3550 2250 3550
Wire Wire Line
	2500 3650 2250 3650
Wire Wire Line
	2500 3750 2250 3750
Wire Wire Line
	2500 3850 2250 3850
Wire Wire Line
	2500 3950 2250 3950
Wire Wire Line
	1900 4050 2500 4050
Wire Wire Line
	2500 4150 2250 4150
Wire Wire Line
	2500 4250 2250 4250
Wire Wire Line
	1900 4350 2500 4350
Wire Wire Line
	2050 4450 2500 4450
Wire Wire Line
	2500 4550 2250 4550
Wire Wire Line
	3800 3450 4050 3450
Wire Wire Line
	3800 3550 7200 3550
Wire Wire Line
	3800 3650 4050 3650
Wire Wire Line
	3800 3750 4050 3750
Wire Wire Line
	3800 4550 4050 4550
Wire Wire Line
	3800 4450 4300 4450
Wire Wire Line
	3800 4350 8100 4350
Wire Wire Line
	3800 4250 7900 4250
Wire Wire Line
	3800 4150 7800 4150
Wire Wire Line
	3800 4050 6250 4050
Wire Wire Line
	3800 3950 5250 3950
Wire Wire Line
	2150 1700 3300 1700
Wire Wire Line
	2150 1800 2350 1800
Wire Wire Line
	3400 5700 2700 5700
Wire Wire Line
	3400 5600 3200 5600
Wire Wire Line
	3200 5100 3200 7100
Wire Wire Line
	1900 5550 1750 5550
Wire Wire Line
	1750 5100 1750 7050
Wire Wire Line
	1900 5700 1450 5700
Wire Wire Line
	1900 5850 1800 5850
Wire Wire Line
	3400 6400 2700 6400
Wire Wire Line
	1900 6250 1750 6250
Wire Wire Line
	1900 6400 1450 6400
Wire Wire Line
	1900 6550 1800 6550
Wire Wire Line
	3400 7200 2700 7200
Wire Wire Line
	1750 7050 1900 7050
Wire Wire Line
	1900 7200 1450 7200
Wire Wire Line
	1800 7350 1900 7350
Connection ~ 1750 5550
Connection ~ 1750 6250
Wire Wire Line
	1800 5850 1800 7600
Connection ~ 1800 6550
Connection ~ 1800 7350
Wire Wire Line
	3200 6300 3400 6300
Connection ~ 3200 5600
Wire Wire Line
	3200 7100 3400 7100
Connection ~ 3200 6300
Wire Wire Line
	4300 4450 4300 4300
Wire Wire Line
	4300 4300 8000 4300
Wire Wire Line
	6250 4050 6250 3250
Wire Wire Line
	5250 3950 5250 3250
Wire Wire Line
	5550 3250 5550 4350
Connection ~ 5550 4350
Wire Wire Line
	6550 3250 6550 4350
Connection ~ 6550 4350
Wire Wire Line
	6450 4300 6450 3250
Connection ~ 6450 4300
Wire Wire Line
	6350 3250 6350 4250
Connection ~ 6350 4250
Wire Wire Line
	6150 3250 6150 3550
Connection ~ 6150 3550
Wire Wire Line
	4850 3450 7300 3450
Wire Wire Line
	3650 3300 4850 3300
Wire Wire Line
	4850 3300 4850 3450
Wire Wire Line
	6050 3250 6050 3450
Connection ~ 6050 3450
Wire Wire Line
	5050 3450 5050 3250
Connection ~ 5050 3450
Wire Wire Line
	5150 3250 5150 3550
Connection ~ 5150 3550
Wire Wire Line
	5450 3250 5450 4300
Connection ~ 5450 4300
Wire Wire Line
	5350 3250 5350 4250
Connection ~ 5350 4250
Wire Wire Line
	6500 5750 5800 5750
Wire Wire Line
	6500 5650 6300 5650
Wire Wire Line
	5000 5600 4850 5600
Wire Wire Line
	5000 5750 4550 5750
Wire Wire Line
	5000 5900 4900 5900
Wire Wire Line
	6500 6450 5800 6450
Wire Wire Line
	4850 6300 5000 6300
Wire Wire Line
	5000 6450 4550 6450
Wire Wire Line
	5000 6600 4900 6600
Connection ~ 4850 5600
Connection ~ 4900 6600
Wire Wire Line
	6300 6350 6500 6350
Connection ~ 6300 5650
Wire Wire Line
	4900 5900 4900 6950
Wire Wire Line
	6300 5150 6300 6350
Wire Wire Line
	8100 4350 8100 3250
Wire Wire Line
	8000 4300 8000 3250
Wire Wire Line
	7900 4250 7900 3250
Wire Wire Line
	7800 4150 7800 3250
Wire Wire Line
	3800 3850 7700 3850
Wire Wire Line
	7700 3850 7700 3250
Wire Wire Line
	4850 5150 4850 6300
Wire Wire Line
	7600 3250 7600 3550
Text Label 7500 3550 1    60   ~ 0
RS
$Comp
L CONN_01X10 P115
U 1 1 5548E950
P 7650 3050
F 0 "P115" H 7650 3600 50  0000 C CNN
F 1 "CONN_01X10" V 7750 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10" H 7650 3050 60  0001 C CNN
F 3 "" H 7650 3050 60  0000 C CNN
	1    7650 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7200 3550 7200 3250
Wire Wire Line
	7300 3450 7300 3250
$Comp
L R R102
U 1 1 5548FD1D
P 8400 4000
F 0 "R102" V 8480 4000 50  0000 C CNN
F 1 "R" V 8407 4001 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8330 4000 30  0001 C CNN
F 3 "" H 8400 4000 30  0000 C CNN
	1    8400 4000
	1    0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 554904C6
P 8400 3400
F 0 "R101" V 8480 3400 50  0000 C CNN
F 1 "R" V 8407 3401 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8330 3400 30  0001 C CNN
F 3 "" H 8400 3400 30  0000 C CNN
	1    8400 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3250 7400 3550
Text Label 7400 3550 1    60   ~ 0
Vo
Wire Wire Line
	7500 3250 7500 3550
Text Label 7600 3550 1    60   ~ 0
GND
Text Notes 7400 3650 2    60   ~ 0
Contrast
Text Notes 7600 3750 2    60   ~ 0
R/!W
Wire Notes Line
	7400 3550 7400 3650
Wire Notes Line
	7600 3550 7600 3750
Wire Wire Line
	8400 3650 8400 3750
Wire Wire Line
	8400 4250 8400 4400
Wire Wire Line
	8400 3150 8400 3000
Text Label 8400 3000 1    60   ~ 0
5V
Text Label 8400 4400 1    60   ~ 0
GND
Wire Wire Line
	8400 3700 8650 3700
Connection ~ 8400 3700
Text Label 8650 3700 1    60   ~ 0
Vo
Text Notes 8650 3500 1    60   ~ 0
Contrast
$Comp
L SPST SW101
U 1 1 55493E9C
P 1400 4050
F 0 "SW101" H 1400 4150 50  0000 C CNN
F 1 "SPST" H 1400 3950 50  0000 C CNN
F 2 "SMD_Switches:SW_SPST_PTS645" H 1400 4050 60  0001 C CNN
F 3 "" H 1400 4050 60  0000 C CNN
	1    1400 4050
	1    0    0    -1  
$EndComp
$Comp
L SPST SW102
U 1 1 5549433D
P 1400 4350
F 0 "SW102" H 1400 4450 50  0000 C CNN
F 1 "SPST" H 1400 4250 50  0000 C CNN
F 2 "SMD_Switches:SW_SPST_PTS645" H 1400 4350 60  0001 C CNN
F 3 "" H 1400 4350 60  0000 C CNN
	1    1400 4350
	1    0    0    -1  
$EndComp
$Comp
L SPST SW103
U 1 1 55494368
P 1400 4650
F 0 "SW103" H 1400 4750 50  0000 C CNN
F 1 "SPST" H 1400 4550 50  0000 C CNN
F 2 "SMD_Switches:SW_SPST_PTS645" H 1400 4650 60  0001 C CNN
F 3 "" H 1400 4650 60  0000 C CNN
	1    1400 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4450 2050 4650
Wire Wire Line
	2050 4650 1900 4650
Wire Wire Line
	900  4050 700  4050
Wire Wire Line
	700  4050 700  5250
Wire Wire Line
	900  4350 700  4350
Connection ~ 700  4350
Wire Wire Line
	900  4650 700  4650
Connection ~ 700  4650
Text Label 700  5250 1    60   ~ 0
GND
Text Label 2250 4050 0    60   ~ 0
BT0
Text Label 2250 4350 0    60   ~ 0
BT1
Text Label 2250 4450 0    60   ~ 0
BT2
$Comp
L LD1117S50TR U101
U 1 1 555748CF
P 3700 1750
F 0 "U101" H 3700 2000 40  0000 C CNN
F 1 "LD1117S50TR" H 3700 1950 40  0000 C CNN
F 2 "SMD_Packages:SOT-223" H 3700 1850 40  0000 C CNN
F 3 "" H 3700 1750 60  0000 C CNN
	1    3700 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2000 3700 2350
$Comp
L GND #PWR01
U 1 1 55574B87
P 3700 2350
F 0 "#PWR01" H 3700 2100 60  0001 C CNN
F 1 "GND" H 3700 2200 60  0000 C CNN
F 2 "" H 3700 2350 60  0000 C CNN
F 3 "" H 3700 2350 60  0000 C CNN
	1    3700 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 555750D6
P 4900 6950
F 0 "#PWR02" H 4900 6700 60  0001 C CNN
F 1 "GND" H 4900 6800 60  0000 C CNN
F 2 "" H 4900 6950 60  0000 C CNN
F 3 "" H 4900 6950 60  0000 C CNN
	1    4900 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5557536A
P 1800 7600
F 0 "#PWR03" H 1800 7350 60  0001 C CNN
F 1 "GND" H 1800 7450 60  0000 C CNN
F 2 "" H 1800 7600 60  0000 C CNN
F 3 "" H 1800 7600 60  0000 C CNN
	1    1800 7600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 55575732
P 700 5250
F 0 "#PWR04" H 700 5000 60  0001 C CNN
F 1 "GND" H 700 5100 60  0000 C CNN
F 2 "" H 700 5250 60  0000 C CNN
F 3 "" H 700 5250 60  0000 C CNN
	1    700  5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 55576AA6
P 2350 2350
F 0 "#PWR05" H 2350 2100 60  0001 C CNN
F 1 "GND" H 2350 2200 60  0000 C CNN
F 2 "" H 2350 2350 60  0000 C CNN
F 3 "" H 2350 2350 60  0000 C CNN
	1    2350 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1800 2350 2350
$Comp
L C C101
U 1 1 55576E4C
P 3150 2050
F 0 "C101" H 3200 2150 50  0000 L CNN
F 1 "10u" H 3200 1950 50  0000 L CNN
F 2 "SMD_Capacitors:c_elec_4x5.3" H 3188 1900 30  0001 C CNN
F 3 "" H 3150 2050 60  0000 C CNN
	1    3150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 1700 3150 1850
Connection ~ 3150 1700
Wire Wire Line
	3150 2250 3150 2350
$Comp
L GND #PWR06
U 1 1 555770AA
P 3150 2350
F 0 "#PWR06" H 3150 2100 60  0001 C CNN
F 1 "GND" H 3150 2200 60  0000 C CNN
F 2 "" H 3150 2350 60  0000 C CNN
F 3 "" H 3150 2350 60  0000 C CNN
	1    3150 2350
	1    0    0    -1  
$EndComp
$Comp
L C C102
U 1 1 555773B9
P 4300 2050
F 0 "C102" H 4350 2150 50  0000 L CNN
F 1 "100u" H 4350 1950 50  0000 L CNN
F 2 "SMD_Capacitors:c_elec_4x5.3" H 4338 1900 30  0001 C CNN
F 3 "" H 4300 2050 60  0000 C CNN
	1    4300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1700 4300 1850
Wire Wire Line
	4300 2250 4300 2350
$Comp
L GND #PWR07
U 1 1 555773C1
P 4300 2350
F 0 "#PWR07" H 4300 2100 60  0001 C CNN
F 1 "GND" H 4300 2200 60  0000 C CNN
F 2 "" H 4300 2350 60  0000 C CNN
F 3 "" H 4300 2350 60  0000 C CNN
	1    4300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1700 4700 1700
Connection ~ 4300 1700
Text Label 4700 1700 0    60   ~ 0
5V
Text Notes 4550 2150 0    60   ~ 0
Electrolytic output\ncap for stability
$Comp
L DIODESCH D101
U 1 1 555787ED
P 3700 1250
F 0 "D101" H 3700 1350 50  0000 C CNN
F 1 "DIODESCH" H 3700 1150 50  0000 C CNN
F 2 "smd_diodes:SMA_WAVE" H 3700 1250 60  0001 C CNN
F 3 "" H 3700 1250 60  0000 C CNN
	1    3700 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 1700 4200 1250
Wire Wire Line
	4200 1250 3900 1250
Connection ~ 4200 1700
Wire Wire Line
	3500 1250 3050 1250
Wire Wire Line
	3050 1250 3050 1700
Connection ~ 3050 1700
Wire Wire Line
	4050 3450 4050 3300
Connection ~ 4050 3300
Text Notes 3900 3450 0    60   ~ 0
VIN
Text Label 2250 3650 0    60   ~ 0
RESET
Text Label 4050 3650 2    60   ~ 0
RESET
Text Label 4050 3750 2    60   ~ 0
3V3
Text Notes 6550 2850 2    60   ~ 0
SENSOR0
Text Label 4050 4150 2    60   ~ 0
D4
$Comp
L CONN_01X02 P104
U 1 1 5557C5D2
P 6050 5200
F 0 "P104" H 6050 5350 50  0000 C CNN
F 1 "CONN_01X02" V 6200 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6050 5200 60  0001 C CNN
F 3 "" H 6050 5200 60  0000 C CNN
	1    6050 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6300 5150 6250 5150
Wire Wire Line
	6250 5250 6250 6950
$Comp
L GND #PWR08
U 1 1 5557CDCA
P 6250 6950
F 0 "#PWR08" H 6250 6700 60  0001 C CNN
F 1 "GND" H 6250 6800 60  0000 C CNN
F 2 "" H 6250 6950 60  0000 C CNN
F 3 "" H 6250 6950 60  0000 C CNN
	1    6250 6950
	1    0    0    -1  
$EndComp
Text Label 6300 5150 0    60   ~ 0
AUX_12V
$Comp
L GND #PWR09
U 1 1 5557D530
P 8400 4400
F 0 "#PWR09" H 8400 4150 60  0001 C CNN
F 1 "GND" H 8400 4250 60  0000 C CNN
F 2 "" H 8400 4400 60  0000 C CNN
F 3 "" H 8400 4400 60  0000 C CNN
	1    8400 4400
	1    0    0    -1  
$EndComp
Text Label 4850 5150 2    60   ~ 0
AUX_12V
$Comp
L OPEN_HARDWARE_2 LOGO101
U 1 1 55588699
P 8250 5750
F 0 "LOGO101" H 8250 6250 60  0000 C CNN
F 1 "OPEN_HARDWARE_2" H 8250 5300 60  0000 C CNN
F 2 "Symbols:Symbol_OSHW-Logo_SilkScreen_BIG" H 8250 5750 60  0001 C CNN
F 3 "" H 8250 5750 60  0000 C CNN
	1    8250 5750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
