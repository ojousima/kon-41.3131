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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
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
D2
Text Label 2250 3950 0    60   ~ 0
HTR0
Text Label 2250 4150 0    60   ~ 0
FAN0
Text Label 2250 4250 0    60   ~ 0
D6
Text Label 2250 4550 0    60   ~ 0
HTR1
Text Label 4050 4250 2    60   ~ 0
SCLK
Text Label 4050 4350 2    60   ~ 0
MISO
Text Label 4050 4450 2    60   ~ 0
MOSI
Text Label 4050 4150 2    60   ~ 0
SEL0
Text Label 4050 4050 2    60   ~ 0
SEL1
Text Label 4050 3950 2    60   ~ 0
SEL2
Text Label 2900 2850 1    60   ~ 0
DTR
$Comp
L CONN_01X01 P108
U 1 1 5516CFB3
P 3900 2900
F 0 "P108" H 3900 3000 50  0000 C CNN
F 1 "CONN_01X01" V 4000 2900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 3900 2900 60  0001 C CNN
F 3 "" H 3900 2900 60  0000 C CNN
	1    3900 2900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P107
U 1 1 5516CFEE
P 3900 2150
F 0 "P107" H 3900 2300 50  0000 C CNN
F 1 "CONN_01X02" V 4000 2150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3900 2150 60  0001 C CNN
F 3 "" H 3900 2150 60  0000 C CNN
	1    3900 2150
	1    0    0    -1  
$EndComp
Text Label 3500 2100 0    60   ~ 0
12V
Text Label 3500 2200 0    60   ~ 0
GND
Text Label 3500 2900 0    60   ~ 0
5V
$Comp
L CONN_01X06 P109
U 1 1 5516D2AE
P 6250 3050
F 0 "P109" H 6250 3400 50  0000 C CNN
F 1 "CONN_01X06" V 6350 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 6250 3050 60  0001 C CNN
F 3 "" H 6250 3050 60  0000 C CNN
	1    6250 3050
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P110
U 1 1 5516D31F
P 7250 3050
F 0 "P110" H 7250 3400 50  0000 C CNN
F 1 "CONN_01X06" V 7350 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7250 3050 60  0001 C CNN
F 3 "" H 7250 3050 60  0000 C CNN
	1    7250 3050
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P111
U 1 1 5516D356
P 8250 3050
F 0 "P111" H 8250 3400 50  0000 C CNN
F 1 "CONN_01X06" V 8350 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 8250 3050 60  0001 C CNN
F 3 "" H 8250 3050 60  0000 C CNN
	1    8250 3050
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P104
U 1 1 5516D3D5
P 3600 5650
F 0 "P104" H 3600 5800 50  0000 C CNN
F 1 "CONN_01X02" V 3700 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3600 5650 60  0001 C CNN
F 3 "" H 3600 5650 60  0000 C CNN
	1    3600 5650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P105
U 1 1 5516D458
P 3600 6350
F 0 "P105" H 3600 6500 50  0000 C CNN
F 1 "CONN_01X02" V 3700 6350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3600 6350 60  0001 C CNN
F 3 "" H 3600 6350 60  0000 C CNN
	1    3600 6350
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P106
U 1 1 5516D47F
P 3600 7150
F 0 "P106" H 3600 7300 50  0000 C CNN
F 1 "CONN_01X02" V 3700 7150 50  0000 C CNN
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
Text Label 1850 7700 0    60   ~ 0
GND
Text Label 1450 5700 2    60   ~ 0
HTR0
Text Label 1450 6400 2    60   ~ 0
FAN0
Text Label 1450 7200 2    60   ~ 0
HTR1
Text Label 1750 5100 2    60   ~ 0
12V
Text Label 3200 5100 2    60   ~ 0
12V
Text Notes 4300 5650 2    60   ~ 0
HEATER 0
Text Notes 4200 6400 2    60   ~ 0
FAN 0
Text Notes 4250 7200 2    60   ~ 0
HEATER 1
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
	2500 4050 2250 4050
Wire Wire Line
	2500 4150 2250 4150
Wire Wire Line
	2500 4250 2250 4250
Wire Wire Line
	2500 4350 2250 4350
Wire Wire Line
	2500 4450 2250 4450
Wire Wire Line
	2500 4550 2250 4550
Wire Wire Line
	3800 3450 4050 3450
Wire Wire Line
	3800 3550 8100 3550
Wire Wire Line
	3800 3650 4050 3650
Wire Wire Line
	3800 3750 4050 3750
Wire Wire Line
	3800 4550 4050 4550
Wire Wire Line
	3800 4450 4300 4450
Wire Wire Line
	3800 4350 8500 4350
Wire Wire Line
	3800 4250 8300 4250
Wire Wire Line
	3800 4150 8200 4150
Wire Wire Line
	3800 4050 7200 4050
Wire Wire Line
	3800 3950 6200 3950
Wire Wire Line
	3700 2900 3500 2900
Wire Wire Line
	3700 2200 3500 2200
Wire Wire Line
	3700 2100 3500 2100
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
	1800 5850 1800 7700
Connection ~ 1800 6550
Wire Wire Line
	1800 7700 1850 7700
Connection ~ 1800 7350
Wire Wire Line
	3200 6300 3400 6300
Connection ~ 3200 5600
Wire Wire Line
	3200 7100 3400 7100
Connection ~ 3200 6300
Wire Wire Line
	8500 4350 8500 3250
Wire Wire Line
	4300 4450 4300 4300
Wire Wire Line
	4300 4300 8400 4300
Wire Wire Line
	8400 4300 8400 3250
Wire Wire Line
	8300 4250 8300 3250
Wire Wire Line
	8200 4150 8200 3250
Wire Wire Line
	7200 4050 7200 3250
Wire Wire Line
	6200 3950 6200 3250
Wire Wire Line
	6500 3250 6500 4350
Connection ~ 6500 4350
Wire Wire Line
	7500 3250 7500 4350
Connection ~ 7500 4350
Wire Wire Line
	7400 4300 7400 3250
Connection ~ 7400 4300
Wire Wire Line
	7300 3250 7300 4250
Connection ~ 7300 4250
Wire Wire Line
	8100 3550 8100 3250
Wire Wire Line
	7100 3250 7100 3550
Connection ~ 7100 3550
Wire Wire Line
	8000 3250 8000 3450
Wire Wire Line
	8000 3450 4850 3450
Wire Wire Line
	3650 2900 3650 3300
Wire Wire Line
	3650 3300 4850 3300
Wire Wire Line
	4850 3300 4850 3450
Connection ~ 3650 2900
Wire Wire Line
	7000 3250 7000 3450
Connection ~ 7000 3450
Wire Wire Line
	6000 3450 6000 3250
Connection ~ 6000 3450
Wire Wire Line
	6100 3250 6100 3550
Connection ~ 6100 3550
Text Notes 6450 2800 2    60   ~ 0
SENSOR2
Text Notes 7400 2800 2    60   ~ 0
SENSOR1
Text Notes 8450 2800 2    60   ~ 0
SENSOR0
Wire Wire Line
	6400 3250 6400 4300
Connection ~ 6400 4300
Wire Wire Line
	6300 3250 6300 4250
Connection ~ 6300 4250
$EndSCHEMATC
