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
LIBS:adapter-cache
EELAYER 24 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2350 4300 0    60   Input ~ 0
GND
Text HLabel 2350 2600 0    60   Input ~ 0
12V
Text HLabel 2350 3450 0    60   Input ~ 0
INV_PWM
$Comp
L R R201
U 1 1 5516D5B5
P 3000 3000
AR Path="/5516D580/5516D5B5" Ref="R201"  Part="1" 
AR Path="/5516E183/5516D5B5" Ref="R301"  Part="1" 
AR Path="/5516E3F1/5516D5B5" Ref="R401"  Part="1" 
F 0 "R401" V 3080 3000 50  0000 C CNN
F 1 "10k" V 3007 3001 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2930 3000 30  0001 C CNN
F 3 "" H 3000 3000 30  0000 C CNN
	1    3000 3000
	1    0    0    -1  
$EndComp
$Comp
L R R203
U 1 1 5516D5F0
P 4450 3000
AR Path="/5516D580/5516D5F0" Ref="R203"  Part="1" 
AR Path="/5516E183/5516D5F0" Ref="R303"  Part="1" 
AR Path="/5516E3F1/5516D5F0" Ref="R403"  Part="1" 
F 0 "R403" V 4530 3000 50  0000 C CNN
F 1 "10k" V 4457 3001 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4380 3000 30  0001 C CNN
F 3 "" H 4450 3000 30  0000 C CNN
	1    4450 3000
	1    0    0    -1  
$EndComp
$Comp
L R R202
U 1 1 5516D648
P 3500 3450
AR Path="/5516D580/5516D648" Ref="R202"  Part="1" 
AR Path="/5516E183/5516D648" Ref="R302"  Part="1" 
AR Path="/5516E3F1/5516D648" Ref="R402"  Part="1" 
F 0 "R402" V 3580 3450 50  0000 C CNN
F 1 "1k" V 3507 3451 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3430 3450 30  0001 C CNN
F 3 "" H 3500 3450 30  0000 C CNN
	1    3500 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 2600 4450 2600
Wire Wire Line
	3000 2600 3000 2750
Wire Wire Line
	4450 2600 4450 2750
Connection ~ 3000 2600
Wire Wire Line
	3000 3250 3000 3450
Wire Wire Line
	2350 3450 3250 3450
Wire Wire Line
	3750 3450 3900 3450
Connection ~ 3000 3450
$Comp
L NPN Q201
U 1 1 5516D75F
P 4350 3750
AR Path="/5516D580/5516D75F" Ref="Q201"  Part="1" 
AR Path="/5516E183/5516D75F" Ref="Q301"  Part="1" 
AR Path="/5516E3F1/5516D75F" Ref="Q401"  Part="1" 
F 0 "Q401" H 4350 3600 50  0000 R CNN
F 1 "BC850" H 4350 3900 50  0000 R CNN
F 2 "smd_packages_fork:SOT23EBC" H 4350 3750 60  0001 C CNN
F 3 "" H 4350 3750 60  0000 C CNN
	1    4350 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3450 3900 3750
Wire Wire Line
	3900 3750 4150 3750
Wire Wire Line
	4450 3250 4450 3550
Wire Wire Line
	4450 3950 4450 4300
Wire Wire Line
	2350 4300 5050 4300
$Comp
L MOS_N Q202
U 1 1 5516D844
P 4950 3400
AR Path="/5516D580/5516D844" Ref="Q202"  Part="1" 
AR Path="/5516E183/5516D844" Ref="Q302"  Part="1" 
AR Path="/5516E3F1/5516D844" Ref="Q402"  Part="1" 
F 0 "Q402" H 4960 3570 50  0000 R CNN
F 1 "IRF530N" H 4900 3250 50  0000 R CNN
F 2 "Transistors_TO-220:TO-220_FET-GDS_Vertical" H 4950 3400 60  0001 C CNN
F 3 "" H 4950 3400 60  0000 C CNN
	1    4950 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4300 5050 3600
Connection ~ 4450 4300
Wire Wire Line
	4750 3400 4450 3400
Connection ~ 4450 3400
Wire Wire Line
	5050 3200 5050 2900
Wire Wire Line
	5050 2900 5300 2900
Text HLabel 5300 2900 2    60   Output ~ 0
Drain
$EndSCHEMATC
