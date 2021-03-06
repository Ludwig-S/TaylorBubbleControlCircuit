EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Taylor Bubble Control Circuit"
Date "2022-07-07"
Rev "1.0"
Comp "TU Dresden"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Amplifier_Operational:TL062 U3
U 1 1 62CD608B
P 2950 2550
F 0 "U3" H 2950 2917 50  0000 C CNN
F 1 "TL062" H 2950 2826 50  0000 C CNN
F 2 "" H 2950 2550 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A200%2FTL062%23STM.pdf" H 2950 2550 50  0001 C CNN
F 4 "https://www.reichelt.de/operationsverstaerker-2-fach-dip-8-tl-062-dip-p21552.html?&GROUP=A2C4&START=0&SORT=preis&OFFSET=16&nbc=1" H 2950 2550 50  0001 C CNN "Reichelt link"
	1    2950 2550
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TL062 U3
U 3 1 62CDCC37
P 5650 1650
F 0 "U3" H 5608 1696 50  0000 L CNN
F 1 "TL062" H 5608 1605 50  0000 L CNN
F 2 "" H 5650 1650 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A200%2FTL062%23STM.pdf" H 5650 1650 50  0001 C CNN
F 4 "https://www.reichelt.de/operationsverstaerker-2-fach-dip-8-tl-062-dip-p21552.html?&GROUP=A2C4&START=0&SORT=preis&OFFSET=16&nbc=1" H 5650 1650 50  0001 C CNN "Reichelt link"
	3    5650 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2550 3450 2550
Wire Wire Line
	3450 2550 3450 2650
Wire Wire Line
	2650 2800 2650 2650
$Comp
L power:GND #PWR01
U 1 1 62CF9495
P 2500 2450
F 0 "#PWR01" H 2500 2200 50  0001 C CNN
F 1 "GND" H 2505 2277 50  0000 C CNN
F 2 "" H 2500 2450 50  0001 C CNN
F 3 "" H 2500 2450 50  0001 C CNN
	1    2500 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2450 2500 2450
Wire Wire Line
	2650 2800 3300 2800
$Comp
L power:GND #PWR03
U 1 1 62D170A5
P 2650 2900
F 0 "#PWR03" H 2650 2650 50  0001 C CNN
F 1 "GND" H 2655 2727 50  0000 C CNN
F 2 "" H 2650 2900 50  0001 C CNN
F 3 "" H 2650 2900 50  0001 C CNN
	1    2650 2900
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U2
U 1 1 62D18C8D
P 3350 1350
F 0 "U2" H 3350 1592 50  0000 C CNN
F 1 "L7805CV-DG" H 3350 1501 50  0000 C CNN
F 2 "" H 3375 1200 50  0001 L CIN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A200%2FL78_ENG_TDS.pdf" H 3350 1300 50  0001 C CNN
F 4 "https://www.reichelt.de/spannungsregler-fest-5-v-1-5a-4-to-220-l7805cv-dg-stm-p216677.html?&GROUP=A2A2&START=0&SORT=preis&OFFSET=16&nbc=1" H 3350 1350 50  0001 C CNN "Reichelt link"
	1    3350 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 62D1A831
P 2750 1500
F 0 "C1" H 2868 1546 50  0000 L CNN
F 1 "330n" H 2868 1455 50  0000 L CNN
F 2 "" H 2788 1350 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=B300%2FSM_TRANS.pdf" H 2750 1500 50  0001 C CNN
F 4 "https://www.reichelt.de/subminiatur-elko-radial-330-nf-63-v-rm-1-5-85-c-1000h-20--sm-0-33-63rad-p18159.html?&GROUP=B315&START=0&SORT=preis&OFFSET=16&nbc=1" H 2750 1500 50  0001 C CNN "Reichelt link"
	1    2750 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 62D1B42A
P 3750 1500
F 0 "C2" H 3868 1546 50  0000 L CNN
F 1 "330n" H 3868 1455 50  0000 L CNN
F 2 "" H 3788 1350 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=B300%2FSM_TRANS.pdf" H 3750 1500 50  0001 C CNN
F 4 "https://www.reichelt.de/subminiatur-elko-radial-330-nf-63-v-rm-1-5-85-c-1000h-20--sm-0-33-63rad-p18159.html?&GROUP=B315&START=0&SORT=preis&OFFSET=16&nbc=1" H 3750 1500 50  0001 C CNN "Reichelt link"
	1    3750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 1350 3750 1350
Wire Wire Line
	3050 1350 2750 1350
$Comp
L power:GND #PWR04
U 1 1 62D1D949
P 2750 1650
F 0 "#PWR04" H 2750 1400 50  0001 C CNN
F 1 "GND" H 2755 1477 50  0000 C CNN
F 2 "" H 2750 1650 50  0001 C CNN
F 3 "" H 2750 1650 50  0001 C CNN
	1    2750 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 62D1DB97
P 3750 1650
F 0 "#PWR06" H 3750 1400 50  0001 C CNN
F 1 "GND" H 3755 1477 50  0000 C CNN
F 2 "" H 3750 1650 50  0001 C CNN
F 3 "" H 3750 1650 50  0001 C CNN
	1    3750 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 62D1E0D5
P 3350 1650
F 0 "#PWR05" H 3350 1400 50  0001 C CNN
F 1 "GND" H 3355 1477 50  0000 C CNN
F 2 "" H 3350 1650 50  0001 C CNN
F 3 "" H 3350 1650 50  0001 C CNN
	1    3350 1650
	1    0    0    -1  
$EndComp
Connection ~ 2750 1350
$Comp
L power:GND #PWR010
U 1 1 62D27363
P 5550 1950
F 0 "#PWR010" H 5550 1700 50  0001 C CNN
F 1 "GND" H 5555 1777 50  0000 C CNN
F 2 "" H 5550 1950 50  0001 C CNN
F 3 "" H 5550 1950 50  0001 C CNN
	1    5550 1950
	1    0    0    -1  
$EndComp
Text Notes 3550 1650 0    50   ~ 0
close to U1
$Comp
L Device:C C4
U 1 1 62D4D672
P 5150 1500
F 0 "C4" H 5035 1454 50  0000 R CNN
F 1 "100n" H 5035 1545 50  0000 R CNN
F 2 "" H 5188 1350 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=B300%2Fkerko.pdf" H 5150 1500 50  0001 C CNN
F 4 "https://www.reichelt.de/keramik-kondensator-100-nf-20-80-y5v-50-100-v-rm-5-kerko-100n-p9265.html?&GROUP=B353&START=0&SORT=preis&OFFSET=16&nbc=1" H 5150 1500 50  0001 C CNN "Reichelt link"
	1    5150 1500
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 62D4E897
P 5150 1650
F 0 "#PWR09" H 5150 1400 50  0001 C CNN
F 1 "GND" H 5155 1477 50  0000 C CNN
F 2 "" H 5150 1650 50  0001 C CNN
F 3 "" H 5150 1650 50  0001 C CNN
	1    5150 1650
	1    0    0    -1  
$EndComp
Text Notes 4950 1650 0    50   ~ 0
close to U3
Wire Wire Line
	5150 1350 5550 1350
Wire Wire Line
	4100 2550 3450 2550
Connection ~ 3450 2550
Text Notes 3500 2550 0    50   ~ 0
Photo Voltage
Text Notes 1950 2900 0    50   ~ 0
Kathode
Text Notes 1950 2800 0    50   ~ 0
Anode
Wire Wire Line
	2200 1350 2750 1350
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 62D22786
P 2000 1350
F 0 "J1" H 2100 1300 50  0000 C CNN
F 1 "Banana Plug" H 2300 1400 50  0000 C CNN
F 2 "" H 2000 1350 50  0001 C CNN
F 3 "~" H 2000 1350 50  0001 C CNN
F 4 "https://www.reichelt.de/bananenbuchse-4mm-vollisoliert-rot-bb-4-rt-p4896.html?&trstct=pos_9&nbc=1" H 2000 1350 50  0001 C CNN "Reichelt link"
	1    2000 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	3750 1350 4100 1350
Connection ~ 3750 1350
Text GLabel 4100 1350 2    50   Input ~ 0
5V_reg
Text GLabel 4100 2550 2    50   Input ~ 0
PhotoVoltage
Text GLabel 4950 1350 0    50   Input ~ 0
5V_reg
Wire Wire Line
	4950 1350 5150 1350
Connection ~ 5150 1350
Wire Wire Line
	3950 3900 4100 3900
$Comp
L power:GND #PWR07
U 1 1 62DDF2C7
P 4100 4200
F 0 "#PWR07" H 4100 3950 50  0001 C CNN
F 1 "GND" H 4105 4027 50  0000 C CNN
F 2 "" H 4100 4200 50  0001 C CNN
F 3 "" H 4100 4200 50  0001 C CNN
	1    4100 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 62DDF2C1
P 4100 4050
F 0 "C3" H 3985 4004 50  0000 R CNN
F 1 "100n" H 3985 4095 50  0000 R CNN
F 2 "" H 4138 3900 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=B300%2Fkerko.pdf" H 4100 4050 50  0001 C CNN
F 4 "https://www.reichelt.de/keramik-kondensator-100-nf-20-80-y5v-50-100-v-rm-5-kerko-100n-p9265.html?&GROUP=B353&START=0&SORT=preis&OFFSET=16&nbc=1" H 4100 4050 50  0001 C CNN "Reichelt link"
	1    4100 4050
	-1   0    0    1   
$EndComp
Text GLabel 3950 3900 0    50   Input ~ 0
5V_reg
Wire Wire Line
	2550 3700 2600 3700
Text GLabel 2550 3700 0    50   Input ~ 0
PWM_3V3
Wire Wire Line
	2600 4100 2600 4200
Connection ~ 2600 4100
Wire Wire Line
	2600 3900 2600 4100
$Comp
L Amplifier_Operational:TL062 U3
U 2 1 62CE49AF
P 2900 3800
F 0 "U3" H 2900 4167 50  0000 C CNN
F 1 "TL062" H 2900 4076 50  0000 C CNN
F 2 "" H 2900 3800 50  0001 C CNN
F 3 "https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A200%2FTL062%23STM.pdf" H 2900 3800 50  0001 C CNN
F 4 "https://www.reichelt.de/operationsverstaerker-2-fach-dip-8-tl-062-dip-p21552.html?&GROUP=A2C4&START=0&SORT=preis&OFFSET=16&nbc=1" H 2900 3800 50  0001 C CNN "Reichelt link"
	2    2900 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 62D5F966
P 3200 3950
F 0 "R1" H 3270 3996 50  0000 L CNN
F 1 "500" H 3270 3905 50  0000 L CNN
F 2 "" V 3130 3950 50  0001 C CNN
F 3 "~" H 3200 3950 50  0001 C CNN
	1    3200 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 62D60914
P 2600 4350
F 0 "R2" H 2670 4396 50  0000 L CNN
F 1 "1k" H 2670 4305 50  0000 L CNN
F 2 "" V 2530 4350 50  0001 C CNN
F 3 "~" H 2600 4350 50  0001 C CNN
	1    2600 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4100 2600 4100
$Comp
L power:GND #PWR02
U 1 1 62D6259C
P 2600 4500
F 0 "#PWR02" H 2600 4250 50  0001 C CNN
F 1 "GND" H 2605 4327 50  0000 C CNN
F 2 "" H 2600 4500 50  0001 C CNN
F 3 "" H 2600 4500 50  0001 C CNN
	1    2600 4500
	1    0    0    -1  
$EndComp
Text Notes 1300 2600 0    50   ~ 0
Photo Diode Connectors
Text Notes 4500 3500 0    50   ~ 0
Servo Motor Connectors
$Comp
L power:GND #PWR08
U 1 1 62DDD2CF
P 4600 4000
F 0 "#PWR08" H 4600 3750 50  0001 C CNN
F 1 "GND" H 4605 3827 50  0000 C CNN
F 2 "" H 4600 4000 50  0001 C CNN
F 3 "" H 4600 4000 50  0001 C CNN
	1    4600 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 62D238DB
P 4800 3900
F 0 "J4" H 4880 3942 50  0000 L CNN
F 1 "Conn_01x03" H 4880 3851 50  0000 L CNN
F 2 "" H 4800 3900 50  0001 C CNN
F 3 "~" H 4800 3900 50  0001 C CNN
F 4 "https://www.reichelt.de/fahrstuhlklemme-3-pol-4-mm-rm-5-08-akl-073-03-p36621.html?&GROUP=C1F3&START=0&SORT=preis&OFFSET=16&nbc=1" H 4800 3900 50  0001 C CNN "Reichelt link"
	1    4800 3900
	1    0    0    -1  
$EndComp
Text Notes 5450 4050 0    50   ~ 0
Connect\nPin 1 to white, yellow, orange or blue cable\nPin 2 to red cable\nPin 3 to black or brown cable\nof the servo motor
Text Notes 3350 3800 0    50   ~ 0
4.95V PWM
Wire Notes Line
	4450 3550 4450 4300
Wire Notes Line
	4450 4300 5400 4300
Wire Notes Line
	5400 4300 5400 3550
Wire Notes Line
	5400 3550 4450 3550
Wire Wire Line
	1950 2800 2650 2800
Connection ~ 2650 2800
Wire Wire Line
	1950 2900 2650 2900
Wire Notes Line
	1200 2650 2300 2650
Wire Notes Line
	2300 3000 1200 3000
Wire Notes Line
	2300 2650 2300 3000
Wire Notes Line
	1200 2650 1200 3000
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 62D22E75
P 1750 2900
F 0 "J3" H 1900 2900 50  0000 C CNN
F 1 "Conn_01x02" H 2050 2800 50  0000 C CNN
F 2 "" H 1750 2900 50  0001 C CNN
F 3 "~" H 1750 2900 50  0001 C CNN
F 4 "https://www.reichelt.de/fahrstuhlklemme-2-pol-4-mm-rm-5-08-akl-073-02-p36620.html?&nbc=1&trstct=lsbght_sldr::36621" H 1750 2900 50  0001 C CNN "Reichelt link"
	1    1750 2900
	-1   0    0    1   
$EndComp
Text Notes 3900 4200 0    50   ~ 0
close to J4
Wire Wire Line
	3200 3800 4600 3800
Connection ~ 3200 3800
Wire Wire Line
	4100 3900 4600 3900
Connection ~ 4100 3900
NoConn ~ 7950 4550
NoConn ~ 7950 4450
NoConn ~ 7950 4350
NoConn ~ 7950 4250
NoConn ~ 7950 4150
NoConn ~ 7950 4050
NoConn ~ 7950 3950
NoConn ~ 7950 3850
NoConn ~ 7950 3750
NoConn ~ 7950 3650
NoConn ~ 7950 3550
NoConn ~ 7950 3450
NoConn ~ 7950 3350
NoConn ~ 7950 3250
NoConn ~ 7950 3150
NoConn ~ 7950 3050
NoConn ~ 7950 2850
NoConn ~ 7950 2650
NoConn ~ 7950 2550
NoConn ~ 7950 1850
NoConn ~ 7950 1650
NoConn ~ 7950 1450
NoConn ~ 9350 4550
NoConn ~ 9350 4450
NoConn ~ 9350 4350
NoConn ~ 9350 4250
NoConn ~ 9350 4150
NoConn ~ 9350 4050
NoConn ~ 9350 3950
NoConn ~ 9350 3850
NoConn ~ 9350 3750
NoConn ~ 9350 3650
NoConn ~ 9350 3550
NoConn ~ 9350 3450
NoConn ~ 9350 3350
NoConn ~ 9350 3250
NoConn ~ 9350 3150
NoConn ~ 9350 2950
NoConn ~ 9350 2850
NoConn ~ 9350 2750
NoConn ~ 9350 2650
NoConn ~ 9350 2550
NoConn ~ 9350 2450
NoConn ~ 9350 2350
NoConn ~ 9350 2250
NoConn ~ 9350 2150
NoConn ~ 9350 2050
Wire Wire Line
	9850 1550 9350 1550
Wire Wire Line
	9350 1950 9850 1950
Text Notes 9400 1950 0    50   ~ 0
3.3V PWM
Text GLabel 9850 1950 2    50   Input ~ 0
PWM_3V3
Text Notes 9400 1550 0    50   ~ 0
ADC input
Text GLabel 9850 1550 2    50   Input ~ 0
PhotoVoltage
Wire Wire Line
	8450 4750 8450 4800
Wire Wire Line
	8550 4800 8650 4800
Connection ~ 8550 4800
Wire Wire Line
	8550 4750 8550 4800
Wire Wire Line
	8650 4750 8650 4800
Wire Wire Line
	8750 4800 8850 4800
Connection ~ 8750 4800
Wire Wire Line
	8750 4750 8750 4800
Wire Wire Line
	8650 4800 8750 4800
Connection ~ 8650 4800
Wire Wire Line
	8450 4800 8550 4800
Wire Wire Line
	8850 4800 8850 4750
Wire Wire Line
	8650 4850 8650 4800
$Comp
L power:GND #PWR011
U 1 1 62D8D228
P 8650 4850
F 0 "#PWR011" H 8650 4600 50  0001 C CNN
F 1 "GND" H 8650 4700 50  0000 C CNN
F 2 "" H 8650 4850 50  0001 C CNN
F 3 "" H 8650 4850 50  0001 C CNN
	1    8650 4850
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F4:STM32F446RETx U1
U 1 1 62D0A671
P 8650 2950
F 0 "U1" H 8650 750 50  0000 C CNN
F 1 "STM32 NUCLEO F446RE Board" H 8650 650 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 8050 1250 50  0001 R CNN
F 3 "https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf" H 8650 2950 50  0001 C CNN
F 4 "https://www.reichelt.de/nucleo-64-arm-cortex-stm32-f446-serie-nucleo-f446re-p217196.html?search=stm32f446" H 8650 2950 50  0001 C CNN "Reichelt link"
	1    8650 2950
	1    0    0    -1  
$EndComp
NoConn ~ 9350 1450
NoConn ~ 9350 1650
NoConn ~ 9350 1750
NoConn ~ 9350 1850
Text Notes 2550 1650 0    50   ~ 0
close to U1
Text Notes 2550 2100 0    50   ~ 0
Transimpedance amplifier
Text Notes 2500 3350 0    50   ~ 0
3.3V to 4.95V PWM conversion
Text Notes 1400 1050 0    50   ~ 0
Voltage Supply Connectors\n
Text Notes 5250 1300 0    50   ~ 0
Supply Voltage Pins \n    of U3
Text Notes 3000 1000 0    50   ~ 0
5V voltage regulator
Text Notes 3650 2950 0    50   ~ 0
choose actual resistor value \ndepending on the current of \nthe photo diode
$Comp
L Device:R_POT_TRIM RV1
U 1 1 62D00CBC
P 3450 2800
F 0 "RV1" V 3335 2800 50  0000 C CNN
F 1 "R_POT_TRIM" V 3244 2800 50  0000 C CNN
F 2 "" H 3450 2800 50  0001 C CNN
F 3 "~" H 3450 2800 50  0001 C CNN
	1    3450 2800
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 62D04412
P 2000 1650
F 0 "J2" H 2150 1650 50  0000 C CNN
F 1 "Banana Plug" H 2300 1750 50  0000 C CNN
F 2 "" H 2000 1650 50  0001 C CNN
F 3 "~" H 2000 1650 50  0001 C CNN
F 4 "https://www.reichelt.de/bananenbuchse-4mm-vollisoliert-schwarz-bb-4-sw-p4897.html?&trstct=pos_3&nbc=1" H 2000 1650 50  0001 C CNN "Reichelt link"
	1    2000 1650
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62D04E76
P 2200 1650
F 0 "#PWR?" H 2200 1400 50  0001 C CNN
F 1 "GND" H 2205 1477 50  0000 C CNN
F 2 "" H 2200 1650 50  0001 C CNN
F 3 "" H 2200 1650 50  0001 C CNN
	1    2200 1650
	1    0    0    -1  
$EndComp
Wire Notes Line
	1400 1900 2350 1900
Text Notes 1800 1250 0    50   ~ 0
8... 12V
Text Notes 1900 1600 0    50   ~ 0
GND
Wire Notes Line
	1400 1100 2350 1100
Wire Notes Line
	2350 1100 2350 1900
Wire Notes Line
	1400 1900 1400 1100
$EndSCHEMATC
