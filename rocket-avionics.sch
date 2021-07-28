EESchema Schematic File Version 4
EELAYER 30 0
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
Text Notes 7400 7500 0    50   ~ 0
Rocket Avionics
Text Notes 8150 7650 0    50   ~ 0
24 July 2021
Text Notes 10600 7650 0    50   ~ 0
1-0
Text Notes 7000 6650 0    50   ~ 0
1-0: ATMEGA328P 3.3V integrated version.
$Comp
L Connector:Micro_SD_Card J?
U 1 1 60FC3766
P 4000 6250
F 0 "J?" H 3950 6967 50  0000 C CNN
F 1 "Micro_SD_Card" H 3950 6876 50  0000 C CNN
F 2 "" H 5150 6550 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 4000 6250 50  0001 C CNN
	1    4000 6250
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-6050 U?
U 1 1 60FC50AF
P 9250 2450
F 0 "U?" H 9250 1661 50  0000 C CNN
F 1 "MPU-6050" H 9250 1570 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 9250 1650 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 9250 2300 50  0001 C CNN
	1    9250 2450
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Pressure:BMP280 U?
U 1 1 60FC5BFD
P 9200 4750
F 0 "U?" H 9430 4846 50  0000 L CNN
F 1 "BMP280" H 9430 4755 50  0000 L CNN
F 2 "Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering" H 9200 4050 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001.pdf" H 9200 4750 50  0001 C CNN
	1    9200 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 60FC652B
P 2950 3500
F 0 "D?" H 2943 3717 50  0000 C CNN
F 1 "Green LED" H 2943 3626 50  0000 C CNN
F 2 "" H 2950 3500 50  0001 C CNN
F 3 "~" H 2950 3500 50  0001 C CNN
	1    2950 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 60FC66B7
P 2950 4000
F 0 "D?" H 2943 4217 50  0000 C CNN
F 1 "Red LED" H 2943 4126 50  0000 C CNN
F 2 "" H 2950 4000 50  0001 C CNN
F 3 "~" H 2950 4000 50  0001 C CNN
	1    2950 4000
	-1   0    0    1   
$EndComp
$Comp
L Device:Crystal Y?
U 1 1 60FC6E65
P 7800 4700
F 0 "Y?" H 7800 4968 50  0000 C CNN
F 1 "Crystal" H 7800 4877 50  0000 C CNN
F 2 "" H 7800 4700 50  0001 C CNN
F 3 "~" H 7800 4700 50  0001 C CNN
	1    7800 4700
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM3480-3.3 U?
U 1 1 60FC8413
P 3250 2100
F 0 "U?" H 3250 2342 50  0000 C CNN
F 1 "AP2111H-3.3" H 3250 2251 50  0000 C CNN
F 2 "" H 3250 2325 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm3480.pdf" H 3250 2100 50  0001 C CNN
	1    3250 2100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 60FC7A8D
P 1750 2200
F 0 "J?" H 1642 1875 50  0000 C CNN
F 1 "Conn_01x02_Female" H 1642 1966 50  0000 C CNN
F 2 "" H 1750 2200 50  0001 C CNN
F 3 "~" H 1750 2200 50  0001 C CNN
	1    1750 2200
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60FC7F2F
P 3900 1750
F 0 "#PWR?" H 3900 1600 50  0001 C CNN
F 1 "+3.3V" H 3915 1923 50  0000 C CNN
F 2 "" H 3900 1750 50  0001 C CNN
F 3 "" H 3900 1750 50  0001 C CNN
	1    3900 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FC83DF
P 3250 2600
F 0 "#PWR?" H 3250 2350 50  0001 C CNN
F 1 "GND" H 3255 2427 50  0000 C CNN
F 2 "" H 3250 2600 50  0001 C CNN
F 3 "" H 3250 2600 50  0001 C CNN
	1    3250 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FC8637
P 2200 2600
F 0 "#PWR?" H 2200 2350 50  0001 C CNN
F 1 "GND" H 2205 2427 50  0000 C CNN
F 2 "" H 2200 2600 50  0001 C CNN
F 3 "" H 2200 2600 50  0001 C CNN
	1    2200 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2200 2200 2200
Wire Wire Line
	2200 2200 2200 2600
Wire Wire Line
	3250 2400 3250 2600
Wire Wire Line
	3900 2100 3900 1750
Text Notes 1450 1450 0    50   ~ 0
Lipo Battery Power Supply
Wire Notes Line
	1400 2900 1400 1350
$Comp
L Device:R R?
U 1 1 60FCFBFB
P 2450 3500
F 0 "R?" V 2243 3500 50  0000 C CNN
F 1 "220R" V 2334 3500 50  0000 C CNN
F 2 "" V 2380 3500 50  0001 C CNN
F 3 "~" H 2450 3500 50  0001 C CNN
	1    2450 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD03A9
P 2450 4000
F 0 "R?" V 2243 4000 50  0000 C CNN
F 1 "220R" V 2334 4000 50  0000 C CNN
F 2 "" V 2380 4000 50  0001 C CNN
F 3 "~" H 2450 4000 50  0001 C CNN
	1    2450 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FD3593
P 3400 4250
F 0 "#PWR?" H 3400 4000 50  0001 C CNN
F 1 "GND" H 3405 4077 50  0000 C CNN
F 2 "" H 3400 4250 50  0001 C CNN
F 3 "" H 3400 4250 50  0001 C CNN
	1    3400 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3500 3400 3500
Wire Wire Line
	3400 3500 3400 4000
Wire Wire Line
	3100 4000 3400 4000
Connection ~ 3400 4000
Wire Wire Line
	3400 4000 3400 4250
Wire Wire Line
	2600 4000 2800 4000
Wire Wire Line
	2600 3500 2800 3500
Text GLabel 2000 3500 0    50   Input ~ 0
SuccessLED
Text GLabel 2000 4000 0    50   Input ~ 0
ErrorLED
Wire Wire Line
	2000 3500 2300 3500
Wire Wire Line
	2000 4000 2300 4000
Text Notes 1450 3200 0    50   ~ 0
Status LEDs
Wire Notes Line
	1400 3050 3550 3050
Wire Notes Line
	3550 3050 3550 4550
Wire Notes Line
	3550 4550 1400 4550
Wire Notes Line
	1400 4550 1400 3050
Text GLabel 1900 6350 0    50   Input ~ 0
SPI-SCK
$Comp
L power:GND #PWR?
U 1 1 60FD907C
P 2100 6800
F 0 "#PWR?" H 2100 6550 50  0001 C CNN
F 1 "GND" H 2105 6627 50  0000 C CNN
F 2 "" H 2100 6800 50  0001 C CNN
F 3 "" H 2100 6800 50  0001 C CNN
	1    2100 6800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60FD9B39
P 2100 5100
F 0 "#PWR?" H 2100 4950 50  0001 C CNN
F 1 "+3.3V" H 2115 5273 50  0000 C CNN
F 2 "" H 2100 5100 50  0001 C CNN
F 3 "" H 2100 5100 50  0001 C CNN
	1    2100 5100
	1    0    0    -1  
$EndComp
Text GLabel 1900 6050 0    50   Input ~ 0
SPI-SS
Text GLabel 1900 6150 0    50   Input ~ 0
SPI-MOSI
Text GLabel 1900 6550 0    50   Input ~ 0
SPI-MISO
Wire Wire Line
	3100 5950 2850 5950
Wire Wire Line
	2850 5950 2850 5750
$Comp
L Device:R R?
U 1 1 60FE3369
P 2850 5600
F 0 "R?" H 2900 5300 50  0000 R CNN
F 1 "10k" H 2900 5200 50  0000 R CNN
F 2 "" V 2780 5600 50  0001 C CNN
F 3 "~" H 2850 5600 50  0001 C CNN
	1    2850 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 6050 2700 6050
Wire Wire Line
	2700 6050 2700 5750
Wire Wire Line
	3100 6150 2550 6150
Wire Wire Line
	2550 6150 2550 5750
Wire Wire Line
	3100 6550 2400 6550
Wire Wire Line
	2400 6550 2400 5750
Wire Wire Line
	3100 6650 2250 6650
Wire Wire Line
	2250 6650 2250 5750
Wire Wire Line
	1900 6050 2700 6050
Connection ~ 2700 6050
Wire Wire Line
	1900 6150 2550 6150
Connection ~ 2550 6150
Wire Wire Line
	1900 6350 3100 6350
Wire Wire Line
	1900 6550 2400 6550
Connection ~ 2400 6550
Wire Wire Line
	2100 6250 3100 6250
Wire Wire Line
	2100 6450 2100 6800
Wire Wire Line
	2100 6450 3100 6450
$Comp
L power:GND #PWR?
U 1 1 60FF813E
P 4950 7000
F 0 "#PWR?" H 4950 6750 50  0001 C CNN
F 1 "GND" H 4955 6827 50  0000 C CNN
F 2 "" H 4950 7000 50  0001 C CNN
F 3 "" H 4950 7000 50  0001 C CNN
	1    4950 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 6850 4950 6850
Wire Wire Line
	4950 6850 4950 7000
$Comp
L Device:C C?
U 1 1 61000346
P 1700 5400
F 0 "C?" H 1815 5446 50  0000 L CNN
F 1 "0.1uF" H 1815 5355 50  0000 L CNN
F 2 "" H 1738 5250 50  0001 C CNN
F 3 "~" H 1700 5400 50  0001 C CNN
	1    1700 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61000E14
P 1700 5600
F 0 "#PWR?" H 1700 5350 50  0001 C CNN
F 1 "GND" H 1705 5427 50  0000 C CNN
F 2 "" H 1700 5600 50  0001 C CNN
F 3 "" H 1700 5600 50  0001 C CNN
	1    1700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5200 1700 5250
Wire Wire Line
	1700 5550 1700 5600
Wire Wire Line
	2850 5450 2850 5400
Wire Wire Line
	2850 5400 2700 5400
Connection ~ 2100 5400
Wire Wire Line
	2100 5400 2100 6250
$Comp
L Device:R R?
U 1 1 61008E9C
P 2700 5600
F 0 "R?" H 2750 5300 50  0000 R CNN
F 1 "10k" H 2750 5200 50  0000 R CNN
F 2 "" V 2630 5600 50  0001 C CNN
F 3 "~" H 2700 5600 50  0001 C CNN
	1    2700 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 610090A5
P 2550 5600
F 0 "R?" H 2600 5300 50  0000 R CNN
F 1 "10k" H 2600 5200 50  0000 R CNN
F 2 "" V 2480 5600 50  0001 C CNN
F 3 "~" H 2550 5600 50  0001 C CNN
	1    2550 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 6100920B
P 2400 5600
F 0 "R?" H 2450 5300 50  0000 R CNN
F 1 "10k" H 2450 5200 50  0000 R CNN
F 2 "" V 2330 5600 50  0001 C CNN
F 3 "~" H 2400 5600 50  0001 C CNN
	1    2400 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 6100937B
P 2250 5600
F 0 "R?" H 2300 5300 50  0000 R CNN
F 1 "10k" H 2300 5200 50  0000 R CNN
F 2 "" V 2180 5600 50  0001 C CNN
F 3 "~" H 2250 5600 50  0001 C CNN
	1    2250 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	2250 5450 2250 5400
Connection ~ 2250 5400
Wire Wire Line
	2250 5400 2100 5400
Wire Wire Line
	2400 5450 2400 5400
Connection ~ 2400 5400
Wire Wire Line
	2400 5400 2250 5400
Wire Wire Line
	2550 5450 2550 5400
Connection ~ 2550 5400
Wire Wire Line
	2550 5400 2400 5400
Wire Wire Line
	2700 5450 2700 5400
Connection ~ 2700 5400
Wire Wire Line
	2700 5400 2550 5400
Wire Wire Line
	2100 5100 2100 5200
Wire Wire Line
	1700 5200 2100 5200
Connection ~ 2100 5200
Wire Wire Line
	2100 5200 2100 5400
Text Notes 1450 4800 0    50   ~ 0
MicroSD Card
Wire Notes Line
	5100 4700 5100 7300
Wire Notes Line
	5100 7300 1400 7300
Wire Notes Line
	1400 7300 1400 4700
Wire Notes Line
	1400 4700 5100 4700
$Comp
L Device:D D?
U 1 1 6101BE6B
P 2450 2100
F 0 "D?" H 2450 1883 50  0000 C CNN
F 1 "D" H 2450 1974 50  0000 C CNN
F 2 "" H 2450 2100 50  0001 C CNN
F 3 "~" H 2450 2100 50  0001 C CNN
	1    2450 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 2100 2300 2100
Wire Wire Line
	2600 2100 2700 2100
$Comp
L power:GND #PWR?
U 1 1 610282A3
P 2700 2600
F 0 "#PWR?" H 2700 2350 50  0001 C CNN
F 1 "GND" H 2705 2427 50  0000 C CNN
F 2 "" H 2700 2600 50  0001 C CNN
F 3 "" H 2700 2600 50  0001 C CNN
	1    2700 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2600 2700 2500
Wire Wire Line
	2700 2200 2700 2100
Connection ~ 2700 2100
Wire Wire Line
	2700 2100 2950 2100
$Comp
L power:GND #PWR?
U 1 1 6102F42A
P 3750 2600
F 0 "#PWR?" H 3750 2350 50  0001 C CNN
F 1 "GND" H 3755 2427 50  0000 C CNN
F 2 "" H 3750 2600 50  0001 C CNN
F 3 "" H 3750 2600 50  0001 C CNN
	1    3750 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2500 3750 2600
Wire Wire Line
	3750 2200 3750 2100
Connection ~ 3750 2100
Wire Wire Line
	3750 2100 3900 2100
Wire Wire Line
	3550 2100 3750 2100
$Comp
L Device:C C?
U 1 1 61027742
P 2700 2350
F 0 "C?" H 2815 2396 50  0000 L CNN
F 1 "1uF" H 2815 2305 50  0000 L CNN
F 2 "" H 2738 2200 50  0001 C CNN
F 3 "~" H 2700 2350 50  0001 C CNN
	1    2700 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 6102EEFB
P 3750 2350
F 0 "C?" H 3865 2396 50  0000 L CNN
F 1 "1uF" H 3865 2305 50  0000 L CNN
F 2 "" H 3788 2200 50  0001 C CNN
F 3 "~" H 3750 2350 50  0001 C CNN
	1    3750 2350
	1    0    0    -1  
$EndComp
Wire Notes Line
	4150 1350 4150 2900
Wire Notes Line
	1400 1350 4150 1350
Wire Notes Line
	1400 2900 4150 2900
Wire Wire Line
	6350 1800 6650 1800
Wire Wire Line
	6350 1700 6650 1700
Wire Wire Line
	6650 1600 6350 1600
Wire Wire Line
	6350 1500 6650 1500
Text GLabel 6650 1800 2    50   Input ~ 0
SPI-SCK
Text GLabel 6650 1700 2    50   Input ~ 0
SPI-MISO
Text GLabel 6650 1600 2    50   Input ~ 0
SPI-MOSI
Text GLabel 6650 1500 2    50   Input ~ 0
SPI-SS
Wire Wire Line
	6350 1400 6650 1400
Wire Wire Line
	6350 1300 6650 1300
Text GLabel 6650 1400 2    50   Input ~ 0
ErrorLED
Text GLabel 6650 1300 2    50   Input ~ 0
SuccessLED
$Comp
L MCU_Microchip_ATmega:ATmega328P-AU U?
U 1 1 60FC1465
P 5750 2500
F 0 "U?" H 5750 911 50  0000 C CNN
F 1 "ATmega328P-AU" H 5750 820 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 5750 2500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 5750 2500 50  0001 C CNN
	1    5750 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FEF5FA
P 8350 2950
F 0 "#PWR?" H 8350 2700 50  0001 C CNN
F 1 "GND" H 8355 2777 50  0000 C CNN
F 2 "" H 8350 2950 50  0001 C CNN
F 3 "" H 8350 2950 50  0001 C CNN
	1    8350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2650 8350 2650
Wire Wire Line
	8350 2650 8350 2750
Wire Wire Line
	8550 2750 8350 2750
Connection ~ 8350 2750
Wire Wire Line
	8350 2750 8350 2950
NoConn ~ 9950 2350
NoConn ~ 9950 2450
NoConn ~ 9950 2150
NoConn ~ 8550 2350
Text GLabel 7950 2150 0    50   Input ~ 0
I2C-Data
Text GLabel 7950 2250 0    50   Input ~ 0
I2C-Clock
Text GLabel 6650 2600 2    50   Input ~ 0
I2C-Data
Text GLabel 6650 2700 2    50   Input ~ 0
I2C-Clock
Wire Wire Line
	6350 2600 6650 2600
Wire Wire Line
	6350 2700 6650 2700
$Comp
L power:+3.3V #PWR?
U 1 1 60FFDF4B
P 7900 1500
F 0 "#PWR?" H 7900 1350 50  0001 C CNN
F 1 "+3.3V" H 7915 1673 50  0000 C CNN
F 2 "" H 7900 1500 50  0001 C CNN
F 3 "" H 7900 1500 50  0001 C CNN
	1    7900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 2150 8350 2150
Wire Wire Line
	7950 2250 8150 2250
$Comp
L Device:R R?
U 1 1 61004B6A
P 8350 1850
F 0 "R?" H 8400 1550 50  0000 R CNN
F 1 "10k" H 8400 1450 50  0000 R CNN
F 2 "" V 8280 1850 50  0001 C CNN
F 3 "~" H 8350 1850 50  0001 C CNN
	1    8350 1850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 610053C1
P 8150 1850
F 0 "R?" H 8200 1550 50  0000 R CNN
F 1 "10k" H 8200 1450 50  0000 R CNN
F 2 "" V 8080 1850 50  0001 C CNN
F 3 "~" H 8150 1850 50  0001 C CNN
	1    8150 1850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8350 2000 8350 2150
Connection ~ 8350 2150
Wire Wire Line
	8350 2150 8550 2150
Wire Wire Line
	8150 2000 8150 2250
Connection ~ 8150 2250
Wire Wire Line
	8150 2250 8550 2250
Wire Wire Line
	8350 1700 8350 1650
Wire Wire Line
	8350 1650 8150 1650
Wire Wire Line
	7900 1650 7900 1500
Wire Wire Line
	8150 1700 8150 1650
Connection ~ 8150 1650
Wire Wire Line
	8150 1650 7900 1650
$Comp
L power:GND #PWR?
U 1 1 610114E9
P 9250 3450
F 0 "#PWR?" H 9250 3200 50  0001 C CNN
F 1 "GND" H 9255 3277 50  0000 C CNN
F 2 "" H 9250 3450 50  0001 C CNN
F 3 "" H 9250 3450 50  0001 C CNN
	1    9250 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3450 9250 3150
$Comp
L Device:C C?
U 1 1 610133B8
P 10100 3100
F 0 "C?" H 10215 3146 50  0000 L CNN
F 1 "0.1uF" H 10215 3055 50  0000 L CNN
F 2 "" H 10138 2950 50  0001 C CNN
F 3 "~" H 10100 3100 50  0001 C CNN
	1    10100 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 6101455A
P 10550 3100
F 0 "C?" H 10665 3146 50  0000 L CNN
F 1 "2.2nF" H 10665 3055 50  0000 L CNN
F 2 "" H 10588 2950 50  0001 C CNN
F 3 "~" H 10550 3100 50  0001 C CNN
	1    10550 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 2750 10100 2750
Wire Wire Line
	10100 2750 10100 2950
Wire Wire Line
	9950 2650 10550 2650
Wire Wire Line
	10550 2650 10550 2950
$Comp
L power:GND #PWR?
U 1 1 6101A66D
P 10100 3450
F 0 "#PWR?" H 10100 3200 50  0001 C CNN
F 1 "GND" H 10105 3277 50  0000 C CNN
F 2 "" H 10100 3450 50  0001 C CNN
F 3 "" H 10100 3450 50  0001 C CNN
	1    10100 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 6100E241
P 9600 1650
F 0 "C?" H 9715 1696 50  0000 L CNN
F 1 "10nF" H 9715 1605 50  0000 L CNN
F 2 "" H 9638 1500 50  0001 C CNN
F 3 "~" H 9600 1650 50  0001 C CNN
	1    9600 1650
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 61017F93
P 9150 950
F 0 "#PWR?" H 9150 800 50  0001 C CNN
F 1 "+3.3V" H 9165 1123 50  0000 C CNN
F 2 "" H 9150 950 50  0001 C CNN
F 3 "" H 9150 950 50  0001 C CNN
	1    9150 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 610185B0
P 9600 1150
F 0 "C?" H 9715 1196 50  0000 L CNN
F 1 "0.1uF" H 9715 1105 50  0000 L CNN
F 2 "" H 9638 1000 50  0001 C CNN
F 3 "~" H 9600 1150 50  0001 C CNN
	1    9600 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9150 950  9150 1150
Wire Wire Line
	9450 1650 9350 1650
Connection ~ 9150 1650
Wire Wire Line
	9150 1650 9150 1750
Connection ~ 9150 1150
Wire Wire Line
	9150 1150 9150 1650
$Comp
L power:GND #PWR?
U 1 1 61021154
P 10000 1750
F 0 "#PWR?" H 10000 1500 50  0001 C CNN
F 1 "GND" H 10005 1577 50  0000 C CNN
F 2 "" H 10000 1750 50  0001 C CNN
F 3 "" H 10000 1750 50  0001 C CNN
	1    10000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 1650 10000 1650
Wire Wire Line
	9750 1150 10000 1150
Wire Wire Line
	10000 1150 10000 1650
Connection ~ 10000 1650
Wire Wire Line
	10000 1650 10000 1750
Wire Wire Line
	10100 3250 10100 3450
$Comp
L power:GND #PWR?
U 1 1 61026244
P 10550 3450
F 0 "#PWR?" H 10550 3200 50  0001 C CNN
F 1 "GND" H 10555 3277 50  0000 C CNN
F 2 "" H 10550 3450 50  0001 C CNN
F 3 "" H 10550 3450 50  0001 C CNN
	1    10550 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 3250 10550 3450
Wire Wire Line
	9150 1150 9450 1150
Wire Wire Line
	9350 1750 9350 1650
Connection ~ 9350 1650
Wire Wire Line
	9350 1650 9150 1650
Text Notes 7450 700  0    50   ~ 0
MPU-6050 IMU
Wire Notes Line
	10950 600  10950 3750
Wire Notes Line
	10950 3750 7400 3750
Wire Notes Line
	7400 3750 7400 600 
Wire Notes Line
	7400 600  10950 600 
$EndSCHEMATC
