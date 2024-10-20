EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Aether microcontroller"
Date "2020-05-20"
Rev "Version 1.0"
Comp "Nanodyn"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 "Author: Nicholas Antonaides"
$EndDescr
$Comp
L Project-Aether-rescue:Oscillator G1
U 1 1 5EC73BC1
P 15100 9150
F 0 "G1" H 15125 9415 50  0000 C CNN
F 1 "Oscillator" H 15125 9324 50  0000 C CNN
F 2 "Project Aether footprints:Oscillator" H 15100 9350 50  0001 C CNN
F 3 "" H 15100 9350 50  0001 C CNN
	1    15100 9150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C61
U 1 1 5EC7AFC5
P 14850 9250
F 0 "C61" V 14829 9338 50  0000 L CNN
F 1 "C" V 14920 9338 50  0000 L CNN
F 2 "Project Aether footprints:C" H 14850 9350 50  0001 C CNN
F 3 "" H 14850 9350 50  0001 C CNN
	1    14850 9250
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR085
U 1 1 5EC7D62B
P 14850 9400
F 0 "#PWR085" H 14850 9150 50  0001 C CNN
F 1 "GNDREF" H 14855 9227 50  0000 C CNN
F 2 "" H 14850 9400 50  0001 C CNN
F 3 "" H 14850 9400 50  0001 C CNN
	1    14850 9400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR061
U 1 1 5EC8AA77
P 10600 1100
F 0 "#PWR061" H 10600 850 50  0001 C CNN
F 1 "GNDREF" H 10605 927 50  0000 C CNN
F 2 "" H 10600 1100 50  0001 C CNN
F 3 "" H 10600 1100 50  0001 C CNN
	1    10600 1100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S1
U 1 1 5EC82F69
P 10800 900
F 0 "S1" H 10800 975 50  0000 C CNN
F 1 "Button" H 10800 884 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 900 50  0001 C CNN
F 3 "" H 10800 900 50  0001 C CNN
	1    10800 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR073
U 1 1 5EC8C9C4
P 12500 3150
F 0 "#PWR073" H 12500 2900 50  0001 C CNN
F 1 "GNDREF" H 12505 2977 50  0000 C CNN
F 2 "" H 12500 3150 50  0001 C CNN
F 3 "" H 12500 3150 50  0001 C CNN
	1    12500 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 3150 12500 3150
$Comp
L power:+3.3V #PWR069
U 1 1 5EC8F6B9
P 12500 2450
F 0 "#PWR069" H 12500 2300 50  0001 C CNN
F 1 "+3.3V" H 12515 2623 50  0000 C CNN
F 2 "" H 12500 2450 50  0001 C CNN
F 3 "" H 12500 2450 50  0001 C CNN
	1    12500 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR076
U 1 1 5EC91F5C
P 13650 1850
F 0 "#PWR076" H 13650 1600 50  0001 C CNN
F 1 "GNDREF" H 13655 1677 50  0000 C CNN
F 2 "" H 13650 1850 50  0001 C CNN
F 3 "" H 13650 1850 50  0001 C CNN
	1    13650 1850
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR062
U 1 1 5EC94E54
P 10600 1600
F 0 "#PWR062" H 10600 1350 50  0001 C CNN
F 1 "GNDREF" H 10605 1427 50  0000 C CNN
F 2 "" H 10600 1600 50  0001 C CNN
F 3 "" H 10600 1600 50  0001 C CNN
	1    10600 1600
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S2
U 1 1 5EC94E5A
P 10800 1400
F 0 "S2" H 10800 1475 50  0000 C CNN
F 1 "Button" H 10800 1384 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 1400 50  0001 C CNN
F 3 "" H 10800 1400 50  0001 C CNN
	1    10800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	13000 3450 13100 3450
Wire Wire Line
	14450 4000 14450 4350
$Comp
L power:GNDREF #PWR083
U 1 1 5EC9A7F4
P 14550 4050
F 0 "#PWR083" H 14550 3800 50  0001 C CNN
F 1 "GNDREF" H 14555 3877 50  0000 C CNN
F 2 "" H 14550 4050 50  0001 C CNN
F 3 "" H 14550 4050 50  0001 C CNN
	1    14550 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	14550 4000 14550 4050
Wire Wire Line
	13100 3550 13100 4300
Wire Wire Line
	13950 4000 13950 4750
Wire Wire Line
	14050 4000 14050 4900
$Comp
L power:GNDREF #PWR063
U 1 1 5ECA1967
P 10600 2100
F 0 "#PWR063" H 10600 1850 50  0001 C CNN
F 1 "GNDREF" H 10605 1927 50  0000 C CNN
F 2 "" H 10600 2100 50  0001 C CNN
F 3 "" H 10600 2100 50  0001 C CNN
	1    10600 2100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S3
U 1 1 5ECA196D
P 10800 1900
F 0 "S3" H 10800 1975 50  0000 C CNN
F 1 "Button" H 10800 1884 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 1900 50  0001 C CNN
F 3 "" H 10800 1900 50  0001 C CNN
	1    10800 1900
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:MotorHeader J1
U 1 1 5ECA4C1D
P 13450 9150
F 0 "J1" H 13769 9121 50  0000 L CNN
F 1 "MotorHeader" H 13769 9030 50  0000 L CNN
F 2 "Project Aether footprints:MotorHeaders" H 13450 9600 50  0001 C CNN
F 3 "" H 13450 9600 50  0001 C CNN
	1    13450 9150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR078
U 1 1 5ECAA481
P 13800 9500
F 0 "#PWR078" H 13800 9250 50  0001 C CNN
F 1 "GNDREF" H 13805 9327 50  0000 C CNN
F 2 "" H 13800 9500 50  0001 C CNN
F 3 "" H 13800 9500 50  0001 C CNN
	1    13800 9500
	1    0    0    -1  
$EndComp
Wire Wire Line
	13800 9500 13600 9500
$Comp
L Project-Aether-rescue:R R41
U 1 1 5ED0E33E
P 4250 10550
F 0 "R41" H 4250 10775 50  0000 C CNN
F 1 "R" H 4250 10684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 10550 50  0001 C CNN
F 3 "" H 4250 10550 50  0001 C CNN
	1    4250 10550
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C26
U 1 1 5ED0E344
P 4100 10350
F 0 "C26" H 4125 10575 50  0000 C CNN
F 1 "C" H 4125 10484 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 10450 50  0001 C CNN
F 3 "" H 4100 10450 50  0001 C CNN
	1    4100 10350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R48
U 1 1 5ED0E34A
P 4450 10350
F 0 "R48" H 4450 10575 50  0000 C CNN
F 1 "R" H 4450 10484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 10350 50  0001 C CNN
F 3 "" H 4450 10350 50  0001 C CNN
	1    4450 10350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR026
U 1 1 5ED0E350
P 4250 10750
F 0 "#PWR026" H 4250 10500 50  0001 C CNN
F 1 "GNDREF" H 4255 10577 50  0000 C CNN
F 2 "" H 4250 10750 50  0001 C CNN
F 3 "" H 4250 10750 50  0001 C CNN
	1    4250 10750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C33
U 1 1 5ED0E356
P 4650 10450
F 0 "C33" H 4675 10675 50  0000 C CNN
F 1 "C" H 4675 10584 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 10550 50  0001 C CNN
F 3 "" H 4650 10550 50  0001 C CNN
	1    4650 10450
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR033
U 1 1 5ED0E35C
P 4650 10600
F 0 "#PWR033" H 4650 10350 50  0001 C CNN
F 1 "GNDREF" H 4655 10427 50  0000 C CNN
F 2 "" H 4650 10600 50  0001 C CNN
F 3 "" H 4650 10600 50  0001 C CNN
	1    4650 10600
	1    0    0    -1  
$EndComp
Wire Wire Line
	14250 4600 14250 4000
Wire Wire Line
	13100 3250 12750 3250
Wire Wire Line
	13850 4000 13850 4600
Wire Wire Line
	13750 4000 13750 4450
Wire Wire Line
	13650 4000 13650 4300
Wire Wire Line
	13550 4000 13550 4150
Text HLabel 12800 4150 0    50   Input ~ 0
PWM
Text HLabel 12900 4300 0    50   Input ~ 0
ADC2
Text HLabel 13550 4150 0    50   Input ~ 0
ADC3
Text HLabel 13650 4300 0    50   Input ~ 0
ADC4
Text HLabel 13750 4450 0    50   Input ~ 0
ADC5
Text HLabel 13850 4600 0    50   Input ~ 0
ADC6
Text HLabel 13950 4750 0    50   Input ~ 0
ADC7
Text HLabel 14050 4900 0    50   Input ~ 0
ADC8
Text HLabel 4650 6500 2    50   Input ~ 0
ADC2
Text HLabel 4650 7150 2    50   Input ~ 0
ADC3
Text HLabel 4650 7800 2    50   Input ~ 0
ADC4
Text HLabel 4650 8500 2    50   Input ~ 0
ADC5
Text HLabel 4650 9100 2    50   Input ~ 0
ADC6
Text HLabel 4650 9750 2    50   Input ~ 0
ADC7
Wire Notes Line
	5000 11100 5000 6150
Wire Notes Line
	5000 6150 3200 6150
Wire Notes Line
	3200 6150 3200 11100
Wire Notes Line
	3200 11100 5000 11100
Wire Wire Line
	13950 850  13950 2000
Text HLabel 15100 2900 2    50   Input ~ 0
Limit
Text HLabel 15100 3000 2    50   Input ~ 0
UART_TX
Text HLabel 15100 3100 2    50   Input ~ 0
UART_RX
$Comp
L Project-Aether-rescue:STM32F051C6 U2
U 1 1 5EC6AABC
P 14100 3000
F 0 "U2" H 15144 3046 50  0000 L CNN
F 1 "STM32F051C6" H 15144 2955 50  0000 L CNN
F 2 "Project Aether footprints:STM32F51C6_Header" H 13150 3450 50  0001 C CNN
F 3 "" H 13150 3450 50  0001 C CNN
	1    14100 3000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R1
U 1 1 5EDE1BF4
P 1950 2450
F 0 "R1" H 1950 2675 50  0000 C CNN
F 1 "R" H 1950 2584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 2450 50  0001 C CNN
F 3 "" H 1950 2450 50  0001 C CNN
	1    1950 2450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R16
U 1 1 5EDE2B55
P 2350 2450
F 0 "R16" H 2350 2675 50  0000 C CNN
F 1 "R" H 2350 2584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 2450 50  0001 C CNN
F 3 "" H 2350 2450 50  0001 C CNN
	1    2350 2450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C1
U 1 1 5EDE3E3E
P 2150 2550
F 0 "C1" V 2129 2638 50  0000 L CNN
F 1 "C" V 2220 2638 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 2650 50  0001 C CNN
F 3 "" H 2150 2650 50  0001 C CNN
	1    2150 2550
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR01
U 1 1 5EDE5008
P 2150 2700
F 0 "#PWR01" H 2150 2450 50  0001 C CNN
F 1 "GNDREF" H 2155 2527 50  0000 C CNN
F 2 "" H 2150 2700 50  0001 C CNN
F 3 "" H 2150 2700 50  0001 C CNN
	1    2150 2700
	1    0    0    -1  
$EndComp
Text HLabel 14350 4450 2    50   Input ~ 0
PB10
Wire Wire Line
	14350 4000 14350 4450
Text HLabel 15100 3600 2    50   Input ~ 0
PB12
Text HLabel 15100 3500 2    50   Input ~ 0
PB13
Text HLabel 15100 3400 2    50   Input ~ 0
PB14
Text HLabel 15100 3300 2    50   Input ~ 0
PB15
Text HLabel 15100 3200 2    50   Input ~ 0
PA8
Text HLabel 15100 2800 2    50   Input ~ 0
PA12
Text HLabel 15100 2700 2    50   Input ~ 0
PA13
Text HLabel 15100 2600 2    50   Input ~ 0
PF6
Text HLabel 15100 2500 2    50   Input ~ 0
PF7
Wire Wire Line
	13650 1850 13650 2000
Wire Wire Line
	13750 1600 13750 2000
Wire Wire Line
	13850 1500 13850 2000
Text HLabel 13750 1600 0    50   Input ~ 0
PB9
Text HLabel 13850 1500 0    50   Input ~ 0
PB8
$Comp
L Project-Aether-rescue:R R75
U 1 1 5EDC5631
P 13800 1050
F 0 "R75" V 13754 1138 50  0000 L CNN
F 1 "R" V 13845 1138 50  0000 L CNN
F 2 "Project Aether footprints:R" H 13800 1050 50  0001 C CNN
F 3 "" H 13800 1050 50  0001 C CNN
	1    13800 1050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13800 850  13950 850 
$Comp
L power:GNDREF #PWR077
U 1 1 5EDC81BE
P 13800 1250
F 0 "#PWR077" H 13800 1000 50  0001 C CNN
F 1 "GNDREF" H 13805 1077 50  0000 C CNN
F 2 "" H 13800 1250 50  0001 C CNN
F 3 "" H 13800 1250 50  0001 C CNN
	1    13800 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	14050 950  14050 2000
Wire Wire Line
	14150 1100 14150 2000
Wire Wire Line
	14250 1250 14250 2000
Wire Wire Line
	14350 1400 14350 2000
Wire Wire Line
	14550 1700 14550 2000
Wire Wire Line
	14650 1850 14650 2000
Text HLabel 2550 4100 2    50   Input ~ 0
PB12
Text HLabel 2550 3550 2    50   Input ~ 0
PB13
Text HLabel 2550 3000 2    50   Input ~ 0
PB14
Text HLabel 2550 2450 2    50   Input ~ 0
PB15
Text HLabel 2550 5200 2    50   Input ~ 0
PB10
Wire Wire Line
	14450 1550 14450 2000
Text HLabel 14050 950  2    50   Input ~ 0
PB7
Text HLabel 14150 1100 2    50   Input ~ 0
PB6
Text HLabel 14250 1250 2    50   Input ~ 0
PB5
Text HLabel 14350 1400 2    50   Input ~ 0
PB4
Text HLabel 14450 1550 2    50   Input ~ 0
PB3
Text HLabel 14550 1700 2    50   Input ~ 0
PA15
Text HLabel 14650 1850 2    50   Input ~ 0
PA14
Text HLabel 2550 6850 2    50   Input ~ 0
PB7
Text HLabel 2550 7400 2    50   Input ~ 0
PB6
Text HLabel 2550 7950 2    50   Input ~ 0
PB5
Text HLabel 2550 8500 2    50   Input ~ 0
PB4
Text HLabel 2550 9100 2    50   Input ~ 0
PB3
Text HLabel 2550 5750 2    50   Input ~ 0
PB9
Text HLabel 2550 6300 2    50   Input ~ 0
PB8
Wire Notes Line
	750  2100 2950 2100
Wire Notes Line
	14250 8750 14250 9900
Wire Notes Line
	14250 9900 13150 9900
Wire Notes Line
	13150 9900 13150 8750
Wire Notes Line
	13150 8750 14250 8750
Text HLabel 13100 2550 0    50   Input ~ 0
PC13
Text HLabel 13100 2650 0    50   Input ~ 0
PC14
Text HLabel 13100 2750 0    50   Input ~ 0
PC15
$Comp
L Project-Aether-rescue:R R52
U 1 1 5EF3A953
P 6400 6600
F 0 "R52" H 6400 6825 50  0000 C CNN
F 1 "R" H 6400 6734 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 6600 50  0001 C CNN
F 3 "" H 6400 6600 50  0001 C CNN
	1    6400 6600
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C34
U 1 1 5EF3A959
P 6200 6700
F 0 "C34" V 6179 6788 50  0000 L CNN
F 1 "C" V 6270 6788 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 6800 50  0001 C CNN
F 3 "" H 6200 6800 50  0001 C CNN
	1    6200 6700
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR034
U 1 1 5EF3A960
P 6200 6850
F 0 "#PWR034" H 6200 6600 50  0001 C CNN
F 1 "GNDREF" H 6205 6677 50  0000 C CNN
F 2 "" H 6200 6850 50  0001 C CNN
F 3 "" H 6200 6850 50  0001 C CNN
	1    6200 6850
	1    0    0    -1  
$EndComp
Text HLabel 6600 8900 2    50   Input ~ 0
Limit
Text HLabel 6600 9450 2    50   Input ~ 0
UART_TX
Text HLabel 6600 10000 2    50   Input ~ 0
UART_RX
Text HLabel 6600 10550 2    50   Input ~ 0
PA8
Text HLabel 6600 8350 2    50   Input ~ 0
PA12
Text HLabel 6600 7800 2    50   Input ~ 0
PA13
Text HLabel 6600 6600 2    50   Input ~ 0
PA15
Text HLabel 6600 7150 2    50   Input ~ 0
PA14
Wire Notes Line
	7200 11100 7200 6300
Wire Notes Line
	7200 6300 5200 6300
Wire Notes Line
	5200 6300 5200 11100
Wire Notes Line
	5200 11100 7200 11100
$Comp
L power:GNDREF #PWR089
U 1 1 5EC7DAEA
P 15350 9400
F 0 "#PWR089" H 15350 9150 50  0001 C CNN
F 1 "GNDREF" H 15355 9227 50  0000 C CNN
F 2 "" H 15350 9400 50  0001 C CNN
F 3 "" H 15350 9400 50  0001 C CNN
	1    15350 9400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C65
U 1 1 5EC7B582
P 15350 9250
F 0 "C65" V 15329 9338 50  0000 L CNN
F 1 "C" V 15420 9338 50  0000 L CNN
F 2 "Project Aether footprints:C" H 15350 9350 50  0001 C CNN
F 3 "" H 15350 9350 50  0001 C CNN
	1    15350 9250
	0    1    1    0   
$EndComp
Text HLabel 13100 2850 0    50   Input ~ 0
PF0
Text HLabel 13100 2950 0    50   Input ~ 0
PF1
Text HLabel 15450 9150 2    50   Input ~ 0
PF0
Text HLabel 14750 9150 0    50   Input ~ 0
PF1
Text HLabel 4050 5000 2    50   Input ~ 0
PF0
Text HLabel 4050 4850 2    50   Input ~ 0
PF1
Text HLabel 4050 4300 2    50   Input ~ 0
PF6
Text HLabel 4050 4150 2    50   Input ~ 0
PF7
Wire Notes Line
	4750 5900 4750 3300
Wire Notes Line
	4750 3300 3200 3300
Wire Notes Line
	3200 3300 3200 5900
Text HLabel 6250 5400 2    50   Input ~ 0
PC13
Text HLabel 6250 5100 2    50   Input ~ 0
PC15
Wire Notes Line
	5200 4200 5200 6150
Wire Notes Line
	5200 6150 7100 6150
Wire Notes Line
	7100 6150 7100 4200
Wire Notes Line
	7100 4200 5200 4200
Wire Wire Line
	14750 9150 14850 9150
Wire Wire Line
	15300 9150 15350 9150
$Comp
L Project-Aether-rescue:R R2
U 1 1 5EC7A69A
P 1950 3000
F 0 "R2" H 1950 3225 50  0000 C CNN
F 1 "R" H 1950 3134 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 3000 50  0001 C CNN
F 3 "" H 1950 3000 50  0001 C CNN
	1    1950 3000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R17
U 1 1 5EC7A6A0
P 2350 3000
F 0 "R17" H 2350 3225 50  0000 C CNN
F 1 "R" H 2350 3134 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 3000 50  0001 C CNN
F 3 "" H 2350 3000 50  0001 C CNN
	1    2350 3000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C2
U 1 1 5EC7A6A6
P 2150 3100
F 0 "C2" V 2129 3188 50  0000 L CNN
F 1 "C" V 2220 3188 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 3200 50  0001 C CNN
F 3 "" H 2150 3200 50  0001 C CNN
	1    2150 3100
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR02
U 1 1 5EC7A6AC
P 2150 3250
F 0 "#PWR02" H 2150 3000 50  0001 C CNN
F 1 "GNDREF" H 2155 3077 50  0000 C CNN
F 2 "" H 2150 3250 50  0001 C CNN
F 3 "" H 2150 3250 50  0001 C CNN
	1    2150 3250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R3
U 1 1 5EC85AA2
P 1950 3550
F 0 "R3" H 1950 3775 50  0000 C CNN
F 1 "R" H 1950 3684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 3550 50  0001 C CNN
F 3 "" H 1950 3550 50  0001 C CNN
	1    1950 3550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R18
U 1 1 5EC85AA8
P 2350 3550
F 0 "R18" H 2350 3775 50  0000 C CNN
F 1 "R" H 2350 3684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 3550 50  0001 C CNN
F 3 "" H 2350 3550 50  0001 C CNN
	1    2350 3550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C3
U 1 1 5EC85AAE
P 2150 3650
F 0 "C3" V 2129 3738 50  0000 L CNN
F 1 "C" V 2220 3738 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 3750 50  0001 C CNN
F 3 "" H 2150 3750 50  0001 C CNN
	1    2150 3650
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR03
U 1 1 5EC85AB4
P 2150 3800
F 0 "#PWR03" H 2150 3550 50  0001 C CNN
F 1 "GNDREF" H 2155 3627 50  0000 C CNN
F 2 "" H 2150 3800 50  0001 C CNN
F 3 "" H 2150 3800 50  0001 C CNN
	1    2150 3800
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R4
U 1 1 5EC8B301
P 1950 4100
F 0 "R4" H 1950 4325 50  0000 C CNN
F 1 "R" H 1950 4234 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 4100 50  0001 C CNN
F 3 "" H 1950 4100 50  0001 C CNN
	1    1950 4100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R19
U 1 1 5EC8B307
P 2350 4100
F 0 "R19" H 2350 4325 50  0000 C CNN
F 1 "R" H 2350 4234 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 4100 50  0001 C CNN
F 3 "" H 2350 4100 50  0001 C CNN
	1    2350 4100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C4
U 1 1 5EC8B30D
P 2150 4200
F 0 "C4" V 2129 4288 50  0000 L CNN
F 1 "C" V 2220 4288 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 4300 50  0001 C CNN
F 3 "" H 2150 4300 50  0001 C CNN
	1    2150 4200
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR04
U 1 1 5EC8B313
P 2150 4350
F 0 "#PWR04" H 2150 4100 50  0001 C CNN
F 1 "GNDREF" H 2155 4177 50  0000 C CNN
F 2 "" H 2150 4350 50  0001 C CNN
F 3 "" H 2150 4350 50  0001 C CNN
	1    2150 4350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R5
U 1 1 5EC9094C
P 1950 4650
F 0 "R5" H 1950 4875 50  0000 C CNN
F 1 "R" H 1950 4784 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 4650 50  0001 C CNN
F 3 "" H 1950 4650 50  0001 C CNN
	1    1950 4650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R20
U 1 1 5EC90952
P 2350 4650
F 0 "R20" H 2350 4875 50  0000 C CNN
F 1 "R" H 2350 4784 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 4650 50  0001 C CNN
F 3 "" H 2350 4650 50  0001 C CNN
	1    2350 4650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C5
U 1 1 5EC90958
P 2150 4750
F 0 "C5" V 2129 4838 50  0000 L CNN
F 1 "C" V 2220 4838 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 4850 50  0001 C CNN
F 3 "" H 2150 4850 50  0001 C CNN
	1    2150 4750
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR05
U 1 1 5EC9095E
P 2150 4900
F 0 "#PWR05" H 2150 4650 50  0001 C CNN
F 1 "GNDREF" H 2155 4727 50  0000 C CNN
F 2 "" H 2150 4900 50  0001 C CNN
F 3 "" H 2150 4900 50  0001 C CNN
	1    2150 4900
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R6
U 1 1 5EC959C3
P 1950 5200
F 0 "R6" H 1950 5425 50  0000 C CNN
F 1 "R" H 1950 5334 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 5200 50  0001 C CNN
F 3 "" H 1950 5200 50  0001 C CNN
	1    1950 5200
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R21
U 1 1 5EC959C9
P 2350 5200
F 0 "R21" H 2350 5425 50  0000 C CNN
F 1 "R" H 2350 5334 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 5200 50  0001 C CNN
F 3 "" H 2350 5200 50  0001 C CNN
	1    2350 5200
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C6
U 1 1 5EC959CF
P 2150 5300
F 0 "C6" V 2129 5388 50  0000 L CNN
F 1 "C" V 2220 5388 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 5400 50  0001 C CNN
F 3 "" H 2150 5400 50  0001 C CNN
	1    2150 5300
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR06
U 1 1 5EC959D5
P 2150 5450
F 0 "#PWR06" H 2150 5200 50  0001 C CNN
F 1 "GNDREF" H 2155 5277 50  0000 C CNN
F 2 "" H 2150 5450 50  0001 C CNN
F 3 "" H 2150 5450 50  0001 C CNN
	1    2150 5450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R7
U 1 1 5EC9AF80
P 1950 5750
F 0 "R7" H 1950 5975 50  0000 C CNN
F 1 "R" H 1950 5884 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 5750 50  0001 C CNN
F 3 "" H 1950 5750 50  0001 C CNN
	1    1950 5750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R22
U 1 1 5EC9AF86
P 2350 5750
F 0 "R22" H 2350 5975 50  0000 C CNN
F 1 "R" H 2350 5884 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 5750 50  0001 C CNN
F 3 "" H 2350 5750 50  0001 C CNN
	1    2350 5750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C7
U 1 1 5EC9AF8C
P 2150 5850
F 0 "C7" V 2129 5938 50  0000 L CNN
F 1 "C" V 2220 5938 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 5950 50  0001 C CNN
F 3 "" H 2150 5950 50  0001 C CNN
	1    2150 5850
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR07
U 1 1 5EC9AF92
P 2150 6000
F 0 "#PWR07" H 2150 5750 50  0001 C CNN
F 1 "GNDREF" H 2155 5827 50  0000 C CNN
F 2 "" H 2150 6000 50  0001 C CNN
F 3 "" H 2150 6000 50  0001 C CNN
	1    2150 6000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R8
U 1 1 5ECA05B7
P 1950 6300
F 0 "R8" H 1950 6525 50  0000 C CNN
F 1 "R" H 1950 6434 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 6300 50  0001 C CNN
F 3 "" H 1950 6300 50  0001 C CNN
	1    1950 6300
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R23
U 1 1 5ECA05BD
P 2350 6300
F 0 "R23" H 2350 6525 50  0000 C CNN
F 1 "R" H 2350 6434 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 6300 50  0001 C CNN
F 3 "" H 2350 6300 50  0001 C CNN
	1    2350 6300
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C8
U 1 1 5ECA05C3
P 2150 6400
F 0 "C8" V 2129 6488 50  0000 L CNN
F 1 "C" V 2220 6488 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 6500 50  0001 C CNN
F 3 "" H 2150 6500 50  0001 C CNN
	1    2150 6400
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR08
U 1 1 5ECA05C9
P 2150 6550
F 0 "#PWR08" H 2150 6300 50  0001 C CNN
F 1 "GNDREF" H 2155 6377 50  0000 C CNN
F 2 "" H 2150 6550 50  0001 C CNN
F 3 "" H 2150 6550 50  0001 C CNN
	1    2150 6550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R9
U 1 1 5ECA6076
P 1950 6850
F 0 "R9" H 1950 7075 50  0000 C CNN
F 1 "R" H 1950 6984 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 6850 50  0001 C CNN
F 3 "" H 1950 6850 50  0001 C CNN
	1    1950 6850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R24
U 1 1 5ECA607C
P 2350 6850
F 0 "R24" H 2350 7075 50  0000 C CNN
F 1 "R" H 2350 6984 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 6850 50  0001 C CNN
F 3 "" H 2350 6850 50  0001 C CNN
	1    2350 6850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C9
U 1 1 5ECA6082
P 2150 6950
F 0 "C9" V 2129 7038 50  0000 L CNN
F 1 "C" V 2220 7038 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 7050 50  0001 C CNN
F 3 "" H 2150 7050 50  0001 C CNN
	1    2150 6950
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR09
U 1 1 5ECA6088
P 2150 7100
F 0 "#PWR09" H 2150 6850 50  0001 C CNN
F 1 "GNDREF" H 2155 6927 50  0000 C CNN
F 2 "" H 2150 7100 50  0001 C CNN
F 3 "" H 2150 7100 50  0001 C CNN
	1    2150 7100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R10
U 1 1 5ECABA1F
P 1950 7400
F 0 "R10" H 1950 7625 50  0000 C CNN
F 1 "R" H 1950 7534 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 7400 50  0001 C CNN
F 3 "" H 1950 7400 50  0001 C CNN
	1    1950 7400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R25
U 1 1 5ECABA25
P 2350 7400
F 0 "R25" H 2350 7625 50  0000 C CNN
F 1 "R" H 2350 7534 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 7400 50  0001 C CNN
F 3 "" H 2350 7400 50  0001 C CNN
	1    2350 7400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C10
U 1 1 5ECABA2B
P 2150 7500
F 0 "C10" V 2129 7588 50  0000 L CNN
F 1 "C" V 2220 7588 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 7600 50  0001 C CNN
F 3 "" H 2150 7600 50  0001 C CNN
	1    2150 7500
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR010
U 1 1 5ECABA31
P 2150 7650
F 0 "#PWR010" H 2150 7400 50  0001 C CNN
F 1 "GNDREF" H 2155 7477 50  0000 C CNN
F 2 "" H 2150 7650 50  0001 C CNN
F 3 "" H 2150 7650 50  0001 C CNN
	1    2150 7650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R11
U 1 1 5ECB09F2
P 1950 7950
F 0 "R11" H 1950 8175 50  0000 C CNN
F 1 "R" H 1950 8084 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 7950 50  0001 C CNN
F 3 "" H 1950 7950 50  0001 C CNN
	1    1950 7950
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R26
U 1 1 5ECB09F8
P 2350 7950
F 0 "R26" H 2350 8175 50  0000 C CNN
F 1 "R" H 2350 8084 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 7950 50  0001 C CNN
F 3 "" H 2350 7950 50  0001 C CNN
	1    2350 7950
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C11
U 1 1 5ECB09FE
P 2150 8050
F 0 "C11" V 2129 8138 50  0000 L CNN
F 1 "C" V 2220 8138 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 8150 50  0001 C CNN
F 3 "" H 2150 8150 50  0001 C CNN
	1    2150 8050
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR011
U 1 1 5ECB0A04
P 2150 8200
F 0 "#PWR011" H 2150 7950 50  0001 C CNN
F 1 "GNDREF" H 2155 8027 50  0000 C CNN
F 2 "" H 2150 8200 50  0001 C CNN
F 3 "" H 2150 8200 50  0001 C CNN
	1    2150 8200
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R12
U 1 1 5ECB6429
P 1950 8500
F 0 "R12" H 1950 8725 50  0000 C CNN
F 1 "R" H 1950 8634 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 8500 50  0001 C CNN
F 3 "" H 1950 8500 50  0001 C CNN
	1    1950 8500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R27
U 1 1 5ECB642F
P 2350 8500
F 0 "R27" H 2350 8725 50  0000 C CNN
F 1 "R" H 2350 8634 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 8500 50  0001 C CNN
F 3 "" H 2350 8500 50  0001 C CNN
	1    2350 8500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C12
U 1 1 5ECB6435
P 2150 8600
F 0 "C12" V 2129 8688 50  0000 L CNN
F 1 "C" V 2220 8688 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 8700 50  0001 C CNN
F 3 "" H 2150 8700 50  0001 C CNN
	1    2150 8600
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR012
U 1 1 5ECB643B
P 2150 8750
F 0 "#PWR012" H 2150 8500 50  0001 C CNN
F 1 "GNDREF" H 2155 8577 50  0000 C CNN
F 2 "" H 2150 8750 50  0001 C CNN
F 3 "" H 2150 8750 50  0001 C CNN
	1    2150 8750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R13
U 1 1 5ECBB84E
P 1950 9100
F 0 "R13" H 1950 9325 50  0000 C CNN
F 1 "R" H 1950 9234 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 9100 50  0001 C CNN
F 3 "" H 1950 9100 50  0001 C CNN
	1    1950 9100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R28
U 1 1 5ECBB854
P 2350 9100
F 0 "R28" H 2350 9325 50  0000 C CNN
F 1 "R" H 2350 9234 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 9100 50  0001 C CNN
F 3 "" H 2350 9100 50  0001 C CNN
	1    2350 9100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C13
U 1 1 5ECBB85A
P 2150 9200
F 0 "C13" V 2129 9288 50  0000 L CNN
F 1 "C" V 2220 9288 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 9300 50  0001 C CNN
F 3 "" H 2150 9300 50  0001 C CNN
	1    2150 9200
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR013
U 1 1 5ECBB860
P 2150 9350
F 0 "#PWR013" H 2150 9100 50  0001 C CNN
F 1 "GNDREF" H 2155 9177 50  0000 C CNN
F 2 "" H 2150 9350 50  0001 C CNN
F 3 "" H 2150 9350 50  0001 C CNN
	1    2150 9350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R53
U 1 1 5ECC615A
P 6400 7150
F 0 "R53" H 6400 7375 50  0000 C CNN
F 1 "R" H 6400 7284 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 7150 50  0001 C CNN
F 3 "" H 6400 7150 50  0001 C CNN
	1    6400 7150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C35
U 1 1 5ECC6160
P 6200 7250
F 0 "C35" V 6179 7338 50  0000 L CNN
F 1 "C" V 6270 7338 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 7350 50  0001 C CNN
F 3 "" H 6200 7350 50  0001 C CNN
	1    6200 7250
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR035
U 1 1 5ECC6166
P 6200 7400
F 0 "#PWR035" H 6200 7150 50  0001 C CNN
F 1 "GNDREF" H 6205 7227 50  0000 C CNN
F 2 "" H 6200 7400 50  0001 C CNN
F 3 "" H 6200 7400 50  0001 C CNN
	1    6200 7400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R54
U 1 1 5ECCB689
P 6400 7800
F 0 "R54" H 6400 8025 50  0000 C CNN
F 1 "R" H 6400 7934 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 7800 50  0001 C CNN
F 3 "" H 6400 7800 50  0001 C CNN
	1    6400 7800
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C36
U 1 1 5ECCB68F
P 6200 7900
F 0 "C36" V 6179 7988 50  0000 L CNN
F 1 "C" V 6270 7988 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 8000 50  0001 C CNN
F 3 "" H 6200 8000 50  0001 C CNN
	1    6200 7900
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR036
U 1 1 5ECCB695
P 6200 8050
F 0 "#PWR036" H 6200 7800 50  0001 C CNN
F 1 "GNDREF" H 6205 7877 50  0000 C CNN
F 2 "" H 6200 8050 50  0001 C CNN
F 3 "" H 6200 8050 50  0001 C CNN
	1    6200 8050
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R55
U 1 1 5ECD0900
P 6400 8350
F 0 "R55" H 6400 8575 50  0000 C CNN
F 1 "R" H 6400 8484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 8350 50  0001 C CNN
F 3 "" H 6400 8350 50  0001 C CNN
	1    6400 8350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C37
U 1 1 5ECD0906
P 6200 8450
F 0 "C37" V 6179 8538 50  0000 L CNN
F 1 "C" V 6270 8538 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 8550 50  0001 C CNN
F 3 "" H 6200 8550 50  0001 C CNN
	1    6200 8450
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR037
U 1 1 5ECD090C
P 6200 8600
F 0 "#PWR037" H 6200 8350 50  0001 C CNN
F 1 "GNDREF" H 6205 8427 50  0000 C CNN
F 2 "" H 6200 8600 50  0001 C CNN
F 3 "" H 6200 8600 50  0001 C CNN
	1    6200 8600
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R56
U 1 1 5ECD6653
P 6400 8900
F 0 "R56" H 6400 9125 50  0000 C CNN
F 1 "R" H 6400 9034 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 8900 50  0001 C CNN
F 3 "" H 6400 8900 50  0001 C CNN
	1    6400 8900
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C38
U 1 1 5ECD6659
P 6200 9000
F 0 "C38" V 6179 9088 50  0000 L CNN
F 1 "C" V 6270 9088 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 9100 50  0001 C CNN
F 3 "" H 6200 9100 50  0001 C CNN
	1    6200 9000
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR038
U 1 1 5ECD665F
P 6200 9150
F 0 "#PWR038" H 6200 8900 50  0001 C CNN
F 1 "GNDREF" H 6205 8977 50  0000 C CNN
F 2 "" H 6200 9150 50  0001 C CNN
F 3 "" H 6200 9150 50  0001 C CNN
	1    6200 9150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R57
U 1 1 5ECDB6DE
P 6400 9450
F 0 "R57" H 6400 9675 50  0000 C CNN
F 1 "R" H 6400 9584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 9450 50  0001 C CNN
F 3 "" H 6400 9450 50  0001 C CNN
	1    6400 9450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C39
U 1 1 5ECDB6E4
P 6200 9550
F 0 "C39" V 6179 9638 50  0000 L CNN
F 1 "C" V 6270 9638 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 9650 50  0001 C CNN
F 3 "" H 6200 9650 50  0001 C CNN
	1    6200 9550
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR039
U 1 1 5ECDB6EA
P 6200 9700
F 0 "#PWR039" H 6200 9450 50  0001 C CNN
F 1 "GNDREF" H 6205 9527 50  0000 C CNN
F 2 "" H 6200 9700 50  0001 C CNN
F 3 "" H 6200 9700 50  0001 C CNN
	1    6200 9700
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R58
U 1 1 5ECE09BB
P 6400 10000
F 0 "R58" H 6400 10225 50  0000 C CNN
F 1 "R" H 6400 10134 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 10000 50  0001 C CNN
F 3 "" H 6400 10000 50  0001 C CNN
	1    6400 10000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C40
U 1 1 5ECE09C1
P 6200 10100
F 0 "C40" V 6179 10188 50  0000 L CNN
F 1 "C" V 6270 10188 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 10200 50  0001 C CNN
F 3 "" H 6200 10200 50  0001 C CNN
	1    6200 10100
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR040
U 1 1 5ECE09C7
P 6200 10250
F 0 "#PWR040" H 6200 10000 50  0001 C CNN
F 1 "GNDREF" H 6205 10077 50  0000 C CNN
F 2 "" H 6200 10250 50  0001 C CNN
F 3 "" H 6200 10250 50  0001 C CNN
	1    6200 10250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R59
U 1 1 5ECE6440
P 6400 10550
F 0 "R59" H 6400 10775 50  0000 C CNN
F 1 "R" H 6400 10684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 6400 10550 50  0001 C CNN
F 3 "" H 6400 10550 50  0001 C CNN
	1    6400 10550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C41
U 1 1 5ECE6446
P 6200 10650
F 0 "C41" V 6179 10738 50  0000 L CNN
F 1 "C" V 6270 10738 50  0000 L CNN
F 2 "Project Aether footprints:C" H 6200 10750 50  0001 C CNN
F 3 "" H 6200 10750 50  0001 C CNN
	1    6200 10650
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR041
U 1 1 5ECE644C
P 6200 10800
F 0 "#PWR041" H 6200 10550 50  0001 C CNN
F 1 "GNDREF" H 6205 10627 50  0000 C CNN
F 2 "" H 6200 10800 50  0001 C CNN
F 3 "" H 6200 10800 50  0001 C CNN
	1    6200 10800
	1    0    0    -1  
$EndComp
Text HLabel 4650 10350 2    50   Input ~ 0
ADC8
$Comp
L Project-Aether-rescue:R R40
U 1 1 5ED4D862
P 4250 9950
F 0 "R40" H 4250 10175 50  0000 C CNN
F 1 "R" H 4250 10084 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 9950 50  0001 C CNN
F 3 "" H 4250 9950 50  0001 C CNN
	1    4250 9950
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C25
U 1 1 5ED4D868
P 4100 9750
F 0 "C25" H 4125 9975 50  0000 C CNN
F 1 "C" H 4125 9884 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 9850 50  0001 C CNN
F 3 "" H 4100 9850 50  0001 C CNN
	1    4100 9750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R47
U 1 1 5ED4D86E
P 4450 9750
F 0 "R47" H 4450 9975 50  0000 C CNN
F 1 "R" H 4450 9884 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 9750 50  0001 C CNN
F 3 "" H 4450 9750 50  0001 C CNN
	1    4450 9750
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR025
U 1 1 5ED4D874
P 4250 10150
F 0 "#PWR025" H 4250 9900 50  0001 C CNN
F 1 "GNDREF" H 4255 9977 50  0000 C CNN
F 2 "" H 4250 10150 50  0001 C CNN
F 3 "" H 4250 10150 50  0001 C CNN
	1    4250 10150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C32
U 1 1 5ED4D87A
P 4650 9850
F 0 "C32" H 4675 10075 50  0000 C CNN
F 1 "C" H 4675 9984 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 9950 50  0001 C CNN
F 3 "" H 4650 9950 50  0001 C CNN
	1    4650 9850
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR032
U 1 1 5ED4D880
P 4650 10000
F 0 "#PWR032" H 4650 9750 50  0001 C CNN
F 1 "GNDREF" H 4655 9827 50  0000 C CNN
F 2 "" H 4650 10000 50  0001 C CNN
F 3 "" H 4650 10000 50  0001 C CNN
	1    4650 10000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R39
U 1 1 5ED52F39
P 4250 9300
F 0 "R39" H 4250 9525 50  0000 C CNN
F 1 "R" H 4250 9434 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 9300 50  0001 C CNN
F 3 "" H 4250 9300 50  0001 C CNN
	1    4250 9300
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C24
U 1 1 5ED52F3F
P 4100 9100
F 0 "C24" H 4125 9325 50  0000 C CNN
F 1 "C" H 4125 9234 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 9200 50  0001 C CNN
F 3 "" H 4100 9200 50  0001 C CNN
	1    4100 9100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R46
U 1 1 5ED52F45
P 4450 9100
F 0 "R46" H 4450 9325 50  0000 C CNN
F 1 "R" H 4450 9234 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 9100 50  0001 C CNN
F 3 "" H 4450 9100 50  0001 C CNN
	1    4450 9100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR024
U 1 1 5ED52F4B
P 4250 9500
F 0 "#PWR024" H 4250 9250 50  0001 C CNN
F 1 "GNDREF" H 4255 9327 50  0000 C CNN
F 2 "" H 4250 9500 50  0001 C CNN
F 3 "" H 4250 9500 50  0001 C CNN
	1    4250 9500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C31
U 1 1 5ED52F51
P 4650 9200
F 0 "C31" H 4675 9425 50  0000 C CNN
F 1 "C" H 4675 9334 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 9300 50  0001 C CNN
F 3 "" H 4650 9300 50  0001 C CNN
	1    4650 9200
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR031
U 1 1 5ED52F57
P 4650 9350
F 0 "#PWR031" H 4650 9100 50  0001 C CNN
F 1 "GNDREF" H 4655 9177 50  0000 C CNN
F 2 "" H 4650 9350 50  0001 C CNN
F 3 "" H 4650 9350 50  0001 C CNN
	1    4650 9350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R38
U 1 1 5ED58FFC
P 4250 8700
F 0 "R38" H 4250 8925 50  0000 C CNN
F 1 "R" H 4250 8834 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 8700 50  0001 C CNN
F 3 "" H 4250 8700 50  0001 C CNN
	1    4250 8700
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C23
U 1 1 5ED59002
P 4100 8500
F 0 "C23" H 4125 8725 50  0000 C CNN
F 1 "C" H 4125 8634 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 8600 50  0001 C CNN
F 3 "" H 4100 8600 50  0001 C CNN
	1    4100 8500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R45
U 1 1 5ED59008
P 4450 8500
F 0 "R45" H 4450 8725 50  0000 C CNN
F 1 "R" H 4450 8634 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 8500 50  0001 C CNN
F 3 "" H 4450 8500 50  0001 C CNN
	1    4450 8500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR023
U 1 1 5ED5900E
P 4250 8900
F 0 "#PWR023" H 4250 8650 50  0001 C CNN
F 1 "GNDREF" H 4255 8727 50  0000 C CNN
F 2 "" H 4250 8900 50  0001 C CNN
F 3 "" H 4250 8900 50  0001 C CNN
	1    4250 8900
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C30
U 1 1 5ED59014
P 4650 8600
F 0 "C30" H 4675 8825 50  0000 C CNN
F 1 "C" H 4675 8734 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 8700 50  0001 C CNN
F 3 "" H 4650 8700 50  0001 C CNN
	1    4650 8600
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR030
U 1 1 5ED5901A
P 4650 8750
F 0 "#PWR030" H 4650 8500 50  0001 C CNN
F 1 "GNDREF" H 4655 8577 50  0000 C CNN
F 2 "" H 4650 8750 50  0001 C CNN
F 3 "" H 4650 8750 50  0001 C CNN
	1    4650 8750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R37
U 1 1 5ED5ED1B
P 4250 8000
F 0 "R37" H 4250 8225 50  0000 C CNN
F 1 "R" H 4250 8134 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 8000 50  0001 C CNN
F 3 "" H 4250 8000 50  0001 C CNN
	1    4250 8000
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C22
U 1 1 5ED5ED21
P 4100 7800
F 0 "C22" H 4125 8025 50  0000 C CNN
F 1 "C" H 4125 7934 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 7900 50  0001 C CNN
F 3 "" H 4100 7900 50  0001 C CNN
	1    4100 7800
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R44
U 1 1 5ED5ED27
P 4450 7800
F 0 "R44" H 4450 8025 50  0000 C CNN
F 1 "R" H 4450 7934 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 7800 50  0001 C CNN
F 3 "" H 4450 7800 50  0001 C CNN
	1    4450 7800
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR022
U 1 1 5ED5ED2D
P 4250 8200
F 0 "#PWR022" H 4250 7950 50  0001 C CNN
F 1 "GNDREF" H 4255 8027 50  0000 C CNN
F 2 "" H 4250 8200 50  0001 C CNN
F 3 "" H 4250 8200 50  0001 C CNN
	1    4250 8200
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C29
U 1 1 5ED5ED33
P 4650 7900
F 0 "C29" H 4675 8125 50  0000 C CNN
F 1 "C" H 4675 8034 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 8000 50  0001 C CNN
F 3 "" H 4650 8000 50  0001 C CNN
	1    4650 7900
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR029
U 1 1 5ED5ED39
P 4650 8050
F 0 "#PWR029" H 4650 7800 50  0001 C CNN
F 1 "GNDREF" H 4655 7877 50  0000 C CNN
F 2 "" H 4650 8050 50  0001 C CNN
F 3 "" H 4650 8050 50  0001 C CNN
	1    4650 8050
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R36
U 1 1 5ED64968
P 4250 7350
F 0 "R36" H 4250 7575 50  0000 C CNN
F 1 "R" H 4250 7484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 7350 50  0001 C CNN
F 3 "" H 4250 7350 50  0001 C CNN
	1    4250 7350
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C21
U 1 1 5ED6496E
P 4100 7150
F 0 "C21" H 4125 7375 50  0000 C CNN
F 1 "C" H 4125 7284 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 7250 50  0001 C CNN
F 3 "" H 4100 7250 50  0001 C CNN
	1    4100 7150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R43
U 1 1 5ED64974
P 4450 7150
F 0 "R43" H 4450 7375 50  0000 C CNN
F 1 "R" H 4450 7284 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 7150 50  0001 C CNN
F 3 "" H 4450 7150 50  0001 C CNN
	1    4450 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR021
U 1 1 5ED6497A
P 4250 7550
F 0 "#PWR021" H 4250 7300 50  0001 C CNN
F 1 "GNDREF" H 4255 7377 50  0000 C CNN
F 2 "" H 4250 7550 50  0001 C CNN
F 3 "" H 4250 7550 50  0001 C CNN
	1    4250 7550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C28
U 1 1 5ED64980
P 4650 7250
F 0 "C28" H 4675 7475 50  0000 C CNN
F 1 "C" H 4675 7384 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 7350 50  0001 C CNN
F 3 "" H 4650 7350 50  0001 C CNN
	1    4650 7250
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR028
U 1 1 5ED64986
P 4650 7400
F 0 "#PWR028" H 4650 7150 50  0001 C CNN
F 1 "GNDREF" H 4655 7227 50  0000 C CNN
F 2 "" H 4650 7400 50  0001 C CNN
F 3 "" H 4650 7400 50  0001 C CNN
	1    4650 7400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R35
U 1 1 5ED6A535
P 4250 6700
F 0 "R35" H 4250 6925 50  0000 C CNN
F 1 "R" H 4250 6834 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4250 6700 50  0001 C CNN
F 3 "" H 4250 6700 50  0001 C CNN
	1    4250 6700
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C20
U 1 1 5ED6A53B
P 4100 6500
F 0 "C20" H 4125 6725 50  0000 C CNN
F 1 "C" H 4125 6634 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4100 6600 50  0001 C CNN
F 3 "" H 4100 6600 50  0001 C CNN
	1    4100 6500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R42
U 1 1 5ED6A541
P 4450 6500
F 0 "R42" H 4450 6725 50  0000 C CNN
F 1 "R" H 4450 6634 50  0000 C CNN
F 2 "Project Aether footprints:R" H 4450 6500 50  0001 C CNN
F 3 "" H 4450 6500 50  0001 C CNN
	1    4450 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR020
U 1 1 5ED6A547
P 4250 6900
F 0 "#PWR020" H 4250 6650 50  0001 C CNN
F 1 "GNDREF" H 4255 6727 50  0000 C CNN
F 2 "" H 4250 6900 50  0001 C CNN
F 3 "" H 4250 6900 50  0001 C CNN
	1    4250 6900
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C27
U 1 1 5ED6A54D
P 4650 6600
F 0 "C27" H 4675 6825 50  0000 C CNN
F 1 "C" H 4675 6734 50  0000 C CNN
F 2 "Project Aether footprints:C" H 4650 6700 50  0001 C CNN
F 3 "" H 4650 6700 50  0001 C CNN
	1    4650 6600
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR027
U 1 1 5ED6A553
P 4650 6750
F 0 "#PWR027" H 4650 6500 50  0001 C CNN
F 1 "GNDREF" H 4655 6577 50  0000 C CNN
F 2 "" H 4650 6750 50  0001 C CNN
F 3 "" H 4650 6750 50  0001 C CNN
	1    4650 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 4300 12900 4300
Wire Wire Line
	12800 4150 13000 4150
Wire Wire Line
	13000 3450 13000 4150
$Comp
L power:+3.3V #PWR068
U 1 1 5ECEFB2C
P 12500 3600
F 0 "#PWR068" H 12500 3450 50  0001 C CNN
F 1 "+3.3V" H 12515 3773 50  0000 C CNN
F 2 "" H 12500 3600 50  0001 C CNN
F 3 "" H 12500 3600 50  0001 C CNN
	1    12500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	12750 3600 12750 3250
Wire Wire Line
	13100 3050 13200 3050
$Comp
L power:+3.3V #PWR088
U 1 1 5ED4D05F
P 15000 4150
F 0 "#PWR088" H 15000 4000 50  0001 C CNN
F 1 "+3.3V" H 15015 4323 50  0000 C CNN
F 2 "" H 15000 4150 50  0001 C CNN
F 3 "" H 15000 4150 50  0001 C CNN
	1    15000 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	14650 4000 14650 4150
$Comp
L power:+3.3V #PWR072
U 1 1 5EDCFCEA
P 12500 1850
F 0 "#PWR072" H 12500 1700 50  0001 C CNN
F 1 "+3.3V" H 12515 2023 50  0000 C CNN
F 2 "" H 12500 1850 50  0001 C CNN
F 3 "" H 12500 1850 50  0001 C CNN
	1    12500 1850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C58
U 1 1 5EDCFCF0
P 12900 1950
F 0 "C58" V 12879 2038 50  0000 L CNN
F 1 "100n" V 12970 2038 50  0000 L CNN
F 2 "Project Aether footprints:C" H 12900 2050 50  0001 C CNN
F 3 "" H 12900 2050 50  0001 C CNN
	1    12900 1950
	0    1    1    0   
$EndComp
$Comp
L Project-Aether-rescue:C C57
U 1 1 5EDCFCF6
P 12650 1950
F 0 "C57" V 12629 2038 50  0000 L CNN
F 1 "1u" V 12720 2038 50  0000 L CNN
F 2 "Project Aether footprints:C" H 12650 2050 50  0001 C CNN
F 3 "" H 12650 2050 50  0001 C CNN
	1    12650 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	12900 2100 12800 2100
$Comp
L power:GNDREF #PWR074
U 1 1 5EDCFCFE
P 12800 2100
F 0 "#PWR074" H 12800 1850 50  0001 C CNN
F 1 "GNDREF" H 12805 1927 50  0000 C CNN
F 2 "" H 12800 2100 50  0001 C CNN
F 3 "" H 12800 2100 50  0001 C CNN
	1    12800 2100
	1    0    0    -1  
$EndComp
Connection ~ 12800 2100
Wire Wire Line
	12800 2100 12650 2100
Wire Wire Line
	13550 1850 13550 2000
Connection ~ 14850 9150
Wire Wire Line
	14850 9150 14950 9150
Connection ~ 15350 9150
Wire Wire Line
	15350 9150 15450 9150
$Comp
L Project-Aether-rescue:LM317 PS1
U 1 1 5EE08AC6
P 14550 7600
F 0 "PS1" H 14550 7865 50  0000 C CNN
F 1 "LM317" H 14550 7774 50  0000 C CNN
F 2 "Project Aether footprints:LM317" H 14550 7750 50  0001 C CNN
F 3 "" H 14550 7750 50  0001 C CNN
	1    14550 7600
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C64
U 1 1 5EE111BD
P 15250 7700
F 0 "C64" V 15229 7788 50  0000 L CNN
F 1 "C" V 15320 7788 50  0000 L CNN
F 2 "Project Aether footprints:C" H 15250 7800 50  0001 C CNN
F 3 "" H 15250 7800 50  0001 C CNN
	1    15250 7700
	0    1    1    0   
$EndComp
$Comp
L Project-Aether-rescue:C C66
U 1 1 5EE32901
P 15500 7700
F 0 "C66" V 15479 7788 50  0000 L CNN
F 1 "C" V 15570 7788 50  0000 L CNN
F 2 "Project Aether footprints:C" H 15500 7800 50  0001 C CNN
F 3 "" H 15500 7800 50  0001 C CNN
	1    15500 7700
	0    1    1    0   
$EndComp
$Comp
L Project-Aether-rescue:C C60
U 1 1 5EE32E9B
P 14100 7750
F 0 "C60" V 14171 7672 50  0000 R CNN
F 1 "C" V 14080 7672 50  0000 R CNN
F 2 "Project Aether footprints:C" H 14100 7850 50  0001 C CNN
F 3 "" H 14100 7850 50  0001 C CNN
	1    14100 7750
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C59
U 1 1 5EE4372F
P 13850 7700
F 0 "C59" V 13829 7788 50  0000 L CNN
F 1 "C" V 13920 7788 50  0000 L CNN
F 2 "Project Aether footprints:C" H 13850 7800 50  0001 C CNN
F 3 "" H 13850 7800 50  0001 C CNN
	1    13850 7700
	0    1    1    0   
$EndComp
$Comp
L Project-Aether-rescue:R R77
U 1 1 5EE544DF
P 14950 7800
F 0 "R77" V 14904 7888 50  0000 L CNN
F 1 "R" V 14995 7888 50  0000 L CNN
F 2 "Project Aether footprints:R" H 14950 7800 50  0001 C CNN
F 3 "" H 14950 7800 50  0001 C CNN
	1    14950 7800
	0    1    1    0   
$EndComp
Wire Wire Line
	14900 7600 14950 7600
Wire Wire Line
	14950 7600 15250 7600
Connection ~ 14950 7600
Connection ~ 15250 7600
Wire Wire Line
	15250 7600 15500 7600
Wire Wire Line
	13850 7600 14100 7600
Connection ~ 14100 7600
Wire Wire Line
	14100 7600 14200 7600
$Comp
L Project-Aether-rescue:R R76
U 1 1 5EE9163B
P 14550 8200
F 0 "R76" V 14504 8288 50  0000 L CNN
F 1 "R" V 14595 8288 50  0000 L CNN
F 2 "Project Aether footprints:R" H 14550 8200 50  0001 C CNN
F 3 "" H 14550 8200 50  0001 C CNN
	1    14550 8200
	0    1    1    0   
$EndComp
Wire Wire Line
	14550 7900 14550 8000
Wire Wire Line
	14550 8000 14950 8000
Connection ~ 14550 8000
Wire Wire Line
	14550 8400 15250 8400
Wire Wire Line
	15250 7850 15250 8400
Connection ~ 15250 8400
Wire Wire Line
	15250 8400 15500 8400
Wire Wire Line
	14100 7850 14100 8400
Connection ~ 14550 8400
Wire Wire Line
	13850 7850 13850 8400
Wire Wire Line
	13850 8400 14100 8400
Connection ~ 14100 8400
Wire Wire Line
	14100 8400 14550 8400
Wire Wire Line
	15500 7850 15500 8400
$Comp
L power:GNDREF #PWR084
U 1 1 5EF08574
P 14550 8400
F 0 "#PWR084" H 14550 8150 50  0001 C CNN
F 1 "GNDREF" H 14555 8227 50  0000 C CNN
F 2 "" H 14550 8400 50  0001 C CNN
F 3 "" H 14550 8400 50  0001 C CNN
	1    14550 8400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR079
U 1 1 5EF11177
P 13850 7600
F 0 "#PWR079" H 13850 7450 50  0001 C CNN
F 1 "+5V" H 13865 7773 50  0000 C CNN
F 2 "" H 13850 7600 50  0001 C CNN
F 3 "" H 13850 7600 50  0001 C CNN
	1    13850 7600
	1    0    0    -1  
$EndComp
Connection ~ 13850 7600
$Comp
L power:+3.3V #PWR091
U 1 1 5EF13CBA
P 15500 7600
F 0 "#PWR091" H 15500 7450 50  0001 C CNN
F 1 "+3.3V" H 15515 7773 50  0000 C CNN
F 2 "" H 15500 7600 50  0001 C CNN
F 3 "" H 15500 7600 50  0001 C CNN
	1    15500 7600
	1    0    0    -1  
$EndComp
Connection ~ 15500 7600
$Comp
L Project-Aether-rescue:Diode D6
U 1 1 5EF53C90
P 14550 7200
F 0 "D6" H 14550 6985 50  0000 C CNN
F 1 "Diode" H 14550 7076 50  0000 C CNN
F 2 "Project Aether footprints:Diode" H 14550 7100 50  0001 C CNN
F 3 "" H 14550 7100 50  0001 C CNN
	1    14550 7200
	-1   0    0    1   
$EndComp
Wire Wire Line
	14700 7200 14950 7200
Wire Wire Line
	14950 7200 14950 7600
Wire Wire Line
	14400 7200 14100 7200
Wire Wire Line
	14100 7200 14100 7600
Wire Notes Line
	13700 6700 13700 8650
Wire Notes Line
	13700 8650 15750 8650
Wire Notes Line
	15750 8650 15750 6700
Wire Notes Line
	14450 8800 15750 8800
Wire Notes Line
	15750 8800 15750 9800
Wire Notes Line
	15750 9800 14450 9800
Wire Notes Line
	14450 9800 14450 8800
Wire Notes Line
	13700 6700 15750 6700
Text HLabel 15400 6100 2    50   Input ~ 0
PA13
Text HLabel 15400 6000 2    50   Input ~ 0
PA14
Connection ~ 2150 2450
Connection ~ 2150 3000
Connection ~ 2150 3550
Connection ~ 2150 4100
Connection ~ 2150 4650
Connection ~ 2150 5200
Connection ~ 2150 5750
Connection ~ 2150 6300
Connection ~ 2150 6850
Connection ~ 2150 7400
Connection ~ 2150 7950
Connection ~ 2150 8500
Connection ~ 2150 9100
Connection ~ 4250 6500
Connection ~ 4250 7150
Connection ~ 4250 7800
Connection ~ 4250 8500
Connection ~ 4250 9100
Connection ~ 4250 9750
Connection ~ 4250 10350
$Comp
L power:+5V #PWR082
U 1 1 5F0DB69E
P 14350 5700
F 0 "#PWR082" H 14350 5550 50  0001 C CNN
F 1 "+5V" H 14365 5873 50  0000 C CNN
F 2 "" H 14350 5700 50  0001 C CNN
F 3 "" H 14350 5700 50  0001 C CNN
	1    14350 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR075
U 1 1 5F0E4E73
P 13150 6150
F 0 "#PWR075" H 13150 5900 50  0001 C CNN
F 1 "GNDREF" H 13155 5977 50  0000 C CNN
F 2 "" H 13150 6150 50  0001 C CNN
F 3 "" H 13150 6150 50  0001 C CNN
	1    13150 6150
	1    0    0    -1  
$EndComp
Text HLabel 13100 3050 0    50   Input ~ 0
NRST
Text HLabel 11000 1100 2    50   Input ~ 0
NRST
Text HLabel 13100 3350 0    50   Input ~ 0
PA0
Text HLabel 11000 1600 2    50   Input ~ 0
PA0
Wire Wire Line
	14150 4000 14150 4750
Wire Wire Line
	14150 4750 14250 4750
Text HLabel 14250 4750 2    50   Input ~ 0
PB1
Text HLabel 11000 2100 2    50   Input ~ 0
PB1
Text HLabel 2550 9700 2    50   Input ~ 0
PB2
Text HLabel 2550 4650 2    50   Input ~ 0
PB11
Text HLabel 13600 9100 2    50   Input ~ 0
PB11
Text HLabel 13600 8950 2    50   Input ~ 0
PB2
Text HLabel 14250 4600 2    50   Input ~ 0
PB2
Text HLabel 14450 4350 2    50   Input ~ 0
PB11
$Comp
L Project-Aether-rescue:R R14
U 1 1 5F21B7D9
P 1950 9700
F 0 "R14" H 1950 9925 50  0000 C CNN
F 1 "R" H 1950 9834 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 9700 50  0001 C CNN
F 3 "" H 1950 9700 50  0001 C CNN
	1    1950 9700
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R29
U 1 1 5F21B7DF
P 2350 9700
F 0 "R29" H 2350 9925 50  0000 C CNN
F 1 "R" H 2350 9834 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 9700 50  0001 C CNN
F 3 "" H 2350 9700 50  0001 C CNN
	1    2350 9700
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C14
U 1 1 5F21B7E5
P 2150 9800
F 0 "C14" V 2129 9888 50  0000 L CNN
F 1 "C" V 2220 9888 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 9900 50  0001 C CNN
F 3 "" H 2150 9900 50  0001 C CNN
	1    2150 9800
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR014
U 1 1 5F21B7EB
P 2150 9950
F 0 "#PWR014" H 2150 9700 50  0001 C CNN
F 1 "GNDREF" H 2155 9777 50  0000 C CNN
F 2 "" H 2150 9950 50  0001 C CNN
F 3 "" H 2150 9950 50  0001 C CNN
	1    2150 9950
	1    0    0    -1  
$EndComp
Connection ~ 2150 9700
$Comp
L Project-Aether-rescue:R R15
U 1 1 5F22E123
P 1950 10350
F 0 "R15" H 1950 10575 50  0000 C CNN
F 1 "R" H 1950 10484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 1950 10350 50  0001 C CNN
F 3 "" H 1950 10350 50  0001 C CNN
	1    1950 10350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R30
U 1 1 5F22E129
P 2350 10350
F 0 "R30" H 2350 10575 50  0000 C CNN
F 1 "R" H 2350 10484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 2350 10350 50  0001 C CNN
F 3 "" H 2350 10350 50  0001 C CNN
	1    2350 10350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C15
U 1 1 5F22E12F
P 2150 10450
F 0 "C15" V 2129 10538 50  0000 L CNN
F 1 "C" V 2220 10538 50  0000 L CNN
F 2 "Project Aether footprints:C" H 2150 10550 50  0001 C CNN
F 3 "" H 2150 10550 50  0001 C CNN
	1    2150 10450
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR015
U 1 1 5F22E135
P 2150 10600
F 0 "#PWR015" H 2150 10350 50  0001 C CNN
F 1 "GNDREF" H 2155 10427 50  0000 C CNN
F 2 "" H 2150 10600 50  0001 C CNN
F 3 "" H 2150 10600 50  0001 C CNN
	1    2150 10600
	1    0    0    -1  
$EndComp
Connection ~ 2150 10350
Text HLabel 2550 10350 2    50   Input ~ 0
PB1
Wire Notes Line
	750  11100 2950 11100
Wire Notes Line
	2950 2100 2950 11100
Wire Notes Line
	750  2100 750  11100
$Comp
L Project-Aether-rescue:LED D7
U 1 1 5F288FC5
P 15450 7100
F 0 "D7" H 15475 7415 50  0000 C CNN
F 1 "LED" H 15475 7324 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 15450 7250 50  0001 C CNN
F 3 "" H 15450 7250 50  0001 C CNN
	1    15450 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR090
U 1 1 5F2929C6
P 15600 7100
F 0 "#PWR090" H 15600 6850 50  0001 C CNN
F 1 "GNDREF" H 15605 6927 50  0000 C CNN
F 2 "" H 15600 7100 50  0001 C CNN
F 3 "" H 15600 7100 50  0001 C CNN
	1    15600 7100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR087
U 1 1 5F2942CA
P 14950 7100
F 0 "#PWR087" H 14950 6950 50  0001 C CNN
F 1 "+3.3V" H 14965 7273 50  0000 C CNN
F 2 "" H 14950 7100 50  0001 C CNN
F 3 "" H 14950 7100 50  0001 C CNN
	1    14950 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR064
U 1 1 5F2AAFE1
P 10600 2600
F 0 "#PWR064" H 10600 2350 50  0001 C CNN
F 1 "GNDREF" H 10605 2427 50  0000 C CNN
F 2 "" H 10600 2600 50  0001 C CNN
F 3 "" H 10600 2600 50  0001 C CNN
	1    10600 2600
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S4
U 1 1 5F2AAFE7
P 10800 2400
F 0 "S4" H 10800 2475 50  0000 C CNN
F 1 "Button" H 10800 2384 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 2400 50  0001 C CNN
F 3 "" H 10800 2400 50  0001 C CNN
	1    10800 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR065
U 1 1 5F2AAFED
P 10600 3100
F 0 "#PWR065" H 10600 2850 50  0001 C CNN
F 1 "GNDREF" H 10605 2927 50  0000 C CNN
F 2 "" H 10600 3100 50  0001 C CNN
F 3 "" H 10600 3100 50  0001 C CNN
	1    10600 3100
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S5
U 1 1 5F2AAFF3
P 10800 2900
F 0 "S5" H 10800 2975 50  0000 C CNN
F 1 "Button" H 10800 2884 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 2900 50  0001 C CNN
F 3 "" H 10800 2900 50  0001 C CNN
	1    10800 2900
	1    0    0    -1  
$EndComp
Text HLabel 11000 2600 2    50   Input ~ 0
PB3
Text HLabel 11000 3100 2    50   Input ~ 0
PB4
$Comp
L power:GNDREF #PWR066
U 1 1 5F2B41E9
P 10600 3650
F 0 "#PWR066" H 10600 3400 50  0001 C CNN
F 1 "GNDREF" H 10605 3477 50  0000 C CNN
F 2 "" H 10600 3650 50  0001 C CNN
F 3 "" H 10600 3650 50  0001 C CNN
	1    10600 3650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S6
U 1 1 5F2B41EF
P 10800 3450
F 0 "S6" H 10800 3525 50  0000 C CNN
F 1 "Button" H 10800 3434 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 3450 50  0001 C CNN
F 3 "" H 10800 3450 50  0001 C CNN
	1    10800 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR067
U 1 1 5F2B41F5
P 10600 4150
F 0 "#PWR067" H 10600 3900 50  0001 C CNN
F 1 "GNDREF" H 10605 3977 50  0000 C CNN
F 2 "" H 10600 4150 50  0001 C CNN
F 3 "" H 10600 4150 50  0001 C CNN
	1    10600 4150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:Button S7
U 1 1 5F2B41FB
P 10800 3950
F 0 "S7" H 10800 4025 50  0000 C CNN
F 1 "Button" H 10800 3934 50  0000 C CNN
F 2 "Project Aether footprints:Mini Button" H 10800 3950 50  0001 C CNN
F 3 "" H 10800 3950 50  0001 C CNN
	1    10800 3950
	1    0    0    -1  
$EndComp
Text HLabel 11000 3650 2    50   Input ~ 0
PB12
Text HLabel 11000 4150 2    50   Input ~ 0
PB13
Wire Notes Line
	10250 650  10250 4500
Wire Notes Line
	10250 4500 11450 4500
Wire Notes Line
	11450 4500 11450 650 
Wire Notes Line
	10250 650  11450 650 
$Comp
L Project-Aether-rescue:LED D1
U 1 1 5F2DF1A6
P 9600 1050
F 0 "D1" H 9625 1365 50  0000 C CNN
F 1 "LED" H 9625 1274 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 9600 1200 50  0001 C CNN
F 3 "" H 9600 1200 50  0001 C CNN
	1    9600 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR056
U 1 1 5F2E5E00
P 9750 1050
F 0 "#PWR056" H 9750 800 50  0001 C CNN
F 1 "GNDREF" H 9755 877 50  0000 C CNN
F 2 "" H 9750 1050 50  0001 C CNN
F 3 "" H 9750 1050 50  0001 C CNN
	1    9750 1050
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:LED D2
U 1 1 5F2EA223
P 9600 1650
F 0 "D2" H 9625 1965 50  0000 C CNN
F 1 "LED" H 9625 1874 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 9600 1800 50  0001 C CNN
F 3 "" H 9600 1800 50  0001 C CNN
	1    9600 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR057
U 1 1 5F2EA229
P 9750 1650
F 0 "#PWR057" H 9750 1400 50  0001 C CNN
F 1 "GNDREF" H 9755 1477 50  0000 C CNN
F 2 "" H 9750 1650 50  0001 C CNN
F 3 "" H 9750 1650 50  0001 C CNN
	1    9750 1650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:LED D3
U 1 1 5F2ED27E
P 9600 2250
F 0 "D3" H 9625 2565 50  0000 C CNN
F 1 "LED" H 9625 2474 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 9600 2400 50  0001 C CNN
F 3 "" H 9600 2400 50  0001 C CNN
	1    9600 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR058
U 1 1 5F2ED284
P 9750 2250
F 0 "#PWR058" H 9750 2000 50  0001 C CNN
F 1 "GNDREF" H 9755 2077 50  0000 C CNN
F 2 "" H 9750 2250 50  0001 C CNN
F 3 "" H 9750 2250 50  0001 C CNN
	1    9750 2250
	1    0    0    -1  
$EndComp
Text HLabel 9100 1050 0    50   Input ~ 0
PB14
Text HLabel 9100 1650 0    50   Input ~ 0
PB15
Text HLabel 9100 2250 0    50   Input ~ 0
PA8
Text HLabel 9100 3450 0    50   Input ~ 0
PB9
Text HLabel 9100 2850 0    50   Input ~ 0
PB8
$Comp
L Project-Aether-rescue:LED D4
U 1 1 5F2FB8C0
P 9600 2850
F 0 "D4" H 9625 3165 50  0000 C CNN
F 1 "LED" H 9625 3074 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 9600 3000 50  0001 C CNN
F 3 "" H 9600 3000 50  0001 C CNN
	1    9600 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR059
U 1 1 5F2FB8C6
P 9750 2850
F 0 "#PWR059" H 9750 2600 50  0001 C CNN
F 1 "GNDREF" H 9755 2677 50  0000 C CNN
F 2 "" H 9750 2850 50  0001 C CNN
F 3 "" H 9750 2850 50  0001 C CNN
	1    9750 2850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:LED D5
U 1 1 5F2FB8CC
P 9600 3450
F 0 "D5" H 9625 3765 50  0000 C CNN
F 1 "LED" H 9625 3674 50  0000 C CNN
F 2 "Project Aether footprints:LED" H 9600 3600 50  0001 C CNN
F 3 "" H 9600 3600 50  0001 C CNN
	1    9600 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR060
U 1 1 5F2FB8D2
P 9750 3450
F 0 "#PWR060" H 9750 3200 50  0001 C CNN
F 1 "GNDREF" H 9755 3277 50  0000 C CNN
F 2 "" H 9750 3450 50  0001 C CNN
F 3 "" H 9750 3450 50  0001 C CNN
	1    9750 3450
	1    0    0    -1  
$EndComp
Wire Notes Line
	8700 650  10000 650 
Wire Notes Line
	10000 650  10000 3850
Wire Notes Line
	10000 3850 8700 3850
Wire Notes Line
	8700 3850 8700 650 
Wire Notes Line
	15750 650  15750 5050
Wire Notes Line
	15750 5050 11800 5050
Wire Notes Line
	11800 5050 11800 650 
Wire Notes Line
	11800 650  15750 650 
$Comp
L Project-Aether-rescue:61729-0010BLF U1
U 1 1 5F31FA41
P 13650 6000
F 0 "U1" H 13733 6315 50  0000 C CNN
F 1 "61729-0010BLF" H 13733 6224 50  0000 C CNN
F 2 "Project Aether footprints:61729-0010BLF" H 13650 6250 50  0001 C CNN
F 3 "" H 13650 6250 50  0001 C CNN
	1    13650 6000
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:STM32F1-debugger U3
U 1 1 5F33B17F
P 14950 6050
F 0 "U3" H 14975 6475 50  0000 C CNN
F 1 "STM32F1-debugger" H 14975 6384 50  0000 C CNN
F 2 "Project Aether footprints:STM32F1_Debugger" H 14950 6350 50  0001 C CNN
F 3 "" H 14950 6350 50  0001 C CNN
	1    14950 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	13350 6000 13350 6050
Wire Wire Line
	13350 6050 13150 6050
Wire Wire Line
	13150 6050 13150 6150
Connection ~ 13350 6050
Wire Wire Line
	13350 6050 13350 6150
Wire Wire Line
	14000 6150 14550 6150
Wire Wire Line
	14550 6250 14300 6250
$Comp
L power:GNDREF #PWR081
U 1 1 5F3661AA
P 14300 6250
F 0 "#PWR081" H 14300 6000 50  0001 C CNN
F 1 "GNDREF" H 14305 6077 50  0000 C CNN
F 2 "" H 14300 6250 50  0001 C CNN
F 3 "" H 14300 6250 50  0001 C CNN
	1    14300 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	14000 6050 14550 6050
$Comp
L power:+3.3V #PWR080
U 1 1 5F36A4E2
P 14300 6000
F 0 "#PWR080" H 14300 5850 50  0001 C CNN
F 1 "+3.3V" H 14315 6173 50  0000 C CNN
F 2 "" H 14300 6000 50  0001 C CNN
F 3 "" H 14300 6000 50  0001 C CNN
	1    14300 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	14550 5950 14450 5950
Wire Wire Line
	14450 5950 14450 6000
Wire Wire Line
	14450 6000 14300 6000
Wire Wire Line
	14000 5950 14150 5950
Wire Wire Line
	14150 5950 14150 5700
Wire Wire Line
	14150 5700 14350 5700
Wire Wire Line
	14550 5700 14550 5850
Connection ~ 14350 5700
Wire Wire Line
	14350 5700 14550 5700
Text HLabel 15400 5900 2    50   Input ~ 0
NRST
Connection ~ 14300 6250
Wire Wire Line
	14300 6250 14000 6250
$Comp
L Project-Aether-rescue:3-pinTerminal J_B5
U 1 1 5F48E489
P 1150 10050
F 0 "J_B5" H 1183 10315 50  0000 C CNN
F 1 "3-pinTerminal" H 1183 10224 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 1150 10200 50  0001 C CNN
F 3 "" H 1150 10200 50  0001 C CNN
	1    1150 10050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 9700 1750 10200
Wire Wire Line
	1650 9100 1750 9100
Wire Wire Line
	1750 10350 1300 10350
Wire Wire Line
	1750 10200 1300 10200
Wire Wire Line
	1300 10050 1650 10050
Wire Wire Line
	1650 10050 1650 9100
$Comp
L Project-Aether-rescue:3-pinTerminal J_B4
U 1 1 5F4B73AD
P 1150 8200
F 0 "J_B4" H 1183 8465 50  0000 C CNN
F 1 "3-pinTerminal" H 1183 8374 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 1150 8350 50  0001 C CNN
F 3 "" H 1150 8350 50  0001 C CNN
	1    1150 8200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 8500 1750 8500
Wire Wire Line
	1300 8350 1750 8350
Wire Wire Line
	1750 8350 1750 7950
Wire Wire Line
	1300 8200 1650 8200
Wire Wire Line
	1650 8200 1650 7400
Wire Wire Line
	1650 7400 1750 7400
$Comp
L Project-Aether-rescue:3-pinTerminal J_B3
U 1 1 5F4D112F
P 1150 6550
F 0 "J_B3" H 1183 6815 50  0000 C CNN
F 1 "3-pinTerminal" H 1183 6724 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 1150 6700 50  0001 C CNN
F 3 "" H 1150 6700 50  0001 C CNN
	1    1150 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6850 1750 6850
Wire Wire Line
	1300 6700 1750 6700
Wire Wire Line
	1750 6700 1750 6300
Wire Wire Line
	1300 6550 1650 6550
Wire Wire Line
	1650 6550 1650 5750
Wire Wire Line
	1650 5750 1750 5750
$Comp
L Project-Aether-rescue:3-pinTerminal J_B2
U 1 1 5F4EBB0B
P 1150 4900
F 0 "J_B2" H 1183 5165 50  0000 C CNN
F 1 "3-pinTerminal" H 1183 5074 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 1150 5050 50  0001 C CNN
F 3 "" H 1150 5050 50  0001 C CNN
	1    1150 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5200 1750 5200
Wire Wire Line
	1300 5050 1750 5050
Wire Wire Line
	1750 5050 1750 4650
Wire Wire Line
	1300 4900 1650 4900
Wire Wire Line
	1650 4900 1650 4100
Wire Wire Line
	1650 4100 1750 4100
$Comp
L Project-Aether-rescue:3-pinTerminal J_B1
U 1 1 5F5024B7
P 1150 3250
F 0 "J_B1" H 1183 3515 50  0000 C CNN
F 1 "3-pinTerminal" H 1183 3424 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 1150 3400 50  0001 C CNN
F 3 "" H 1150 3400 50  0001 C CNN
	1    1150 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3550 1750 3550
Wire Wire Line
	1300 3400 1750 3400
Wire Wire Line
	1750 3400 1750 3000
Wire Wire Line
	1300 3250 1650 3250
Wire Wire Line
	1650 3250 1650 2450
Wire Wire Line
	1650 2450 1750 2450
$Comp
L Project-Aether-rescue:2-pinTerminal J_F1
U 1 1 5F5596D3
P 3900 4200
F 0 "J_F1" H 3933 4515 50  0000 C CNN
F 1 "2-pinTerminal" H 3933 4424 50  0000 C CNN
F 2 "Project Aether footprints:2-pin Terminal" H 3900 4400 50  0001 C CNN
F 3 "" H 3900 4400 50  0001 C CNN
	1    3900 4200
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J_ADC1
U 1 1 5F5A70E1
P 3500 7500
F 0 "J_ADC1" H 3533 7765 50  0000 C CNN
F 1 "3-pinTerminal" H 3533 7674 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 3500 7650 50  0001 C CNN
F 3 "" H 3500 7650 50  0001 C CNN
	1    3500 7500
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:2-pinTerminal J_ADC3
U 1 1 5F5A9075
P 3500 10250
F 0 "J_ADC3" H 3533 10565 50  0000 C CNN
F 1 "2-pinTerminal" H 3533 10474 50  0000 C CNN
F 2 "Project Aether footprints:2-pin Terminal" H 3500 10450 50  0001 C CNN
F 3 "" H 3500 10450 50  0001 C CNN
	1    3500 10250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:2-pinTerminal J_ADC2
U 1 1 5F5AAF1C
P 3500 9000
F 0 "J_ADC2" H 3533 9315 50  0000 C CNN
F 1 "2-pinTerminal" H 3533 9224 50  0000 C CNN
F 2 "Project Aether footprints:2-pin Terminal" H 3500 9200 50  0001 C CNN
F 3 "" H 3500 9200 50  0001 C CNN
	1    3500 9000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 10350 3650 10350
Wire Wire Line
	3650 10200 3950 10200
Wire Wire Line
	3950 10200 3950 9750
Wire Wire Line
	3950 9750 4000 9750
Wire Wire Line
	3650 9100 4000 9100
Wire Wire Line
	3650 8950 3950 8950
Wire Wire Line
	3950 8950 3950 8500
Wire Wire Line
	3950 8500 4000 8500
Wire Wire Line
	3650 7800 4000 7800
Wire Wire Line
	3650 7650 3950 7650
Wire Wire Line
	3950 7650 3950 7150
Wire Wire Line
	3950 7150 4000 7150
Wire Wire Line
	3650 7500 3850 7500
Wire Wire Line
	3850 7500 3850 6500
Wire Wire Line
	3850 6500 4000 6500
$Comp
L Project-Aether-rescue:3-pinTerminal J_PA3
U 1 1 5F66DE65
P 5500 10250
F 0 "J_PA3" H 5533 10515 50  0000 C CNN
F 1 "3-pinTerminal" H 5533 10424 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 5500 10400 50  0001 C CNN
F 3 "" H 5500 10400 50  0001 C CNN
	1    5500 10250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 10550 6200 10550
Connection ~ 6200 10550
Wire Wire Line
	5650 10400 6000 10400
Wire Wire Line
	6000 10400 6000 10000
Wire Wire Line
	6000 10000 6200 10000
Connection ~ 6200 10000
Wire Wire Line
	5650 10250 5900 10250
Wire Wire Line
	5900 10250 5900 9450
Wire Wire Line
	5900 9450 6200 9450
Connection ~ 6200 9450
$Comp
L Project-Aether-rescue:3-pinTerminal J_PA2
U 1 1 5F696A0C
P 5500 8600
F 0 "J_PA2" H 5533 8865 50  0000 C CNN
F 1 "3-pinTerminal" H 5533 8774 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 5500 8750 50  0001 C CNN
F 3 "" H 5500 8750 50  0001 C CNN
	1    5500 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 8900 6200 8900
Wire Wire Line
	5650 8750 6000 8750
Wire Wire Line
	5650 8600 5900 8600
Wire Wire Line
	6000 8750 6000 8350
Wire Wire Line
	6000 8350 6200 8350
Connection ~ 6200 8350
Wire Wire Line
	5900 8600 5900 7800
Wire Wire Line
	5900 7800 6200 7800
Connection ~ 6200 7800
$Comp
L Project-Aether-rescue:2-pinTerminal J_PA1
U 1 1 5F6B378B
P 5500 7050
F 0 "J_PA1" H 5533 7365 50  0000 C CNN
F 1 "2-pinTerminal" H 5533 7274 50  0000 C CNN
F 2 "Project Aether footprints:2-pin Terminal" H 5500 7250 50  0001 C CNN
F 3 "" H 5500 7250 50  0001 C CNN
	1    5500 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 7150 5650 7150
Connection ~ 6200 7150
Wire Wire Line
	5650 7000 5950 7000
Wire Wire Line
	5950 7000 5950 6600
Wire Wire Line
	5950 6600 6200 6600
Connection ~ 6200 6600
Connection ~ 6200 8900
Wire Wire Line
	13600 9350 13600 9500
Connection ~ 13600 9500
$Comp
L power:+3.3V #PWR054
U 1 1 5F99E74B
P 8550 10700
F 0 "#PWR054" H 8550 10550 50  0001 C CNN
F 1 "+3.3V" H 8565 10873 50  0000 C CNN
F 2 "" H 8550 10700 50  0001 C CNN
F 3 "" H 8550 10700 50  0001 C CNN
	1    8550 10700
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR055
U 1 1 5F9DE33C
P 9150 10750
F 0 "#PWR055" H 9150 10600 50  0001 C CNN
F 1 "+5V" H 9165 10923 50  0000 C CNN
F 2 "" H 9150 10750 50  0001 C CNN
F 3 "" H 9150 10750 50  0001 C CNN
	1    9150 10750
	-1   0    0    1   
$EndComp
Wire Wire Line
	9150 10700 9150 10750
Wire Notes Line
	9350 9200 9350 11100
Wire Notes Line
	9350 11100 7400 11100
Wire Notes Line
	7400 11100 7400 9200
Wire Notes Line
	7400 9200 9350 9200
Wire Wire Line
	12500 1850 12650 1850
Connection ~ 12900 1850
Wire Wire Line
	12900 1850 13550 1850
Connection ~ 12650 1850
Wire Wire Line
	12650 1850 12900 1850
Wire Wire Line
	14650 4150 15000 4150
Text Notes 14750 8600 0    50   ~ 10
Voltage Regulator Circuit
Text Notes 15000 4950 0    50   ~ 10
Microcontroller
Text Notes 8700 11050 0    50   ~ 10
Supply Bus\n
$Comp
L Project-Aether-rescue:3-pinTerminal J_GND2
U 1 1 5FBD570D
P 7700 10400
F 0 "J_GND2" H 7733 10665 50  0000 C CNN
F 1 "3-pinTerminal" H 7733 10574 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 7700 10550 50  0001 C CNN
F 3 "" H 7700 10550 50  0001 C CNN
	1    7700 10400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J_GND1
U 1 1 5FBD88D0
P 7700 9650
F 0 "J_GND1" H 7733 9915 50  0000 C CNN
F 1 "3-pinTerminal" H 7733 9824 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 7700 9800 50  0001 C CNN
F 3 "" H 7700 9800 50  0001 C CNN
	1    7700 9650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J_3V2
U 1 1 5FBF277C
P 8300 10400
F 0 "J_3V2" H 8333 10665 50  0000 C CNN
F 1 "3-pinTerminal" H 8333 10574 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 8300 10550 50  0001 C CNN
F 3 "" H 8300 10550 50  0001 C CNN
	1    8300 10400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J_3V1
U 1 1 5FBF2782
P 8300 9650
F 0 "J_3V1" H 8333 9915 50  0000 C CNN
F 1 "3-pinTerminal" H 8333 9824 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 8300 9800 50  0001 C CNN
F 3 "" H 8300 9800 50  0001 C CNN
	1    8300 9650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J-5V2
U 1 1 5FBFE137
P 8900 10400
F 0 "J-5V2" H 8933 10665 50  0000 C CNN
F 1 "3-pinTerminal" H 8933 10574 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 8900 10550 50  0001 C CNN
F 3 "" H 8900 10550 50  0001 C CNN
	1    8900 10400
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:3-pinTerminal J_5V1
U 1 1 5FBFE13D
P 8900 9650
F 0 "J_5V1" H 8933 9915 50  0000 C CNN
F 1 "3-pinTerminal" H 8933 9824 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 8900 9800 50  0001 C CNN
F 3 "" H 8900 9800 50  0001 C CNN
	1    8900 9650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 9650 7850 9800
Connection ~ 7850 9800
Wire Wire Line
	7850 9800 7850 9950
Wire Wire Line
	7850 9950 7850 10400
Connection ~ 7850 9950
Wire Wire Line
	7850 10400 7850 10550
Connection ~ 7850 10400
Connection ~ 7850 10550
Wire Wire Line
	7850 10550 7850 10700
Wire Wire Line
	8450 10700 8450 10550
Connection ~ 8450 9800
Wire Wire Line
	8450 9800 8450 9650
Connection ~ 8450 9950
Wire Wire Line
	8450 9950 8450 9800
Connection ~ 8450 10400
Wire Wire Line
	8450 10400 8450 9950
Connection ~ 8450 10550
Wire Wire Line
	8450 10550 8450 10400
Wire Wire Line
	9050 9650 9050 9800
Connection ~ 9050 9800
Wire Wire Line
	9050 9800 9050 9950
Connection ~ 9050 9950
Wire Wire Line
	9050 9950 9050 10400
Connection ~ 9050 10400
Wire Wire Line
	9050 10400 9050 10550
Connection ~ 9050 10550
Wire Wire Line
	9050 10550 9050 10700
Wire Wire Line
	7900 10700 7900 10750
$Comp
L power:GNDREF #PWR045
U 1 1 5F978506
P 7900 10750
F 0 "#PWR045" H 7900 10500 50  0001 C CNN
F 1 "GNDREF" H 7905 10577 50  0000 C CNN
F 2 "" H 7900 10750 50  0001 C CNN
F 3 "" H 7900 10750 50  0001 C CNN
	1    7900 10750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 10700 9050 10700
Connection ~ 9050 10700
Wire Wire Line
	8550 10700 8450 10700
Connection ~ 8450 10700
Wire Wire Line
	7900 10700 7850 10700
Connection ~ 7850 10700
Text Notes 4550 11050 0    50   ~ 10
ADC Bus\n
Text Notes 6800 11050 0    50   ~ 10
PA Bus\n
Text Notes 2550 11050 0    50   ~ 10
PB Bus\n
Text Notes 4300 5200 0    50   ~ 10
PF Bus\n
Text Notes 6750 6100 0    50   ~ 10
PC Bus
Text Notes 11100 4450 0    50   ~ 10
Buttons\n
Text Notes 9750 3800 0    50   ~ 10
LEDs\n
Text Notes 15400 9750 0    50   ~ 10
Clock\n
Text Notes 13750 9850 0    50   ~ 10
Motor Driver
Text HLabel 8900 5550 2    50   Input ~ 0
ADC5
Text HLabel 8900 6450 2    50   Input ~ 0
ADC6
Text HLabel 8900 7350 2    50   Input ~ 0
ADC7
Text HLabel 8900 8250 2    50   Input ~ 0
ADC8
$Comp
L Project-Aether-rescue:R R67
U 1 1 5FDD1591
P 8500 5750
F 0 "R67" H 8500 5975 50  0000 C CNN
F 1 "R" H 8500 5884 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8500 5750 50  0001 C CNN
F 3 "" H 8500 5750 50  0001 C CNN
	1    8500 5750
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C45
U 1 1 5FDD1597
P 8350 5550
F 0 "C45" H 8375 5775 50  0000 C CNN
F 1 "C" H 8375 5684 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8350 5650 50  0001 C CNN
F 3 "" H 8350 5650 50  0001 C CNN
	1    8350 5550
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R71
U 1 1 5FDD159D
P 8700 5550
F 0 "R71" H 8700 5775 50  0000 C CNN
F 1 "R" H 8700 5684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8700 5550 50  0001 C CNN
F 3 "" H 8700 5550 50  0001 C CNN
	1    8700 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR050
U 1 1 5FDD15A3
P 8500 5950
F 0 "#PWR050" H 8500 5700 50  0001 C CNN
F 1 "GNDREF" H 8505 5777 50  0000 C CNN
F 2 "" H 8500 5950 50  0001 C CNN
F 3 "" H 8500 5950 50  0001 C CNN
	1    8500 5950
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C49
U 1 1 5FDD15A9
P 8900 5650
F 0 "C49" H 8925 5875 50  0000 C CNN
F 1 "C" H 8925 5784 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8900 5750 50  0001 C CNN
F 3 "" H 8900 5750 50  0001 C CNN
	1    8900 5650
	0    1    1    0   
$EndComp
Connection ~ 8500 5550
$Comp
L Project-Aether-rescue:Potentiometer R63
U 1 1 5FE0DC62
P 8050 5550
F 0 "R63" H 7963 5596 50  0000 R CNN
F 1 "Potentiometer" H 7963 5505 50  0000 R CNN
F 2 "Project Aether footprints:Potentiometers" H 8050 5850 50  0001 C CNN
F 3 "" H 8050 5850 50  0001 C CNN
	1    8050 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 5750 8050 5950
Wire Wire Line
	8050 5950 8500 5950
Connection ~ 8500 5950
Wire Wire Line
	8900 5800 8900 5950
Wire Wire Line
	8900 5950 8500 5950
$Comp
L power:+3.3V #PWR046
U 1 1 5FEBAFCD
P 8050 5350
F 0 "#PWR046" H 8050 5200 50  0001 C CNN
F 1 "+3.3V" H 8065 5523 50  0000 C CNN
F 2 "" H 8050 5350 50  0001 C CNN
F 3 "" H 8050 5350 50  0001 C CNN
	1    8050 5350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R68
U 1 1 5FF18E6E
P 8500 6650
F 0 "R68" H 8500 6875 50  0000 C CNN
F 1 "R" H 8500 6784 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8500 6650 50  0001 C CNN
F 3 "" H 8500 6650 50  0001 C CNN
	1    8500 6650
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C46
U 1 1 5FF18E74
P 8350 6450
F 0 "C46" H 8375 6675 50  0000 C CNN
F 1 "C" H 8375 6584 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8350 6550 50  0001 C CNN
F 3 "" H 8350 6550 50  0001 C CNN
	1    8350 6450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R72
U 1 1 5FF18E7A
P 8700 6450
F 0 "R72" H 8700 6675 50  0000 C CNN
F 1 "R" H 8700 6584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8700 6450 50  0001 C CNN
F 3 "" H 8700 6450 50  0001 C CNN
	1    8700 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR051
U 1 1 5FF18E80
P 8500 6850
F 0 "#PWR051" H 8500 6600 50  0001 C CNN
F 1 "GNDREF" H 8505 6677 50  0000 C CNN
F 2 "" H 8500 6850 50  0001 C CNN
F 3 "" H 8500 6850 50  0001 C CNN
	1    8500 6850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C50
U 1 1 5FF18E86
P 8900 6550
F 0 "C50" H 8925 6775 50  0000 C CNN
F 1 "C" H 8925 6684 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8900 6650 50  0001 C CNN
F 3 "" H 8900 6650 50  0001 C CNN
	1    8900 6550
	0    1    1    0   
$EndComp
Connection ~ 8500 6450
$Comp
L Project-Aether-rescue:Potentiometer R64
U 1 1 5FF18E8D
P 8050 6450
F 0 "R64" H 7963 6496 50  0000 R CNN
F 1 "Potentiometer" H 7963 6405 50  0000 R CNN
F 2 "Project Aether footprints:Potentiometers" H 8050 6750 50  0001 C CNN
F 3 "" H 8050 6750 50  0001 C CNN
	1    8050 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 6650 8050 6850
Wire Wire Line
	8050 6850 8500 6850
Connection ~ 8500 6850
Wire Wire Line
	8900 6700 8900 6850
Wire Wire Line
	8900 6850 8500 6850
$Comp
L power:+3.3V #PWR047
U 1 1 5FF18E98
P 8050 6250
F 0 "#PWR047" H 8050 6100 50  0001 C CNN
F 1 "+3.3V" H 8065 6423 50  0000 C CNN
F 2 "" H 8050 6250 50  0001 C CNN
F 3 "" H 8050 6250 50  0001 C CNN
	1    8050 6250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R69
U 1 1 5FF260DB
P 8500 7550
F 0 "R69" H 8500 7775 50  0000 C CNN
F 1 "R" H 8500 7684 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8500 7550 50  0001 C CNN
F 3 "" H 8500 7550 50  0001 C CNN
	1    8500 7550
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C47
U 1 1 5FF260E1
P 8350 7350
F 0 "C47" H 8375 7575 50  0000 C CNN
F 1 "C" H 8375 7484 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8350 7450 50  0001 C CNN
F 3 "" H 8350 7450 50  0001 C CNN
	1    8350 7350
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R73
U 1 1 5FF260E7
P 8700 7350
F 0 "R73" H 8700 7575 50  0000 C CNN
F 1 "R" H 8700 7484 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8700 7350 50  0001 C CNN
F 3 "" H 8700 7350 50  0001 C CNN
	1    8700 7350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR052
U 1 1 5FF260ED
P 8500 7750
F 0 "#PWR052" H 8500 7500 50  0001 C CNN
F 1 "GNDREF" H 8505 7577 50  0000 C CNN
F 2 "" H 8500 7750 50  0001 C CNN
F 3 "" H 8500 7750 50  0001 C CNN
	1    8500 7750
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C51
U 1 1 5FF260F3
P 8900 7450
F 0 "C51" H 8925 7675 50  0000 C CNN
F 1 "C" H 8925 7584 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8900 7550 50  0001 C CNN
F 3 "" H 8900 7550 50  0001 C CNN
	1    8900 7450
	0    1    1    0   
$EndComp
Connection ~ 8500 7350
$Comp
L Project-Aether-rescue:Potentiometer R65
U 1 1 5FF260FA
P 8050 7350
F 0 "R65" H 7963 7396 50  0000 R CNN
F 1 "Potentiometer" H 7963 7305 50  0000 R CNN
F 2 "Project Aether footprints:Potentiometers" H 8050 7650 50  0001 C CNN
F 3 "" H 8050 7650 50  0001 C CNN
	1    8050 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 7550 8050 7750
Wire Wire Line
	8050 7750 8500 7750
Connection ~ 8500 7750
Wire Wire Line
	8900 7600 8900 7750
Wire Wire Line
	8900 7750 8500 7750
$Comp
L power:+3.3V #PWR048
U 1 1 5FF26105
P 8050 7150
F 0 "#PWR048" H 8050 7000 50  0001 C CNN
F 1 "+3.3V" H 8065 7323 50  0000 C CNN
F 2 "" H 8050 7150 50  0001 C CNN
F 3 "" H 8050 7150 50  0001 C CNN
	1    8050 7150
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R70
U 1 1 5FF33607
P 8500 8450
F 0 "R70" H 8500 8675 50  0000 C CNN
F 1 "R" H 8500 8584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8500 8450 50  0001 C CNN
F 3 "" H 8500 8450 50  0001 C CNN
	1    8500 8450
	0    -1   -1   0   
$EndComp
$Comp
L Project-Aether-rescue:C C48
U 1 1 5FF3360D
P 8350 8250
F 0 "C48" H 8375 8475 50  0000 C CNN
F 1 "C" H 8375 8384 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8350 8350 50  0001 C CNN
F 3 "" H 8350 8350 50  0001 C CNN
	1    8350 8250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R74
U 1 1 5FF33613
P 8700 8250
F 0 "R74" H 8700 8475 50  0000 C CNN
F 1 "R" H 8700 8384 50  0000 C CNN
F 2 "Project Aether footprints:R" H 8700 8250 50  0001 C CNN
F 3 "" H 8700 8250 50  0001 C CNN
	1    8700 8250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR053
U 1 1 5FF33619
P 8500 8650
F 0 "#PWR053" H 8500 8400 50  0001 C CNN
F 1 "GNDREF" H 8505 8477 50  0000 C CNN
F 2 "" H 8500 8650 50  0001 C CNN
F 3 "" H 8500 8650 50  0001 C CNN
	1    8500 8650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:C C52
U 1 1 5FF3361F
P 8900 8350
F 0 "C52" H 8925 8575 50  0000 C CNN
F 1 "C" H 8925 8484 50  0000 C CNN
F 2 "Project Aether footprints:C" H 8900 8450 50  0001 C CNN
F 3 "" H 8900 8450 50  0001 C CNN
	1    8900 8350
	0    1    1    0   
$EndComp
Connection ~ 8500 8250
$Comp
L Project-Aether-rescue:Potentiometer R66
U 1 1 5FF33626
P 8050 8250
F 0 "R66" H 7963 8296 50  0000 R CNN
F 1 "Potentiometer" H 7963 8205 50  0000 R CNN
F 2 "Project Aether footprints:Potentiometers" H 8050 8550 50  0001 C CNN
F 3 "" H 8050 8550 50  0001 C CNN
	1    8050 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 8450 8050 8650
Wire Wire Line
	8050 8650 8500 8650
Connection ~ 8500 8650
Wire Wire Line
	8900 8500 8900 8650
Wire Wire Line
	8900 8650 8500 8650
$Comp
L power:+3.3V #PWR049
U 1 1 5FF33631
P 8050 8050
F 0 "#PWR049" H 8050 7900 50  0001 C CNN
F 1 "+3.3V" H 8065 8223 50  0000 C CNN
F 2 "" H 8050 8050 50  0001 C CNN
F 3 "" H 8050 8050 50  0001 C CNN
	1    8050 8050
	1    0    0    -1  
$EndComp
Wire Notes Line
	7400 5050 9250 5050
Wire Notes Line
	9250 5050 9250 9000
Wire Notes Line
	9250 9000 7400 9000
Text Notes 8600 8950 0    50   ~ 10
Potentiometers\n
Wire Notes Line
	7400 9000 7400 5050
$Comp
L Project-Aether-rescue:3-pinTerminal J_PC1
U 1 1 5ECBA41A
P 6100 5100
F 0 "J_PC1" H 6133 5365 50  0000 C CNN
F 1 "3-pinTerminal" H 6133 5274 50  0000 C CNN
F 2 "Project Aether footprints:3-pin Terminal" H 6100 5250 50  0001 C CNN
F 3 "" H 6100 5250 50  0001 C CNN
	1    6100 5100
	1    0    0    -1  
$EndComp
Text Notes 14750 6550 0    50   ~ 10
Programmer Circuit
Wire Notes Line
	12900 6600 15700 6600
Wire Notes Line
	15700 6600 15700 5350
Wire Notes Line
	15700 5350 12900 5350
Wire Notes Line
	12900 5350 12900 6600
$Comp
L Project-Aether-rescue:R R78
U 1 1 5EE7D3E3
P 9300 1050
F 0 "R78" H 9300 1275 50  0000 C CNN
F 1 "100" H 9300 1184 50  0000 C CNN
F 2 "Project Aether footprints:R" H 9300 1050 50  0001 C CNN
F 3 "" H 9300 1050 50  0001 C CNN
	1    9300 1050
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R79
U 1 1 5EE99990
P 9300 1650
F 0 "R79" H 9300 1875 50  0000 C CNN
F 1 "100" H 9300 1784 50  0000 C CNN
F 2 "Project Aether footprints:R" H 9300 1650 50  0001 C CNN
F 3 "" H 9300 1650 50  0001 C CNN
	1    9300 1650
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R80
U 1 1 5EEA6798
P 9300 2250
F 0 "R80" H 9300 2475 50  0000 C CNN
F 1 "100" H 9300 2384 50  0000 C CNN
F 2 "Project Aether footprints:R" H 9300 2250 50  0001 C CNN
F 3 "" H 9300 2250 50  0001 C CNN
	1    9300 2250
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R81
U 1 1 5EEB35E9
P 9300 2850
F 0 "R81" H 9300 3075 50  0000 C CNN
F 1 "100" H 9300 2984 50  0000 C CNN
F 2 "Project Aether footprints:R" H 9300 2850 50  0001 C CNN
F 3 "" H 9300 2850 50  0001 C CNN
	1    9300 2850
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R82
U 1 1 5EEDC910
P 9300 3450
F 0 "R82" H 9300 3675 50  0000 C CNN
F 1 "100" H 9300 3584 50  0000 C CNN
F 2 "Project Aether footprints:R" H 9300 3450 50  0001 C CNN
F 3 "" H 9300 3450 50  0001 C CNN
	1    9300 3450
	1    0    0    -1  
$EndComp
$Comp
L Project-Aether-rescue:R R83
U 1 1 5EC8F466
P 15150 7100
F 0 "R83" V 15104 7188 50  0000 L CNN
F 1 "R" V 15195 7188 50  0000 L CNN
F 2 "Project Aether footprints:R" H 15150 7100 50  0001 C CNN
F 3 "" H 15150 7100 50  0001 C CNN
	1    15150 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	12500 3600 12750 3600
Wire Wire Line
	12500 2450 13100 2450
Text HLabel 6250 5250 2    50   Input ~ 0
PC14
$Comp
L Project-Aether-rescue:2-pinTerminal J_F2
U 1 1 5F557B22
P 3900 4900
F 0 "J_F2" H 3933 5215 50  0000 C CNN
F 1 "2-pinTerminal" H 3933 5124 50  0000 C CNN
F 2 "Project Aether footprints:2-pin Terminal" H 3900 5100 50  0001 C CNN
F 3 "" H 3900 5100 50  0001 C CNN
	1    3900 4900
	1    0    0    -1  
$EndComp
Wire Notes Line
	3200 5900 4750 5900
$EndSCHEMATC
