EESchema Schematic File Version 4
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
L otter:C302_Printhead U1
U 1 1 5C7546DF
P 3250 3900
F 0 "U1" H 3250 4447 60  0000 C CNN
F 1 "C302_Printhead" H 3250 4341 60  0000 C CNN
F 2 "otter:C302_Printhead" H 3250 4950 60  0001 C CNN
F 3 "" H 3250 4950 60  0001 C CNN
	1    3250 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5C75474C
P 2700 3600
F 0 "#PWR0101" H 2700 3350 50  0001 C CNN
F 1 "GND" V 2705 3472 50  0000 R CNN
F 2 "" H 2700 3600 50  0001 C CNN
F 3 "" H 2700 3600 50  0001 C CNN
	1    2700 3600
	0    1    1    0   
$EndComp
Text GLabel 2700 3750 0    50   Input ~ 0
F3_HV
Text GLabel 2700 3850 0    50   Input ~ 0
F5_HV
Text GLabel 2700 4000 0    50   Input ~ 0
CSYNC_HV
Text GLabel 2700 4100 0    50   Input ~ 0
DCLK_HV
Text GLabel 3800 3600 2    50   Input ~ 0
S5
Text GLabel 3800 3700 2    50   Input ~ 0
S4
Text GLabel 3800 3800 2    50   Input ~ 0
S3
Text GLabel 3800 4150 2    50   Input ~ 0
D3
Text GLabel 3800 4250 2    50   Input ~ 0
D2
Text GLabel 3800 4350 2    50   Input ~ 0
D1
Text GLabel 3800 3900 2    50   Input ~ 0
S2
Text GLabel 3800 4000 2    50   Input ~ 0
S1
NoConn ~ 2700 4250
NoConn ~ 2700 4350
$Comp
L power:GND #PWR0102
U 1 1 5C754AFF
P 4950 3200
F 0 "#PWR0102" H 4950 2950 50  0001 C CNN
F 1 "GND" V 4955 3072 50  0000 R CNN
F 2 "" H 4950 3200 50  0001 C CNN
F 3 "" H 4950 3200 50  0001 C CNN
	1    4950 3200
	0    1    1    0   
$EndComp
Text GLabel 4950 4200 0    50   Input ~ 0
S5
Text GLabel 4950 4400 0    50   Input ~ 0
S4
Text GLabel 4950 3700 0    50   Input ~ 0
S3
Text GLabel 4950 4100 0    50   Input ~ 0
D3
Text GLabel 4950 3400 0    50   Input ~ 0
D2
Text GLabel 4950 3600 0    50   Input ~ 0
D1
Text GLabel 4950 4500 0    50   Input ~ 0
S2
Text GLabel 4950 4300 0    50   Input ~ 0
S1
Text GLabel 4950 3300 0    50   Input ~ 0
F3_HV
Text GLabel 4950 3900 0    50   Input ~ 0
F5_HV
Text GLabel 4950 4000 0    50   Input ~ 0
CSYNC_HV
Text GLabel 4950 3800 0    50   Input ~ 0
DCLK_HV
NoConn ~ 4950 3500
NoConn ~ 4950 4600
$Comp
L Connector_Generic:Conn_01x16 J1
U 1 1 5C754E31
P 5150 4000
F 0 "J1" H 5069 2975 50  0000 C CNN
F 1 "Conn_01x16" H 5069 3066 50  0000 C CNN
F 2 "otter:Generic_FPC_0.5mm_15pin" H 5150 4000 50  0001 C CNN
F 3 "~" H 5150 4000 50  0001 C CNN
	1    5150 4000
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C754EAC
P 4950 4700
F 0 "#PWR0103" H 4950 4450 50  0001 C CNN
F 1 "GND" V 4955 4572 50  0000 R CNN
F 2 "" H 4950 4700 50  0001 C CNN
F 3 "" H 4950 4700 50  0001 C CNN
	1    4950 4700
	0    1    1    0   
$EndComp
$EndSCHEMATC
