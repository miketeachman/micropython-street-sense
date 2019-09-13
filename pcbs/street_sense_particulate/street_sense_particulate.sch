EESchema Schematic File Version 4
LIBS:street_sense_particulate-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Street Sense Power Conditioning"
Date "2019-09-13"
Rev "v01"
Comp ""
Comment1 ""
Comment2 "https://opensource.org/licenses/MIT"
Comment3 "License:  MIT License (MIT)"
Comment4 "Author:  Mike Teachman"
$EndDescr
$Comp
L power:GND #PWR02
U 1 1 5BF6FFA5
P 3700 4600
F 0 "#PWR02" H 3700 4350 50  0001 C CNN
F 1 "GND" H 3705 4427 50  0000 C CNN
F 2 "" H 3700 4600 50  0001 C CNN
F 3 "" H 3700 4600 50  0001 C CNN
	1    3700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3950 3700 4000
Wire Wire Line
	3700 4000 3900 4000
Connection ~ 3700 4000
Wire Wire Line
	3800 3650 4350 3650
$Comp
L power:GND #PWR04
U 1 1 5BF71FD6
P 7000 4200
F 0 "#PWR04" H 7000 3950 50  0001 C CNN
F 1 "GND" H 7005 4027 50  0000 C CNN
F 2 "" H 7000 4200 50  0001 C CNN
F 3 "" H 7000 4200 50  0001 C CNN
	1    7000 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4200 7000 4100
Wire Wire Line
	7000 3750 7050 3750
$Comp
L dk_Transistors-FETs-MOSFETs-Single:FQP27P06 Q1
U 1 1 5C5DE4A6
P 3600 3650
F 0 "Q1" V 3867 3650 60  0000 C CNN
F 1 "FQP27P06" V 3761 3650 60  0000 C CNN
F 2 "digikey-footprints:TO-220-3" H 3800 3850 60  0001 L CNN
F 3 "https://www.fairchildsemi.com/datasheets/FQ/FQP27P06.pdf" H 3800 3950 60  0001 L CNN
F 4 "FQP27P06-ND" H 3800 4050 60  0001 L CNN "Digi-Key_PN"
F 5 "FQP27P06" H 3800 4150 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3800 4250 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 3800 4350 60  0001 L CNN "Family"
F 8 "https://www.fairchildsemi.com/datasheets/FQ/FQP27P06.pdf" H 3800 4450 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/FQP27P06/FQP27P06-ND/965349" H 3800 4550 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET P-CH 60V 27A TO-220" H 3800 4650 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3800 4750 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3800 4850 60  0001 L CNN "Status"
	1    3600 3650
	0    -1   -1   0   
$EndComp
$Comp
L dk_Transistors-FETs-MOSFETs-Single:FQP27P06 Q3
U 1 1 5C8F263F
P 5700 3650
F 0 "Q3" V 5967 3650 60  0000 C CNN
F 1 "FQP27P06" V 5861 3650 60  0000 C CNN
F 2 "digikey-footprints:TO-220-3" H 5900 3850 60  0001 L CNN
F 3 "https://www.fairchildsemi.com/datasheets/FQ/FQP27P06.pdf" H 5900 3950 60  0001 L CNN
F 4 "FQP27P06-ND" H 5900 4050 60  0001 L CNN "Digi-Key_PN"
F 5 "FQP27P06" H 5900 4150 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 5900 4250 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 5900 4350 60  0001 L CNN "Family"
F 8 "https://www.fairchildsemi.com/datasheets/FQ/FQP27P06.pdf" H 5900 4450 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/FQP27P06/FQP27P06-ND/965349" H 5900 4550 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET P-CH 60V 27A TO-220" H 5900 4650 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 5900 4750 60  0001 L CNN "Manufacturer"
F 12 "Active" H 5900 4850 60  0001 L CNN "Status"
	1    5700 3650
	0    1    -1   0   
$EndComp
$Comp
L Transistor_BJT:PN2222A Q2
U 1 1 5C8F6DC9
P 5500 4650
F 0 "Q2" H 5691 4696 50  0000 L CNN
F 1 "PN2222A" H 5691 4605 50  0000 L CNN
F 2 "digikey-footprints:TO-92-3_Formed_Leads" H 5700 4575 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/PN/PN2222A.pdf" H 5500 4650 50  0001 L CNN
	1    5500 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5BF6F4AA
P 5250 4000
F 0 "R3" H 5320 4046 50  0000 L CNN
F 1 "10k" H 5320 3955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 5180 4000 50  0001 C CNN
F 3 "~" H 5250 4000 50  0001 C CNN
	1    5250 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5C8F882F
P 5050 4650
F 0 "R2" H 5120 4696 50  0000 L CNN
F 1 "1k" H 5120 4605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4980 4650 50  0001 C CNN
F 3 "~" H 5050 4650 50  0001 C CNN
	1    5050 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5200 4650 5250 4650
$Comp
L power:GND #PWR03
U 1 1 5C90A4BB
P 5600 5150
F 0 "#PWR03" H 5600 4900 50  0001 C CNN
F 1 "GND" H 5605 4977 50  0000 C CNN
F 2 "" H 5600 5150 50  0001 C CNN
F 3 "" H 5600 5150 50  0001 C CNN
	1    5600 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5C90C028
P 5250 4900
F 0 "R4" H 5320 4946 50  0000 L CNN
F 1 "10k" H 5320 4855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 5180 4900 50  0001 C CNN
F 3 "~" H 5250 4900 50  0001 C CNN
	1    5250 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 4750 5250 4650
Connection ~ 5250 4650
Wire Wire Line
	5250 4650 5300 4650
Wire Wire Line
	5250 5050 5600 5050
Wire Wire Line
	5600 5150 5600 5050
Connection ~ 5600 5050
Wire Wire Line
	5600 4850 5600 5050
Wire Wire Line
	4350 4000 4350 3650
Connection ~ 4350 3650
Wire Wire Line
	4350 3650 4950 3650
Wire Wire Line
	5600 3950 5600 4300
Wire Wire Line
	5250 3850 5250 3650
Wire Wire Line
	5250 3650 5500 3650
Wire Wire Line
	5250 4150 5250 4300
Wire Wire Line
	5250 4300 5600 4300
Connection ~ 5600 4300
Wire Wire Line
	5600 4300 5600 4450
$Comp
L Device:CP1 C1
U 1 1 5C9AA527
P 6650 3900
F 0 "C1" H 6765 3946 50  0000 L CNN
F 1 "220uF" H 6765 3855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 6650 3900 50  0001 C CNN
F 3 "~" H 6650 3900 50  0001 C CNN
	1    6650 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4050 6650 4100
Wire Wire Line
	6650 4100 7000 4100
Connection ~ 7000 4100
Wire Wire Line
	7000 4100 7000 3750
Wire Wire Line
	6650 3750 6650 3650
Wire Wire Line
	6650 3650 6800 3650
$Comp
L Device:D_Schottky D1
U 1 1 5C9DBCEF
P 4050 4000
F 0 "D1" V 4050 4216 50  0000 C CNN
F 1 "MBD 301" H 4050 4125 50  0000 C CNN
F 2 "Diode_THT:D_A-405_P5.08mm_Vertical_AnodeUp" H 4050 4000 50  0001 C CNN
F 3 "~" H 4050 4000 50  0001 C CNN
	1    4050 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	4350 4000 4200 4000
$Comp
L Device:R R1
U 1 1 5C8EB050
P 3700 4350
F 0 "R1" H 3770 4396 50  0000 L CNN
F 1 "100k" H 3770 4305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3630 4350 50  0001 C CNN
F 3 "~" H 3700 4350 50  0001 C CNN
	1    3700 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4000 3700 4200
Wire Wire Line
	3700 4500 3700 4550
$Comp
L mike_library:5V_DC_DC_BOOST U1
U 1 1 5D78E040
P 7450 4000
F 0 "U1" H 7325 4617 50  0000 C CNN
F 1 "5V_DC_DC_BOOST" H 7325 4526 50  0000 C CNN
F 2 "digikey-footprints:TO-220-3" H 7450 4000 50  0001 C CNN
F 3 "" H 7450 4000 50  0001 C CNN
	1    7450 4000
	1    0    0    -1  
$EndComp
Connection ~ 5250 3650
$Comp
L Connector_Generic:Conn_01x08 J1
U 1 1 5D794743
P 1750 2850
F 0 "J1" H 1830 2842 50  0000 L CNN
F 1 "Conn_01x08" H 1830 2751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 1750 2850 50  0001 C CNN
F 3 "~" H 1750 2850 50  0001 C CNN
	1    1750 2850
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5D795B5E
P 9350 2750
F 0 "J2" H 9430 2742 50  0000 L CNN
F 1 "Conn_01x04" H 9430 2651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 9350 2750 50  0001 C CNN
F 3 "~" H 9350 2750 50  0001 C CNN
	1    9350 2750
	1    0    0    -1  
$EndComp
NoConn ~ 1950 2550
NoConn ~ 1950 3050
Wire Wire Line
	3200 3650 3400 3650
Wire Wire Line
	3050 4000 3700 4000
$Comp
L power:GND #PWR01
U 1 1 5D79CD31
P 2450 3450
F 0 "#PWR01" H 2450 3200 50  0001 C CNN
F 1 "GND" H 2455 3277 50  0000 C CNN
F 2 "" H 2450 3450 50  0001 C CNN
F 3 "" H 2450 3450 50  0001 C CNN
	1    2450 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3450 2450 2850
Wire Wire Line
	1950 2850 2450 2850
$Comp
L Connector:TestPoint TP1
U 1 1 5D3B45F6
P 3200 4500
F 0 "TP1" H 3250 4700 50  0000 L CNN
F 1 "TestPoint" H 3250 4600 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 3400 4500 50  0001 C CNN
F 3 "~" H 3400 4500 50  0001 C CNN
	1    3200 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5D7C176A
P 8850 3000
F 0 "#PWR05" H 8850 2750 50  0001 C CNN
F 1 "GND" H 8855 2827 50  0000 C CNN
F 2 "" H 8850 3000 50  0001 C CNN
F 3 "" H 8850 3000 50  0001 C CNN
	1    8850 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 2850 9150 2850
Wire Wire Line
	9050 2950 9150 2950
Wire Wire Line
	4650 4650 4650 2950
Wire Wire Line
	4650 2950 1950 2950
Wire Wire Line
	4650 4650 4900 4650
Wire Wire Line
	3050 3250 3050 4000
Wire Wire Line
	3200 3650 3200 3150
Wire Wire Line
	9050 2950 9050 3700
$Comp
L Device:Jumper JP2
U 1 1 5D7CBFE2
P 7900 3700
F 0 "JP2" H 7900 3964 50  0000 C CNN
F 1 "Jumper" H 7900 3873 50  0000 C CNN
F 2 "street_sense_footprints:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 7900 3700 50  0001 C CNN
F 3 "~" H 7900 3700 50  0001 C CNN
	1    7900 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP1
U 1 1 5D7CE97F
P 6300 3650
F 0 "JP1" H 6300 3914 50  0000 C CNN
F 1 "Jumper" H 6300 3823 50  0000 C CNN
F 2 "street_sense_footprints:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 6300 3650 50  0001 C CNN
F 3 "~" H 6300 3650 50  0001 C CNN
	1    6300 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3650 6600 3650
Connection ~ 6650 3650
Wire Wire Line
	5900 3650 6000 3650
Wire Wire Line
	3200 4500 3200 4550
Wire Wire Line
	3200 4550 3700 4550
Connection ~ 3700 4550
Wire Wire Line
	3700 4550 3700 4600
$Comp
L Connector:TestPoint TP2
U 1 1 5D7DAD08
P 4950 3300
F 0 "TP2" H 5000 3500 50  0000 L CNN
F 1 "TestPoint" H 5000 3400 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 5150 3300 50  0001 C CNN
F 3 "~" H 5150 3300 50  0001 C CNN
	1    4950 3300
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5D7DB8B9
P 6800 3250
F 0 "TP3" H 6858 3368 50  0000 L CNN
F 1 "TestPoint" H 6858 3277 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 7000 3250 50  0001 C CNN
F 3 "~" H 7000 3250 50  0001 C CNN
	1    6800 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5D7DC543
P 8200 3250
F 0 "TP4" H 8258 3368 50  0000 L CNN
F 1 "TestPoint" H 8258 3277 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 8400 3250 50  0001 C CNN
F 3 "~" H 8400 3250 50  0001 C CNN
	1    8200 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3250 8200 3700
Connection ~ 8200 3700
Wire Wire Line
	8850 2850 8850 3000
Wire Wire Line
	4950 3300 4950 3650
Connection ~ 4950 3650
Wire Wire Line
	4950 3650 5250 3650
Wire Wire Line
	6800 3250 6800 3650
Connection ~ 6800 3650
Wire Wire Line
	6800 3650 7050 3650
Wire Wire Line
	8200 3700 9050 3700
Wire Wire Line
	1950 2650 9150 2650
Wire Wire Line
	1950 2750 9150 2750
Text Label 2050 2650 0    50   ~ 0
RX
Text Label 2050 2750 0    50   ~ 0
TX
Text Label 2050 3150 0    50   ~ 0
BAT
Text Label 2050 3250 0    50   ~ 0
USB
Wire Wire Line
	1950 3250 2600 3250
$Comp
L Connector:TestPoint TP5
U 1 1 5D788340
P 2600 2400
F 0 "TP5" H 2650 2600 50  0000 L CNN
F 1 "TestPoint" H 2650 2500 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 2800 2400 50  0001 C CNN
F 3 "~" H 2800 2400 50  0001 C CNN
	1    2600 2400
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP6
U 1 1 5D788983
P 3200 2400
F 0 "TP6" H 3250 2600 50  0000 L CNN
F 1 "TestPoint" H 3250 2500 50  0000 L CNN
F 2 "TestPoint:TestPoint_Loop_D2.50mm_Drill1.0mm" H 3400 2400 50  0001 C CNN
F 3 "~" H 3400 2400 50  0001 C CNN
	1    3200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3150 3200 3150
Wire Wire Line
	2600 2400 2600 3250
Connection ~ 2600 3250
Wire Wire Line
	2600 3250 3050 3250
Wire Wire Line
	3200 2400 3200 3150
Connection ~ 3200 3150
Text Label 2050 2950 0    50   ~ 0
CTRL
$EndSCHEMATC
