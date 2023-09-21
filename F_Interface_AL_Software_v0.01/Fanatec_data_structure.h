/*
 * #!/usr/bin/python
# -*- coding: UTF-8 -*-
"""
This script read/write data comming from a fanatec wheel.
Copyright (C) 2015 darknao
https://github.com/darknao/btClubSportWheel

This file is part of btClubSportWheel.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


Fanatec Pinout:

     1   2          1.MISO    8.CS
     O   O          2.MOSI    9.USB+
 3 O  O O  O 6      3.GND    10.
7 O   4 5   O 8     4.5v     11.DP1
 9 O       O 10     5.GND    12.USB-
  11 O   O 12       6.SCK    13.DP2
    13 O            7.3v3


Porsche/BMW/Formula connector pinout:

--
|-- 1 5v        4
|-- 2 3v3       7
|-- 3 GND       3
|-- 4 GND       5
|-- 5 MISO      1
|-- 6 MOSI      2
|-- 7 SCK       6
|-- 8 CS        8
--

UNI HUB pinout:

--
|-- 1  USB-  12
|-- 2  DP2   13
|-- 3  DP1   11
|-- 4  CS    8
|-- 5  SCK   6
|-- 6  MOSI  2
|-- 7  MISO  1
|-- 8  GND   5
|-- 9  GND   3
|-- 10 3v3   7
|-- 11 5v    4
|-- 12 USB+  9
--


DataPort (for information only):

   r---------j      1. USB+
r--l         l--j   2. DP1
|               |   3. DP2
|  1 2 3 4 5 6  |   4. 
|  _ _ _ _ _ _  |   5. 
-----------------   6. USB-


Fanatec Packet INPUT:

headers
 ^  wheel ID?
 |  ^  display
 |  |    ^    revlights
 |  |    |        ^  rumbles
 |  |    |        |     ^                              nothing                                  CRC8
|__|__|________|_____|_____|____________________________________________________________________|__|
 a5 03 FF FF FF FF 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 XX
 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32



Wheel type
      case 0x52:
      case 0xD2: rim_inserted = CSW_WHEEL;
      case 0xE0: rim_inserted = CSL_WHEEL;
      case 0xA5: rim_inserted = MCL_WHEEL;


Wheels IDs

{0x01} for BMW M3 GT2, 
{0x02} for ClubSport FORMULA, 
{0x04) for Uni hub
{0x05} X
{0x06} universal hub hub xbox one
{0x07} CSL elite p1 xbox
{0x08} CSL P1 v2 - FW update required
{0x09} Maclaren GT3 v1
{0x0A} clubsport formula v2 / v2.5
{0x0B} Maclaren GT3 v2
{0x0C} Podium hub
{0x0D} not supported
{0x0E} Bentley FWU
{0x0F} Podium M4 GT3
{0x10} X
{0x11} CSL universal hub
{0x12} CSL elite WRC 
{0x13} BMW GT3 v2
{0x14} X
{0x15} CS universal hub v2
{0x16} CS wheel f1 esports v2
{0x17} Podium BWM m4 GT3
{0x18} and above - nothing



display:
1 byte for 7segments + dot

      1
   _______
  |       |
6 |       | 2
  |   7   |
  |-------|
  |       | 3
5 |       |
  |_______| o 8
      4


revlights:
9 leds = 9 bits

    FF       01
1111 1111 0000 0001
  2 to 9          1

rumbles:
1 byte per motor (left & right)

Fanatec Packet OUTPUT:
headers
 ^  wheel ID?      encoder
 |  ^   buttons       ^       PS btns
 |  |   array1        |          ^
 |  |     ^      Joy  | buttons  |
 |  |     |      X|Y  |  copy    |sqr
 |  |     |       ^   |    ^  L&R|
 |  |     |       |   |    |     |
|__|__|________|_____|__|_____|_____|
 a5 04 00 00 00 00 00 00 00 00 00 40
 0  1  2  3  4  5  6  7  8  9  10 11

details:
buttons array1:
o o o o  o o o o | o o o o  o o o o | o o o o  o o o o

'buttons copy' report the same buttons as 'buttons array', in another order (UNI HUB only)
Except for BUT_5 which is only reported once (first octet, values are 08 10 20)



PS btns are Playstation buttons (UNI HUB)
Firt octet is the cross/square/triangle/round buttons
Second one is L&R buttons


Few samples (incoming packet):

01:14:00 0xa5  0x3  0x0  0x0  0x0  0x0  0x0  0x0 0x48 0x83 0x87 0x43 0x5b 0x9a 0x39 0xa0 0xc9 0xb5 0xd8 0x19 0x72 0x30 0x28 0xfa 0x62 0xf7 0x93  0xc 0xcb 0x98 0xd0 0x12 0x42
01:14:03 0xa5  0x3  0x0 0x80  0x0  0x0  0x0  0x0 0x48 0x83 0x87 0x43 0x5b 0x9a 0x39 0xa0 0xc9 0xb5 0xd8 0x19 0x72 0x30 0x28 0xfa 0x62 0xf7 0x93  0xc 0xcb 0x98 0xd0 0x12 0x58
01:14:04 0xa5  0x3  0x0  0x0  0x0  0x0  0x0  0x0 0x48 0x83 0x87 0x43 0x5b 0x9a 0x39 0xa0 0xc9 0xb5 0xd8 0x19 0x72 0x30 0x28 0xfa 0x62 0xf7 0x93  0xc 0xcb 0x98 0xd0 0x12 0x42
01:14:04 0xa5  0x3 0x10  0x0  0x0  0x0  0x0  0x0 0x48 0x83 0x87 0x43 0x5b 0x9a 0x39 0xa0 0xc9 0xb5 0xd8 0x19 0x72 0x30 0x28 0xfa 0x62 0xf7 0x93  0xc 0xcb 0x98 0xd0 0x12 0x3b
01:14:04 0xa5  0x3  0x0  0x0  0x0  0x0  0x0  0x0 0x48 0x83 0x87 0x43 0x5b 0x9a 0x39 0xa0 0xc9 0xb5 0xd8 0x19 0x72 0x30 0x28 0xfa 0x62 0xf7 0x93  0xc 0xcb 0x98 0xd0 0x12 0x42

UNI HUB:
20:08:26 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x40  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13  0x4 True
20:08:26 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xfc True
20:11:08 0xa5  0x4  0x0  0x0  0x0  0x0  0x0 0xff  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xa9 True
20:11:08 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xfc True
20:11:09 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x1  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xae True
20:11:09 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xfc True
20:11:13 0xa5  0x4  0x1  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xea True
20:11:15 0xa5  0x4  0x0  0x0  0x2  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0x94 True
20:11:15 0xa5  0x4  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0  0x0 0x13 0xfc True

*/
