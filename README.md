# BLE Receiver for Komoot naviagtion app

[Komoot](https://www.komoot.de) is a navigation app for Android and iPhone especially for cycling and hiking. The BLE protocoll for external navigation displays is described on [kommot/BLEConnect](https://github.com/komoot/BLEConnect)

This project implements a BLE receiver based on an ESP32 module and a 64*128 pixel OLED display to show the direction, distance and street name to the next junction.

The prototype uses a [Wemos Lolin32](https://wiki.wemos.cc/products:lolin32:lolin32) board and a 0.96" (SSD1306) or 1.3" (SSD1106) OLED display. The unit can be powered with a 3.7 V LiPo battery (e.g. 600 mAh).

A voltage devider with two resistors is used to measure the battery power.

Navi in 3D-printect case:

![navi in 3d-printed case](/doc/navi-case.jpg)

For more information, please check the [/doc](/doc) folder.
The Blender and STL files for the 3D-printed case are in the [/navi-case](/navi-case)] folder.
