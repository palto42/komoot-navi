# BLE Receiver for Komoot navigation app

[Komoot](https://www.komoot.de) is a navigation app for Android and iPhone especially for cycling and hiking. The BLE protocol for external navigation displays is described on [Komoot BLE Connect](https://docs.google.com/document/d/1iYgV4lDKG8LdCuYwzTXpt3FLO_kaoJGGWzai8yehv6g/) (local [copy](./doc/Komoot%20BLEConnect.odt))

**Note**: The Komoot Github library [kommot/BLEConnect](https://github.com/komoot/BLEConnect) has been removed.

This project implements a BLE receiver based on an ESP32 module and a 64*128 pixel OLED display to show the direction, distance and street name to the next junction.

The device is turned on via a touch input connected to the ESP32 pin 15 and automatically turns off if it doesn't receive new updates from the Komoot app for a while.

For more information, have a look at [Komoot BLE receiver description](/doc/Komoot%20BLE%20receiver.md).

## Navi in 3D-printed case

The name of the next street is shown at the top of the display. In the middle the direction and the distance to the next way point can be found. The bottom shows the current street name and below it the battery level.

![navi in 3d-printed case](/doc/navi-case.jpg)

The Blender and STL files for the 3D-printed case are in the [/navi-case](/navi-case)] folder. It is designed for the Lolin32 lite and the 1.3" OLED display.
