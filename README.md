# vaisala_rs41_receiver

Silicon Labs EFR32 based receiver for Vaisala RS41 meteorological sonde

The firmware requires an EFR32MG12 device to run (only the MG can run BLE + Proprietary, and only the 12 has enough flash to run the BLE stack). The project is created with Simplicity Studio 5, Flex SDK 3.2.3, and Bluetooth SDK 3.2.4 using BRD4173A Rev A00 (radio board) + BRD4001A (mainboard).

# Installation

After cloning the repo import the project by navigating to the vaisala_rs41_ble directory. If the import was successful open the *.slcp* file and click on *Force Generation* then compile the project and download the binary image to the device.

# Features

The application utilizes the LCD available on the mainboard. Currently, it has a dual function, by default the most important information displayed (ID of the sonde, GPS coordinates) but it can display a QR code that contains an URL that points to www.openstreetmap.org with the current GPS coordinates. The left button (PB1) toggles between the two screens.

The device acts as a BLE server and updates a characteristic that contains the received values on every incoming packet from the sonde. The format is not decided should be adjusted to the format required by the client-side.

# Note

By default, the frequency is set to receive the signal of the sondes started from Budapest (currently 403.2MHz). The frequency can be increased by the right button (PB0) by 100kHz on every button push, the minimum frequency is 400MHz, the maximum is 405MHz

# Todo

- Making error corrections work
- Decide the structure of the data transmitted over the BLE connection
- Solution to pass the received information to mapping apps (for example a plugin for OSMAnd)
