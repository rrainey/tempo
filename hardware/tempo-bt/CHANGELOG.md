# Tempo-BT Changelog

## 2025-01-31  V1 PCB

I designed this board to experiment with adding Bluetooth capabilities to a Tempo-style design.  Sensor peripherals are identical to the Tempo V1:

* ICM-46688-V 6-DOF IMU
* BMP390 barometer
* MMC5983MA magnetic compass sensor
* u-blox SAM-M10Q GNSS receiver

A u-blox NORA-B106 is used as the SOC.  The NORA-B106 is a prepackaged Nordic nRF5340 dual core processor combined with an on-board Bluetooth antenna module.

The design also includes an Micro-SD card slot, an external 8MB flash memory IC, and battery backup for the GNSS receiver.

This design also include two tactile pushbuttons user for input, two programmable fixed-color LEDs, and one programmable RGB LED.
