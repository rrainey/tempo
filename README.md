# Tempo

### Experiments in audible feedback for skydivers.

## Directory Structure

| Folder      | Description |
| ----------- | ----------- |
| firmware    | Arduino sketches       |
| hardware    | KiCad PCB projects (using KiCad 7)    |
| hardware/peakick | u-blox GNSS + IMU + Barometric sensor PCB |
| enclosure    | 3D-printable enclosure (Fusion360 format)

SparkFun ESP32 Thing Plus combined with the peakick peripheral board.

## Enclosure

The enclosure is designed to be SLA printable. I print using a MakerBot Replicator 2 with Cura 5.3.1 as the slicer. Print resolution set to 0.15mm.

The PCBs are secured to the enclosure using M2.5 brass inserts and screws.

## Parts List

| QTY | MPN      | Description |
|----| ----------- | ----------- |
| 1 | [WRL-15663](https://www.digikey.com/en/products/detail/sparkfun-electronics/WRL-15663/11506265)   | SparkFun ESP32 Thing Plus WROOM
| 1 | N/A   | peakick PCB assembly
| 3  |       | 6 POS 5mm 0.10" receptacle (soldered to peakick PCB)
| 3  |       | 6 POS 0.10" headers
| 1 | [1578](https://www.digikey.com/en/products/detail/adafruit-industries-llc/1578/5054539?utm_adgroup=Battery%20Products&utm_source=google&utm_medium=cpc&utm_campaign=Dynamic%20Search_EN_RLSA_Buyers&utm_term=&utm_content=Battery%20Products&utm_id=go_cmp-175054755_adg-15264279675_ad-399492818526_aud-505192123430:dsa-53357708014_dev-c_ext-_prd-_sig-Cj0KCQjwm66pBhDQARIsALIR2zAggpxuq8dQv4im2FGo1CqTU3N75aE9USMP6jGWoU6Vr5h_xDsHwy8aAp_DEALw_wcB&gclid=Cj0KCQjwm66pBhDQARIsALIR2zAggpxuq8dQv4im2FGo1CqTU3N75aE9USMP6jGWoU6Vr5h_xDsHwy8aAp_DEALw_wcB) | Adafruit 3.7V 500mAH LiPoly rechargeable battery
| 3 | [1GNL7](https://www.grainger.com/product/GRAINGER-APPROVED-Heat-Set-Insert-M2-5-0-45-1GNL7) | Grainger M2.5 Brass Inserts
| 3 |       | M2.5 x 16 mm phillips screws
| 1 | N/A   | Project enclosure (Top and bottom shells)