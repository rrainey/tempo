# tempo‑bt V1 board — Peripheral IC Connections 

**Core MCU/Module:** u‑blox NORA‑B106 (nRF5340)
**OS:** Zephyr RTOS
**Goal:** Provide a canonical, test‑ready mapping of logical nets from the NORA‑B106 to each peripheral IC for driver bring‑up and HW connectivity tests.

## High‑level digital interconnect

```mermaid
flowchart LR
  subgraph Core [NORA‑B106 (nRF5340)]
    QSPI[QSPI: SCK, CSN, IO0..IO3]
    SPI4[SPI (SPIM4): SCK, MOSI, MISO, CSN]
    I2C[I²C: SCL, SDA]
    UART_A[UART A: NET_NORA_TX/RX]
    UART_B[UART B: APP_NORA_TX/RX, RTS/CTS]
    VCOM[USB‑UART: VCOM1_TX/RX]
  end

  QSPI --> Flash[MX25R6435F 64Mbit QSPI NOR]
  SPI4 --> IMU[ICM‑42688‑V 6‑axis IMU]
  I2C --> Press[BMP390 Baro]
  I2C --> Mag[MMC5983MA Mag]
  UART_A --> GNSS[SAM‑M10Q GNSS]
  UART_B --> TestApp[App/aux interface]
  VCOM --> HostPC[USB‑UART Console]
```

> Notes
> • Net names and bus roles mirror the labels present on the schematic (e.g., `SPIM4_SCK`, `QSPI_IO0`, `SCL`, `SDA`, `NET_NORA_TX`, `APP_NORA_RTS/CTS`, `VCOM1_RX/TX`).
> • Where interrupts or extra control pins exist but were not explicitly net‑labeled to the MCU on the sheet, they’re called out as **TBD/verify** so you can tag them to specific GPIOs during bring‑up.

---

## User Outputs / Inputs

| Signal | B106 connection | Description |
|--------|----------------|--------------|
| LED1   | `P1.06`          | |
| LED2   | `P0.12`          | |
| BTN1   | `P0.23`          | Pushbutton 1 (with hardware debounce) |
| BTN2   | `P0.24`          | Pushbutton 2 (with hardware debounce) |

## Connectivity tables (by peripheral)

### 1) External QSPI NOR flash — **MX25R6435F (64 Mbit)**

| Bus  | PCB logical net | B106 Port  | Flash pin/function | Notes                                |
| ---- | --------------- | ---------- | ------------------ | ------------------------------------ |
| QSPI | `QSPI_SCK`      | `P0.17/SCK`  | SCLK               | Primary quad SPI clock               |
| QSPI | `QSPI_CSN`      | `P0.18/CSN`  | CS#                | Chip select                          |
| QSPI | `QSPI_IO0`      | `P0.13/IO0`  | IO0 (MOSI/SI)      | Data/IO0                             |
| QSPI | `QSPI_IO1`      | `P0.14/IO1`  | IO1 (MISO/SO)      | Data/IO1                             |
| QSPI | `QSPI_IO2`      | `P0.15/IO2`  | IO2 (WP#)          | Quad IO2 / Write Protect when legacy |
| QSPI | `QSPI_IO3`      | `P0.16/IO3`  | IO3 (HOLD#/RESET#) | Quad IO3 / Hold# when legacy         |

Additional compatibility labels seen on sheet:

* `FLASH_MOSI`, `FLASH_MISO`, `FLASH_SCK` appear alongside the `QSPI_*` set; treat as alternative/legacy SPI naming used at fan‑out or test points.

**Zephyr tips:** Use `jedec,spi-nor` with `qspi` child and 4‑line mode; map `io0..io3` and `cs-gpios` to the same nets as above.

---

### 2) 6‑axis IMU — **ICM‑42688‑V**

| Bus         | PCB net         | B106 Port    | IMU pin/function | Notes                                          |
|:-----------:| --------------- | ----------   |:----------------:| ---------------------------------------------- |
| SPI (SPIM4) | `SPIM4_SCK`     | `P0.08/SCK`  | SCLK             | IMU serial clock                               |
| SPI (SPIM4) | `SPIM4_MOSI`    | `P0.09/MOSI` | SDI              | Master‑out / IMU SDI                           |
| SPI (SPIM4) | `SPIM4_MISO`    | `P0.10/MISO` | SDO              | Master‑in / IMU SDO                            |
| SPI (SPIM4) | `SPIM4_CSN`     | `P0.11/CSN`  | CSB              | Chip select, active low                        |
| GPIO        | **D6**          | `P1.07`      | INT1             | Interrupt to NORA GPIO  |
|   | NC   | NC  | INT2/FSYNC/CLKIN | Optional secondary INT/FSYNC (unconnected)          |

**What’s explicit vs. to‑confirm:** The `SPIM4_*` nets are explicit on the sheet and should route to the IMU serial pins. Interrupt lines are present on the symbol but aren’t labeled to a specific net name at the MCU in the text export I parsed—please tag to the intended GPIOs during DTS authoring.

---

### 3) Barometric pressure — **BMP390**

| Bus        | PCB net | B106 port | BMP390 pin/function | Notes                                 |
|:----------:| ---------------|:------:|:-------------------:| ------------------------------------- |
| I²C        | `SCL` | `P1.02/TWI`        | SCL                 | I²C clock                             |
| I²C        | `SDA` | `P1.03/TWI`        | SDA                 | I²C data                              |
| GPIO | **D11** | `P1.11`     | INT                 | |


---

### 4) Magnetometer — **MMC5983MA**

| Bus        | PCB net | B106 port | MMC5983MA pin/function | Notes                                   |
|:----------:| ----------------|:-----:|:----------------------:| --------------------------------------- |
| I²C        | `SCL`  | `P1.02/TWI`               | SCL                    | Shared I²C clock                        |
| I²C        | `SDA`  | `P1.03/TWI`               | SDA                    | Shared I²C data                         |
| GPIO | **D9**  | `P1.09`      | INT                   |   |



---

### 5) GNSS module — **u‑blox SAM‑M10Q**

Note: V1 board has GNSS UART connections only.  V2 board will connect PPS TIMEPULSE to a GPIO line (line to-be-determined).

| Interface          | NORA‑B106 logical net                | SAM‑M10Q function | Notes                                                                                                                                            |
| ------------------ | ------------------------------------ |:-----------------:| ------------------------------------------------------------------------------------------------------------------------------------------------ |
| UART               | `NET_NORA_TX` → GNSS RX              | RXD               | Primary GNSS serial in                                                                                                                           |
| UART               | `NET_NORA_RX` ← GNSS TX              | TXD               | Primary GNSS serial out                                                                                                                          |
| UART (alt/app)     | `APP_NORA_TX/RX`, `APP_NORA_RTS/CTS` | TX/RX/RTS/CTS     | A second NORA UART is brought out as “APP\_NORA\_\*”; use if the GNSS is wired to the app port or for auxiliary host/debug depending on stuffing |
| USB‑UART (console) | `VCOM1_TX/RX`                        | TX/RX             | Labeled console link to host PC/bridge; useful for test logging                                                                                  |

> The schematic contains `NET_NORA_TX/RX` and also `APP_NORA_*` plus `VCOM1_*`. Use the pair that terminates on the SAM‑M10Q in your assembled option. For Zephyr, declare the chosen UART instance as `gps0` with `ubx,m10` (or use NMEA‑compatible driver) and disable the unused one(s).

---

## Zephyr bring‑up checklist (how to use this map)

1. **Pick instances**

   * QSPI: enable nRF QSPI controller, bind `cs-gpios` and `io0..io3` to `QSPI_*`.
   * SPI: enable `spi4` (SPIM4) and bind to IMU.
   * I²C: enable the I²C controller feeding `SCL/SDA` bus; add BMP390 + MMC5983MA as children with proper addresses.
   * UART: choose **either** `NET_NORA_*` (GNSS) **or** `APP_NORA_*` based on your assembly; reserve `VCOM1_*` for console.

2. **GPIOs/interrupts**

   * Assign IMU `INT1` and the magnetometer `INT` to listed GPIOs
   * Add `interrupts = <&gpioX pin IRQ_EDGE_RISING>;` in each child node.

3. **Testing hooks**

   * Flash: probe with JEDEC ID over QSPI, read SFDP.
   * IMU: WHO\_AM\_I on `SPIM4`; read INT on motion.
   * Baro/Mag: ID registers over I²C; poll data‑ready.
   * GNSS: open UART, request UBX‑MON‑VER, check NMEA stream.
   * Console: ensure `VCOM1_*` maps to the chosen Zephyr `console`.

---

## Appendix — Signal name index (as found on the sheet)

From the schematic net labels (selection):
`QSPI_SCK`, `QSPI_CSN`, `QSPI_IO0`, `QSPI_IO1`, `QSPI_IO2`, `QSPI_IO3`, `FLASH_MOSI`, `FLASH_MISO`, `FLASH_SCK`, `SPIM4_SCK`, `SPIM4_MOSI`, `SPIM4_MISO`, `SPIM4_CSN`, `SCL`, `SDA`, `NET_NORA_TX`, `NET_NORA_RX`, `APP_NORA_TX`, `APP_NORA_RX`, `APP_NORA_RTS`, `APP_NORA_CTS`, `VCOM1_TX`, `VCOM1_RX`, `SWDIO`, `SWDCLK`, `~RESET`, `BTN1`, `BTN2`, `BTN3`, `BTN4`, `D5`, `D6`, `D7`, `D9`, `D10`, `D11`, `D12`, `D13`, `P0.31`, `XL1`, `XL2`, `VDDIO`, `VBACKUP`.

`D*` lines (`D5` .. `D13`) were selected to correspond the the nRF8340 Arduino developement kit board definition.

