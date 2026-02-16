# Pico 2 W Air Quality Datalogger (PM2.5 + Multi-Gas + CO2)

This project implements a standalone Air Quality Monitor using a **Raspberry Pi Pico 2 W**. It combines three powerful sensors to monitor particulate matter, gaseous pollutants, and environmental conditions:

1.  **HM3301:** Laser Particulate Matter Sensor (PM1.0, PM2.5, PM10).
2.  **Grove Multichannel Gas V2.0:** Monitors NO2, Ethanol, VOC, and Carbon Monoxide.
3.  **SCD41:** Precision CO2, Temperature, and Relative Humidity Sensor.

The device logs data to an **SD Card** every 15 minutes and acts as a **BLE Peripheral** to communicate with a mobile client for live readings, file transfer, and time synchronization.

## Features

* **Particulate Monitoring:** Reads Standard PM values (CF=1) for PM1.0, PM2.5, and PM10.
* **Gas Monitoring:** Reads raw values for Nitrogen Dioxide (NO2), Ethanol (C2H5OH), Volatile Organic Compounds (VOC), and Carbon Monoxide (CO).
* **Environmental Monitoring:** Reads CO2 (ppm), Temperature (°C), and Relative Humidity (%RH) via the SCD41.
* **SD Card Logging:** Logs combined sensor data every 15 minutes to daily CSV files (e.g., `2025-10-30.txt`).
* **BLE Connectivity:**
    * **Live Updates:** Notifies connected clients of current PM2.5, Gas, and Environmental levels every 5 seconds.
    * **File Transfer:** Allows clients to request and stream logged files wirelessly.
    * **RTC Synchronization:** Syncs the Pico's internal clock via BLE for accurate timestamps.
* **Low Power Design:** Uses the Pico's AON (Always-on) Timer for scheduling. (note: the AON timer is only compatible with the RP2350-based Pico 2 W, not the original RP2040 Pico W)

## Hardware Requirements

* **MCU:** Raspberry Pi Pico 2 W (not the original Pico W, as the AON timer is required for low-power scheduling).
* **PM Sensor:** HM3301 Laser PM2.5 Dust Sensor.
* **Gas Sensor:** [Grove - Multichannel Gas Sensor V2.0](https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/).
* **CO2 Sensor:** Sensirion SCD41 (or SCD4x series).
* **Storage:** SD Card Module (SPI Interface) + MicroSD Card (Formatted FAT32).

## Wiring Guide

### PM & Gas Sensors (I2C0)
These sensors share the primary I2C bus. You can daisy-chain them or use a Grove I2C Hub .

| Pin Function | Pico Pin | Sensor Pin |
| :--- | :--- | :--- |
| **I2C SDA** | `GPIO 8` | SDA (HM3301 & Gas V2)  |
| **I2C SCL** | `GPIO 9` | SCL (HM3301 & Gas V2)  |
| **Power** | `3V3_OUT` | VCC (Both)* |
| **Ground** | `GND` | GND (Both) |

*\*Note: The Gas Sensor heater consumes significant current. Ensure your Pico is powered by a stable USB source .*

### SCD41 Sensor (I2C1)
The SCD41 uses a separate I2C bus to ensure stability and compatability with the official Sensirion HAL.

| Pin Function | Pico Pin | Sensor Pin |
| :--- | :--- | :--- |
| **I2C SDA** | `GPIO 6` | SDA (SCD41) |
| **I2C SCL** | `GPIO 7` | SCL (SCD41) |
| **Power** | `3V3_OUT` | VCC |
| **Ground** | `GND` | GND |

### SD Card (SPI1)
Standard SPI connection for data logging.

| SD Card Pin | Pico Pin | Description |
| :--- | :--- | :--- |
| **SCK** | `GPIO 10` | SPI Clock  |
| **MOSI** | `GPIO 11` | Master Out Slave In  |
| **MISO** | `GPIO 12` | Master In Slave Out  |
| **CS** | `GPIO 13` | Chip Select  |
| **VCC** | `3V3_OUT` | Power  |
| **GND** | `GND` | Ground  |

## BLE Interface (GATT Profile)

The device advertises as "PM2.5 Logger". Service UUID: `0xAAA0`.

| Characteristic | UUID | Type | Description |
| :--- | :--- | :--- | :--- |
| **RTC Sync** | `0xAAA1` | Write | Write 7 bytes: `[Year_H, Year_L, Month, Day, Hour, Min, Sec]` . |
| **Command** | `0xAAA2` | Write | Send `GET:<filename>` or `LIST` . |
| **File Stream** | `0xAAA3` | Notify | File content stream (64-byte chunks). Ends with `$$EOT$$` . |
| **Live PM2.5** | `0xAAA4` | Notify | Current PM2.5 (uint16_t, Little Endian) every 5s. |
| **Live Gas** | `0xAAA5` | Notify | Current Gas Raw Values. 16 bytes: `[NO2, Eth, VOC, CO]` (4x uint32_t, Little Endian) . |
| **Live Env** | `0xAAA6` | Notify | Live Environmental Data. 10 bytes: `[CO2(2B), Temp(4B), Hum(4B)]` (Little Endian). |

**Note:** Temperature and Humidity in `0xAAA6` are transmitted as `int32_t` milli-units. Divide by 1000 to get degrees Celsius and %RH.

## Data Logging Format

Files are named `YYYY-MM-DD.txt`. The content is a CSV-like format:

```csv
YYYY-MM-DDTHH:MM:SS,PM1_0:<val>,PM2_5:<val>,PM10:<val>,NO2:<val>,Eth:<val>,VOC:<val>,CO:<val>,CO2:<val>,T:<val>,H:<val>
```
Example:
```
2025-12-10T14:30:00,PM1_0:10,PM2_5:15,PM10:18,NO2:450,Eth:120,VOC:300,CO:150,CO2:850,T:25400,H:45500
```
(In this example, T=25.40°C and H=45.50%RH)

## Build and Flash

1. Use VSCode with the official [Pico extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) for easier building and flashing.

2. `Ctlr+Shift+P` -> `CMake: Configure`

3. Click `Compile` in bottom bar.

4. Put your Pico 2 W into BOOTSEL mode (hold the BOOTSEL button while plugging it in).

5. Click `Run` in bottom bar to flash the firmware.

**Note:** Ensure your SD card is inserted into the SD card module before powering on the Pico 2 W.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details
