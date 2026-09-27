# Atmosphere Touch - ESP32-S3 Environmental Monitor

A personal embedded project built around an ESP32-S3 DevKitC-1 N16R8, ILI9341 320x240 SPI TFT with XPT2046 resistive touch, and a DHT22 sensor.

## Main features
- Indoor temperature and humidity from DHT22
- Touch-based graphical UI
- Multiple UI themes and character animations
- Wi-Fi + NTP time synchronization
- Open-Meteo outdoor weather data and 3-day forecast
- Local settings stored in ESP32 NVS

## Hardware
- ESP32-S3 DevKitC-1 N16R8
- ILI9341 320x240 SPI TFT
- XPT2046 resistive touch controller
- DHT22 3-pin module

## Wiring
See `hardware/Atmosphere_Touch_Schematic.pdf` and `hardware/wiring.csv`.

| ESP32-S3 | Connected to |
|---|---|
| GPIO4 | DHT22 DATA |
| GPIO10 | TFT CS |
| GPIO8 | TFT RESET |
| GPIO9 | TFT DC |
| GPIO11 | TFT MOSI + Touch T_DIN |
| GPIO12 | TFT SCK + Touch T_CLK |
| GPIO13 | TFT MISO + Touch T_DO |
| GPIO6 | Touch T_CS |
| 5V/VIN | TFT VCC* |
| 3.3V | TFT LED + DHT22 VCC |
| GND | Common ground |

\*Use 5 V for TFT VCC only if the exact ILI9341 module board is designed for 5 V input, as in the tested hardware used for this project.

## Libraries
- Adafruit GFX Library
- Adafruit ILI9341
- XPT2046_Touchscreen by Paul Stoffregen
- DHT sensor library by Adafruit
- ArduinoJson by Benoit Blanchon

## Development note
The project was designed, assembled, tested, calibrated, and iteratively developed on real hardware. AI tools were used as supporting tools for troubleshooting, code review, optimization, and development speed. Hardware integration, component selection, requirements, testing, calibration, and final implementation were carried out through hands-on project work.

## Repository structure
- `src/` - firmware source
- `hardware/` - schematic and wiring documentation
- `images/` - project photos and screenshots
