# esp32_aht20_bmp280: basic example

Reads temperature, humidity, and pressure from an AHT20 + BMP280 sensor pair
every 5 seconds and logs the result.

## Wiring

- AHT20 + BMP280 on the same I2C bus
- SDA → GPIO4, SCL → GPIO5 by default; change via `idf.py menuconfig` →
  "AHT20+BMP280 Sensor configuration" (`I2C_SDA_PIN`/`I2C_SCL_PIN`/`I2C_BUS`)
- BMP280 I2C address defaults to HIGH (0x77); if the SDO pin is tied to GND,
  switch to LOW (0x76) in the same menu
- Vdd must be wired directly to power — this example leaves `POWER_IS_GPIO`
  disabled (see `sdkconfig.defaults`), so nothing drives a power pin

## Build and flash

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

## Expected output

```
I (1234) env_sensor_example: T=22.41°C  RH=41.30%  P=1013.25hPa
```
