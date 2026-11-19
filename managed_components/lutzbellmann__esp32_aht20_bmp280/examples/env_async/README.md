# esp32_aht20_bmp280: async (auto-read) example

Demonstrates `env_sensor_init_auto_read`/`env_sensor_stop_auto_read`: a
background task powers up the sensors, reads them, and powers them down
again every 15 seconds, while `app_main` just logs the latest cached
reading (`env_sensor_last_reading`) on the same interval. After 2 minutes,
automatic reading is stopped.

## Wiring

- AHT20 + BMP280 on the same I2C bus
- SDA → GPIO4, SCL → GPIO5 by default; change via `idf.py menuconfig` →
  "AHT20+BMP280 Sensor configuration" (`I2C_SDA_PIN`/`I2C_SCL_PIN`/`I2C_BUS`)
- BMP280 I2C address defaults to HIGH (0x77); if the SDO pin is tied to GND,
  switch to LOW (0x76) in the same menu
- Vdd connected to GPIO2 (`POWER_GPIO_PIN`), not directly to power — this
  example requires `POWER_IS_GPIO` enabled (see `sdkconfig.defaults`) so the
  sensors can be powered down between readings

## Build and flash

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

## Expected output

```
I (1234) env_async_example: Auto read started. Reading every 15000 ms for 120000 ms.
I (16789) env_sensor: Temperature: 22.41 °C, Humidity: 41.30 %RH, Pressure: 1013.25 hPa
I (16800) env_async_example: Last reading: Temperature: 22.41 °C, Humidity: 41.30 %RH, Pressure: 1013.25 hPa
...
I (121234) env_async_example: Auto read stopped.
```
