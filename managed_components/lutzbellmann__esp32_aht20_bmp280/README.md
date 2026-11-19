# esp32_aht20_bmp280

ESP-IDF component for reading temperature, humidity, and pressure from an
**AHT20** + **BMP280** sensor pair over I2C, using the ESP-IDF I2C *master*
driver (`driver/i2c_master.h`).

Requires **ESP-IDF >= 5.2**.

## Installation

```bash
idf.py add-dependency "lutzbellmann/esp32_aht20_bmp280^0.2.2"
```

Or add it manually to your `main/idf_component.yml`:

```yaml
dependencies:
  lutzbellmann/esp32_aht20_bmp280: "^0.2.2"
```

## Configuration

I2C pins, the optional power GPIO, and the BMP280 I2C address are set via
`idf.py menuconfig` → "AHT20+BMP280 Sensor configuration", not via function
arguments:

| Option | Default | Notes |
|--------|---------|-------|
| `I2C_SDA_PIN` / `I2C_SCL_PIN` | GPIO4 / GPIO5 | any free GPIOs |
| `I2C_BUS` | 0 | I2C port number |
| BMP280 I2C address | HIGH (`0x77`) | switch to LOW (`0x76`) if the BMP280 SDO pin is tied to GND; most marketplace modules use HIGH |
| `POWER_IS_GPIO` | disabled | enable if the sensors' Vdd is switched through a GPIO instead of being wired directly to power |
| `POWER_GPIO_PIN` | GPIO2 | only used when `POWER_IS_GPIO` is enabled |

AHT20's I2C address is fixed at `0x38`.

## Usage

```c
#include "env_sensor.h"

i2c_master_bus_handle_t bus;
ESP_ERROR_CHECK(env_sensor_create_bus(&bus));

env_sensor_t sensor;
ESP_ERROR_CHECK(env_sensor_init(bus, &sensor));

env_sensor_reading_t reading;
ESP_ERROR_CHECK(env_sensor_read(&sensor, &reading));
printf("%.2f°C  %.2f%%RH  %.2fhPa\n",
       reading.temperature, reading.humidity, reading.pressure);
```

See [`examples/basic`](examples/basic) for a complete, runnable project, or
[`examples/env_async`](examples/env_async) for background auto-reading with
power-down between samples.

## API

| Function | Description |
|----------|--------------|
| `env_sensor_create_bus()` | Create an I2C master bus on the configured pins (skip if a bus already exists) |
| `env_sensor_init()` | Probe and initialize AHT20 + BMP280 on an existing bus |
| `env_sensor_read()` | Trigger a measurement and return temperature, humidity, and pressure |
| `env_sensor_poweron()` / `env_sensor_poweroff()` | Switch sensor power via `POWER_GPIO_PIN` (requires `POWER_IS_GPIO`) |
| `env_sensor_init_auto_read()` | Start a background task that powers, reads, and powers down the sensors on a fixed interval, storing the result in `env_sensor_last_reading` (requires `POWER_IS_GPIO`) |
| `env_sensor_stop_auto_read()` | Stop the background task started by `env_sensor_init_auto_read()` (requires `POWER_IS_GPIO`) |

## License

Apache-2.0, see [LICENSE](LICENSE).
