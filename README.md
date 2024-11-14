# ST LPS22HB pressure sensor library for Arduino

This is an Arduino library for the [ST LPS22HB](https://www.st.com/en/mems-and-sensors/lps22hb.html) pressure sensor
like it is used in the [Arduino Nano 33 BLE Sense](https://www.st.com/en/development-tools/nano-33-ble-sense.html).
The library supports several ways of reading the sensor data,
including the provided mechanisms of the sensor like One-Shot mode, Continuous mode and also Interrupt generation.
A main goal of the library is to provide a simple and also efficent way of reading the sensor data.

## Data sheets
- [Website overview](https://www.st.com/en/mems-and-sensors/lps22hb.html#overview)
- [Specification](https://www.st.com/resource/en/datasheet/lps22hb.pdf)
- [Application note](https://www.st.com/resource/en/application_note/an4833-measuring-pressure-data-from-sts-lps22hb-digital-pressure-sensor-stmicroelectronics.pdf)
- [Tech note](https://www.st.com/resource/en/technical_note/tn1229-how-to-interpret-pressure-and-temperature-readings-in-the-lps22hb-pressure-sensor-stmicroelectronics.pdf)

## Usage

### Object creation

```c++
#include "st_lps22hb.h"
andrgrue::sensor::st_lps22hb Pressure(Wire1);

float pressure;
float temperature;
andrgrue::sensor::st_lps22hb::Data pressure_data;
andrgrue::sensor::st_lps22hb::Data temperature_data;
volatile bool interruptFlag = false;
```

### Interrupt handler

```c++
void pressure_interrupt_handler() {
  interruptFlag = true;
}
```

### Setup

```c++
void setup() {

  // wire must be initialized first
  Wire1.begin();
  Wire1.setClock(400000);

  // Option 1. One Shot mode - poll sensor measurements
  if (Pressure.initialize(andrgrue::sensor::st_lps22hb::Rate::RATE_ONE_SHOOT)) {
  // Option 2. Continuous mode - polling of data ready info in status register to trigger sensor measurements
  if (Pressure.initialize(andrgrue::sensor::st_lps22hb::Rate::RATE_50HZ)) {
  // Option 3. Continuous mode - polling of interrupt pin to trigger sensor measurements
  // need to specify interrupt pin
  if (Pressure.initialize(andrgrue::sensor::st_lps22hb::Rate::RATE_50HZ, andrgrue::sensor::st_lps22hb::LowPassFilter::LPF_9, p12)) {
  // Option 4. Continuous mode - use interrupt handler to trigger sensor measurements
  // need to specify interrupt pin and interrupt handler
  if (Pressure.initialize(andrgrue::sensor::st_lps22hb::Rate::RATE_50HZ, andrgrue::sensor::st_lps22hb::LowPassFilter::LPF_20, p12, pressure_interrupt_handler)) {
    Serial.println("LPS22HB Pressure Sensor found.");
  }
  else {
    Serial.println("LPS22HB Pressure Sensor not found.");
    while (true) {  }   // loop forever
  }

}
```

### Loop

```c++
void loop() {

  // Option 1.
  pressure = Pressure.pressure();
  temperature = Pressure.temperature();

  // Option 2/3.
  if (Pressure.dataAvailable()) {
    pressure = Pressure.pressure();
    temperature = Pressure.temperature();
  }

  // Option 4.
  if (interruptFlag) {
    pressure = Pressure.pressure();
    temperature = Pressure.temperature();
    interruptFlag = false;
  }

}
```

## Credits

This project was inspired and includes several elements from the following projects.
Special thanks to the authors.

 - [Reefwing-LPS22HB](https://github.com/Reefwing-Software/Reefwing-LPS22HB)
 - [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)

## License

Copyright © 2024, André Grüttner. All rights reserved.

This project is licensed under the GNU Lesser General Public License v2.1 (LGPL-2.1).
You may use, modify, and distribute this software under the terms of the LGPL-2.1 license.
See the [LICENSE](./LICENSE.txt) file for details, or visit [GNU’s official LGPL-2.1 page](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html) for the full license text.
