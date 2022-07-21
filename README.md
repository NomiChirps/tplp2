# Building
Install `bazel`, `gcc-arm-none-eabi`, `arm-none-eabi-newlib`.

On Windows, requires bazel >= 6.0.0-pre.20220630.1 (due to mystery bug, present in 5.2.0) and gcc-arm-none-eabi >= 10.3.1 (due to [bug](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=95253)).

```sh
# debug build
bazel build :hello.uf2
# optimized build
bazel build :hello.uf2 -c opt

# firmware blob is in bazel-bin
ls -lh bazel-bin/hello.uf2
```

# Flashing
Put the RP2040 into bootloader mode by holding BOOTSEL while pressing RESET. Copy the .uf2 file to the USB mass storage device that appears.

# TODO
- [ ] Vendor all 3rd party libraries
- [ ] Tune FreeRTOS
  - [ ] audit FreeRTOSConfig.h
  - [ ] audit Arduino compat libraries for blocking delays?
  - [ ] use scope to verify reliability in the presence of Arduino libs
  - [ ] write configASSERT handler (use the NeoPixel LED?)
- [ ] Create a front panel UI
  - [ ] Use [LVGL](https://lvgl.io)
- [ ] Get peripheral hardware running
  - [x] LCD display
  - [ ] Make sure LCD display is getting VCOM cleared on the appropriate schedule
  - [ ] Front panel buttons
  - [ ] Stepper drivers (use pico_stepper)
  - [ ] Laser module
  - [ ] Load cell reader https://github.com/endail/hx711-pico-c
  - [ ] Mirror motor (PWM control; still needs a driver circuit)
  - [ ] Mirror optointerrupter
  - [ ] MicroSD card reader (use [SD](https://github.com/arduino-libraries/SD/tree/a64c2bd907460dd01cef07fff003550cfcae0119))
- [ ] Finish the electronics hardware
  - [ ] Power everything from the 12v bus
  - [ ] Add bus capacitors
  - [ ] Install jumpers on the stepper drivers (to configure internal 5v power supply)
  - [ ] Stretch goal: add a WiFi module?
  - [ ] Stretch goal: add a pinhole photodiode for self-calibration and/or self-test
  - [ ] Transfer from breadboard to permaproto
- [ ] Make sure we're not using floating point math anywhere (including dependencies)
- [x] Reset to the bootloader without touching the board: magic 1200 baud connection
- [x] Get FreeRTOS running
- [x] Bazel-ify the build

# Board configuration/pins
## RP2040 pins
See also: https://learn.adafruit.com/adafruit-feather-rp2040-pico/pinouts

| Pin | Assignment |
| --- | ---------- |
| GPIO02/SDA1 | available (future I2C data) |
| GPIO03/SCL1 | available (future I2C clock) |
| GPIO04 | *Reserved by platform* |
| GPIO05 | *Reserved by platform* |
| GPIO06 | LCD CS |
| GPIO07 | available |
| GPIO08 | available |
| GPIO09 | (tmp) Red button = jump to bootloader  |
| GPIO10 | available |
| GPIO11 | available |
| GPIO12 | available |
| GPIO13 | available |
| GPIO14 | *Reserved by platform* |
| GPIO15 | *Reserved by platform* |
| GPIO16 | *Hardwired to Neopixel* |
| GPIO17 | *Reserved by playform* |
| GPIO18/SCK0 | LCK SCLK (and other peripherals later) |
| GPIO19/TX0 | LCD MOSI (and other peripherals later) |
| GPIO20 | available |
| GPIO21 | *Reserved by platform* |
| GPIO22 | *Reserved by platform* |
| GPIO23 | *Reserved by platform* |
| GPIO24 | available |
| GPIO25 | available |
| GPIO26 | available (ADC0) |
| GPIO27 | available (ADC1) |
| GPIO28 | available (ADC2) |
| GPIO29 | available (ADC3) |

GPIO2 and GPIO3 are brought out via the [STEMMA QT](https://learn.adafruit.com/introducing-adafruit-stemma-qt) connector.

Note: "while many pins are capable of I2C, SPI and UART, you can only do two at a
time, and only on separate peripherals, 0 and 1." This means we can use e.g. SCK0 and SCL1
but not SCK0 and SCL0 (if we want hardware acceleration).

## Front panel ribbon cable layout
- Pin 1 (red) - LCD Enable (should be left open)
- Pin 2 - LCD SPI CS (active high)
- Pin 3 - LCD SPI MOSI
- Pin 4 - LCD SPI SCLK
- Pin 5 - GND
- Pin 6 - V3.3
- Pin 7 - Btn Down (Open V3.3, Closed GND)
- Pin 8 - Btn Left (Open V3.3, Closed GND)
- Pin 9 - Btn Up (Open V3.3, Closed GND)
- Pin 10 - Btn Right (Open V3.3, Closed GND)

We might be able to save 2 pins by combining the button signals.

## Stepper driver ribbon cable layout
- Pin 1 (red) - ENA
- Pin 2 - IN1
- Pin 3 - IN2
- Pin 4 - IN3
- Pin 5 - IN4
- Pin 6 - ENB

## Load cell wires
- Red - E+
- Black - E-
- Green - A-
- White - A+

# Reference Materials
- [Raspberry Pi Pico SDK (HTML docs)](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [What is the proper way to debounce a GPIO input?](https://raspberrypi.stackexchange.com/questions/118349/what-is-the-proper-way-to-debounce-a-gpio-input)
- [Hardware Design with the RP2040](https://www.mouser.com/pdfDocs/hardware-design-with-rp2040.pdf)
- [Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [Raspberry Pi Pico C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
- [FreeRTOS Kernel Quick Start Guide](https://www.freertos.org/FreeRTOS-quick-start-guide.html)
- [FreeRTOS RP2040 SMP demos](https://www.freertos.org/smp-demos-for-the-raspberry-pi-pico-board.html)
- [Use Arduino Libraries with the Rasperry Pi Pico C/C++ SDK](https://www.hackster.io/fhdm-dev/use-arduino-libraries-with-the-rasperry-pi-pico-c-c-sdk-eff55c)
