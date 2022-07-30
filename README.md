# Building
Install `bazel`, `gcc-arm-none-eabi`, `arm-none-eabi-newlib`.

Developed with `gcc-arm-11.2-2022.02`.

On Windows, requires bazel >= 6.0.0-pre.20220630.1 (due to mystery bug, present in 5.2.0) and gcc-arm-none-eabi >= 10.3.1 (due to [bug](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=95253)).

```sh
# debug build
bazel build //tplp:firmware
# optimized build
bazel build //tplp:firmware -c opt

# firmware blob is in bazel-bin
ls -lh bazel-bin/tplp/firmware.uf2
```

# TODO / Notes
- [ ] run blaze with layering_check; but need to do it on Linux because it requires CC=clang. might also need to add clang support to rules_pico.
- [ ] move/redirect config headers to a config/ dir
- [ ] Why are we spending so much time in Tmr Svc?
- [ ] (!!!) vApplicationGetIdleTaskMemory mystery: why doesn't my definition clash with the rp2040 port's in idle_task_static_memory.c?
- [ ] consider removing the xTaskDelete() at the end of startup, and switching to heap_1.
- [ ] generate & examine .map file for the firmware blob
- [ ] use bloaty to find things to trim off the firmware size
- [ ] find or make a proper logging framework so i can selectively enable tracing stuff.
- [ ] create a lint.sh or something. to cover cc and bzl files
- [ ] Vendor all 3rd party libraries
- [ ] Create a front panel UI
  - [x] Use [LVGL](https://lvgl.io)
- [ ] Get peripheral hardware running
  - [ ] Shiny new front panel display with touchscreen
  - [x] Sharp LCD display
  - [ ] Front panel buttons
  - [ ] Stepper drivers (use pico_stepper)
  - [ ] Laser module
  - [ ] Load cell reader https://github.com/endail/hx711-pico-c
  - [ ] Mirror motor (PWM control; still needs a driver circuit)
  - [ ] Mirror optointerrupter
  - [ ] MicroSD card reader (use [SD](https://github.com/arduino-libraries/SD/tree/a64c2bd907460dd01cef07fff003550cfcae0119))
- [ ] Finish the electronics hardware
  - [x] install inverter on the Sharp LCD CS pin so it behaves like EVERY OTHER SPI DEVICE IN THE WORLD
  - [ ] Power everything from the 12v bus
  - [ ] Add bus capacitors
  - [ ] Install jumpers on the stepper drivers (to configure internal 5v power supply)
  - [ ] Stretch goal: add a WiFi module?
  - [ ] Stretch goal: add a pinhole photodiode for self-calibration and/or self-test
  - [ ] Transfer from breadboard to permaproto

## Done

- [x] (vanity) rename FreeRTOS or FreeRTOS-Kernel to the same thing, so we can say e.g. "@FreeRTOS" instead of "@FreeRTOS-Kernel//:FreeRTOS"
- [x] I notice Tmr Svc task has only 56 bytes of stack free. what if that's why we're dying? when a callback runs that uses more stack?
  - do we even need software timers? let's try disabling them.
    aha, i see! software timers are used in the RP2040 port for pico sync interop: in the FIFO interrupt handler (on each core), and when unlocking a mutex from an interrupt handler.
  - so what are we doing that's using so much FIFO and/or mutex?
  - increased the stack size; it's fine now.
- [x] Tune FreeRTOS
  - The verdict: we'll be using non-SMP with preemption. If we need the second core for something, we'll put it there manually.
  - [x] enable SMP/multicore
    - turns out this was a terrible idea.
  - [x] audit FreeRTOSConfig.h
  - [x] (friendship ended with Arduino) audit Arduino compat libraries for blocking delays? 
  - [x] (friendship ended with Arduino) use scope to verify reliability in the presence of Arduino libs
  - [x] write configASSERT handler (use the NeoPixel LED?)
- [x] figure out why stdio locking in logging.h is *STILL* flaky
  - The answer was: should have used FreeRTOS mutex *with* properly configured Pico-sync-multicore interop, so as to lock out other tasks AND the other core.
  - [x] check pico sync interop impl
  - [x] check if freertos thread-locals are per core (they are not)
- [x] use the genrule() trick to finally encapsulate FreeRTOS headers?
- [x] use DMA for the Sharp LCD driver, just for funsies
- [x] figure out how to track TODO/FIXME/XXX in IDE
- [x] organize source code under src/ (I changed my mind later)
- [x] (it's not possible, i decided) refer to FreeRTOS headers with a prefix, if possible (actually it is)
- [x] perhaps pico/lock_core.h macros need to be set up for FreeRTOS! that might explain why stdio seemed so flaky...
- [x] Make sure we're not using floating point math anywhere (including dependencies): discovered via std::unordered_map that we panic if any floating point math is used
- [x] Reset to the bootloader without touching the board: magic 1200 baud connection
- [x] Get FreeRTOS running
- [x] Bazel-ify the build

# Board configuration/pins
## RP2040 pins
See also: https://learn.adafruit.com/adafruit-feather-rp2040-pico/pinouts

**Caution!** Numeric labels on the board do NOT necessarily match the GPIO number!

| Pin | Label | Assignment |
| --- | ----- |---------- |
| GPIO02 | SDA | SDA1: I2C data |
| GPIO03 | SCL | SCL1: I2C clock |
| GPIO04 | - | *Reserved by platform* |
| GPIO05 | - | *Reserved by platform* |
| GPIO06 | D4 | **available** |
| GPIO07 | D5 | **available** |
| GPIO08 | D6 | LCD CS |
| GPIO09 | D9 | (tmp) Red button |
| GPIO10 | D10 | **available** |
| GPIO11 | D11 | **available** |
| GPIO12 | D12 | **available** |
| GPIO13 | D13 | **available** |
| GPIO14 | - | *Reserved by platform* |
| GPIO15 | - | *Reserved by platform* |
| GPIO16 | - | *Hardwired to Neopixel* |
| GPIO17 | - | *Reserved by playform* |
| GPIO18 | SCK | SCK0: LCK SCLK (and other peripherals later) |
| GPIO19 | MOSI | TX0: LCD MOSI (and other peripherals later) |
| GPIO20 | MISO | **available** |
| GPIO21 | - | *Reserved by platform* |
| GPIO22 | - | *Reserved by platform* |
| GPIO23 | - | *Reserved by platform* |
| GPIO24 | D24 | **available** |
| GPIO25 | D25 | **available** |
| GPIO26 | A0 | **available** |
| GPIO27 | A1 | **available** |
| GPIO28 | A2 | **available** |
| GPIO29 | A3 | **available** |

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
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) (the good stuff)
- [Raspberry Pi Pico SDK PDF docs](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) (more info than the HTML version)
- [Raspberry Pi Pico SDK HTML docs](https://raspberrypi.github.io/pico-sdk-doxygen/) (bare API reference)
- [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [What is the proper way to debounce a GPIO input?](https://raspberrypi.stackexchange.com/questions/118349/what-is-the-proper-way-to-debounce-a-gpio-input)
- [Hardware Design with the RP2040](https://www.mouser.com/pdfDocs/hardware-design-with-rp2040.pdf)
- [FreeRTOS Kernel Quick Start Guide](https://www.freertos.org/FreeRTOS-quick-start-guide.html)
- [FreeRTOS RP2040 SMP demos](https://www.freertos.org/smp-demos-for-the-raspberry-pi-pico-board.html)
- [Use Arduino Libraries with the Rasperry Pi Pico C/C++ SDK](https://www.hackster.io/fhdm-dev/use-arduino-libraries-with-the-rasperry-pi-pico-c-c-sdk-eff55c)
- [How to define and measure clock jitter](https://www.sitime.com/api/gated/AN10007-Jitter-and-measurement.pdf)
- [Bit Twiddling Hacks](http://graphics.stanford.edu/~seander/bithacks.html) (the one and only)