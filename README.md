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
- [ ] Create a front panel UI
  - [x] Use [LVGL](https://lvgl.io)
  - [ ] runtime stats / logging screen
  - [ ] manual peripheral control screen
  - [ ] parameters screen
  - [ ] (later) print status / main screen ?
- [ ] Get peripheral hardware running
  - [ ] touchscreen input (SPI breakout)
  - [ ] Shiny new front panel display
  - [x] Sharp LCD display
  - [ ] Front panel buttons
  - [ ] Stepper drivers (use pico_stepper)
  - [ ] Laser module
  - [ ] Load cell reader https://github.com/endail/hx711-pico-c
  - [ ] Mirror motor (PWM control; still needs a driver circuit)
  - [ ] Mirror optointerrupter
  - [ ] MicroSD card reader
- [ ] Finish the electronics hardware
  - [x] install inverter on the Sharp LCD CS pin so it behaves like EVERY OTHER SPI DEVICE IN THE WORLD
  - [ ] Power everything from the 12v bus
  - [ ] Add bus capacitors
  - [ ] Install jumpers on the stepper drivers (to configure internal 5v power supply)
  - [ ] Stretch goal: add a WiFi module?
  - [ ] Stretch goal: add a pinhole photodiode for self-calibration and/or self-test
  - [ ] Transfer from breadboard to permaproto
- Nice-to-haves
  - [ ] Trap hard faults in picolog, so we can get stack traces for a nullptr dereference :)
    - https://github.com/yocto-8/yocto-8/blob/main/src/arch/pico/extmem/faulthandler.cpp#L12
    - https://forums.raspberrypi.com/viewtopic.php?t=318745
  - [ ] Why are we spending so much time in Tmr Svc?
    - it's actually not that much time now, without SMP.
  - [ ] move/redirect config headers to a config/ dir
  - [ ] consider removing the xTaskDelete() at the end of startup, and switching to heap_1.
  - [ ] generate & examine .map file for the firmware blob
  - [ ] use bloaty to find things to trim off the firmware size
  - [ ] create a lint.sh or something. to cover cc and bzl files
  - [ ] Vendor all 3rd party libraries
  - [ ] run blaze with layering_check; but need to do it on Linux because it requires CC=clang. might also need to add clang support to rules_pico.
- [x] please let's replace tplp_assert() with CHECK(), CHECK_NOTNULL(), etc. likewise DebugLog() -> LOG(INFO), VLOG(1), etc.
  - [ ] per-file VLOG toggles at compile time (use a constexpr function for string comparison, probably)
  - [ ] LOG level toggles at runtime
  - plan for eventually logging to Flash or SD instead of USB (i did not so plan)

# Board configuration/pins
## RP2040 pins
See also: https://learn.adafruit.com/adafruit-feather-rp2040-pico/pinouts

**Caution!** Numeric labels on the board do NOT necessarily match the GPIO number!

| Pin | Label | Assignment |
| - | - | - |
| GPIO02 | SDA | I2C1 SDA (STEMMA QT connector) |
| GPIO03 | SCL | I2C1 SCL (STEMMA QT connector) |
| GPIO04 | - | *Reserved by platform* |
| GPIO05 | - | *Reserved by platform* |
| GPIO06 | D4 | **available** |
| GPIO07 | D5 | **available** |
| GPIO08 | D6 | **available** |
| GPIO09 | D9 | **available** |
| GPIO10 | D10 | **available** |
| GPIO11 | D11 | **available** |
| GPIO12 | D12 | **available** |
| GPIO13 | D13 | **available** |
| GPIO14 | - | *Reserved by platform* |
| GPIO15 | - | *Reserved by platform* |
| GPIO16 | - | *Hardwired to Neopixel* |
| GPIO17 | - | *Reserved by playform* |
| GPIO18 | SCK | **available** |
| GPIO19 | MOSI | **available** |
| GPIO20 | MISO | **available** |
| GPIO21 | - | *Reserved by platform* |
| GPIO22 | - | *Reserved by platform* |
| GPIO23 | - | *Reserved by platform* |
| GPIO24 | D24 | HX3587 CS |
| GPIO25 | D25 | HX3587 DC |
| GPIO26 | A0 | SPI1 SCK |
| GPIO27 | A1 | SPI1 MOSI |
| GPIO28 | A2 | SPI1 MISO |
| GPIO29 | A3 | **available** |

| Bus | Devices |
| - | - |
| SPI0 | - |
| SPI1 | HX8357 |
| I2C1 | - |

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

# todos whomst done

- [x] spend a week fruitlessly debugging instability between FreeRTOS-SMP + pico-sdk before giving up on SMP in disgust. at least I learned a lot and ironed out a few other issues along the way.
- [x] vApplicationGetIdleTaskMemory mystery: why doesn't my definition clash with the rp2040 port's in idle_task_static_memory.c?
  - ah. because i needed alwayslink=1 in FreeRTOS. it's fine though, we don't need to add it. the local definition gets linked in just fine.
- [x] (vanity) rename FreeRTOS or FreeRTOS-Kernel to the same thing, so we can say e.g. "@FreeRTOS" instead of "@FreeRTOS-Kernel//:FreeRTOS"
- [x] Tune FreeRTOS
  - The verdict: we'll be using non-SMP with preemption. If we need the second core for something, we'll put it there manually.
  - [x] enable SMP/multicore
    - turns out this was a terrible idea.
  - [x] audit FreeRTOSConfig.h
  - [x] (friendship ended with Arduino) audit Arduino compat libraries for blocking delays? 
  - [x] (friendship ended with Arduino) use scope to verify reliability in the presence of Arduino libs
  - [x] write configASSERT handler (use the NeoPixel LED?)
- [x] use the genrule() trick to finally encapsulate FreeRTOS headers
- [x] use DMA for the Sharp LCD driver, just for funsies
- [x] figure out how to track TODO/FIXME/XXX in IDE
- [x] organize source code under src/ (I changed my mind later)
- [x] (it's not possible, i decided) refer to FreeRTOS headers with a prefix, if possible (actually it is)
- [x] Make sure we're not using floating point math anywhere (including dependencies): discovered via std::unordered_map that we panic if any floating point math is used
- [x] Reset to the bootloader without touching the board: magic 1200 baud connection
- [x] Get FreeRTOS running
- [x] Bazel-ify the build
