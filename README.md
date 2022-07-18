# Building
Install `bazel`, `gcc-arm-none-eabi`, `arm-none-eabi-newlib`.

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
- [ ] Create a front panel UI
  - [ ] Use [LVGL](https://lvgl.io)
- [ ] Get peripheral hardware running
  - [x] LCD display
  - [ ] Make sure LCD display is getting VCOM cleared on the appropriate schedule
  - [ ] Front panel buttons
  - [ ] Stepper drivers (use pico_stepper)
  - [ ] Laser module
  - [ ] Load cell reader (use hx711-arduino-library)
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
- [x] Get FreeRTOS running
- [x] Bazel-ify the build

# Board configuration/pins
## Front panel ribbon cable
- Pin 1 (red) - LCD Enable
- Pin 2 - LCD Chip Select (active high)
- Pin 3 - LCD MOSI
- Pin 4 - LCD SCLK
- Pin 5 - GND
- Pin 6 - V3.3
- Pin 7 - Btn Down (Open V3.3, Closed GND)
- Pin 8 - Btn Left (Open V3.3, Closed GND)
- Pin 9 - Btn Up (Open V3.3, Closed GND)
- Pin 10 - Btn Right (Open V3.3, Closed GND)

We might be able to save 2 pins by combining the button signals.

## Load cell wires
- Red - E+
- Black - E-
- Green - A-
- White - A+

# Reference Materials
- [What is the proper way to debounce a GPIO input?](https://raspberrypi.stackexchange.com/questions/118349/what-is-the-proper-way-to-debounce-a-gpio-input)
- [Hardware Design with the RP2040](https://www.mouser.com/pdfDocs/hardware-design-with-rp2040.pdf)
- [Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [FreeRTOS Kernel Quick Start Guide](https://www.freertos.org/FreeRTOS-quick-start-guide.html)
- [FreeRTOS RP2040 SMP demos](https://www.freertos.org/smp-demos-for-the-raspberry-pi-pico-board.html)
- [Use Arduino Libraries with the Rasperry Pi Pico C/C++ SDK](https://www.hackster.io/fhdm-dev/use-arduino-libraries-with-the-rasperry-pi-pico-c-c-sdk-eff55c)
