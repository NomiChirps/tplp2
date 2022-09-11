# TPLP2

## Building

Install `bazel` (or `bazelisk`), `gcc-arm-none-eabi`, `arm-none-eabi-newlib`. Developed with `gcc-arm-12.1.0`.

```sh
# debug build
bazel build //tplp:firmware.uf2
# optimized build
bazel build //tplp:firmware.uf2 -c opt

# firmware blob is in bazel-bin
ls -lh bazel-bin/tplp/firmware.uf2
```

## Shopping list

- 2 more TB6612 drivers (1 for the other steppie, 1 for DC motors)

## TODO / Notes

- [ ] fix all instances of std::setw(). it only sticks for one output field...
- [ ] need a thingy for persistently saving global parameters / preferences / whatever you wanna call em.
  - use the MicroSD card behind the display. fat32?
- [ ] stepper driver: should coils be set to freewheel (LL) or short-brake (HH) at the top of each command cycle? maybe experiment with it.
- [ ] Begin() Setup() Init(), **PICK ONE**
- [ ] alright fine let's get StrCat in here sheeeeesh
- [ ] Milestone: Paper Tensioning
  - Be able to load paper and feed it forward and backward while maintaining correct tension.
  - [x] needs steppers working
  - [ ] needs load cell persistently calibrated & installed
    - [ ] calibrate/zero button on the load cell gui. and a save settings button.
  - [ ] needs a PID loop with all the accoutrements
    - i want a mathematically sound PID tuning UI
- UX Improvements
  - [ ] make the touch target for the settings back button bigger
  - [x] fix everything in the ui being scrollable
  - [x] numeric display of load cell value & raw value
  - [x] oops changed my mind about the load cell units. they're actually completely arbitrary.
- [ ] I2cController doesn't fail gracefully when a read times out!
- [ ] Create a front panel UI (assignee: wembly :)
  - [ ] some kind of display for non-fatal errors. modal dialog box? little icon in the corner to tap on and expand? idk.
  - [ ] fatal error handler to display the stack trace before crashing
    - will require support from picolog. make sure picolog prevents the crashed task from proceeding, and has a watchdog or something to finish crashing the device if the fatal error handler doesn't finish in time.
  - [ ] runtime stats / logging screen
  - [ ] (later) print status / job control / main screen ?
  - [x] manual motor control screen
  - [x] I2C bus scan
  - [x] virtual class interface for callbacks 'n' stuff
  - [x] Use [LVGL](https://lvgl.io)
- [ ] Get peripheral hardware running
  - [ ] Roller encoder
    - neglected to add this to the original hardware design ;(
  - [ ] Laser module (PWM control)
  - [ ] Mirror motor (PWM control; still needs a driver circuit)
  - [ ] Mirror optointerrupter
  - [ ] pinhole photodiode for self-calibration and/or self-test
  - [ ] MicroSD card reader
    - we may want two. the one in the display is very difficult to access, so it's more suited to being the internal storage.
    - alternatively, forget using SD cards to upload and go ahead with the wi-fi interface, haha
      - an embedded target and a command-line client tool, in the same project? time for bazel to shine ~
  - [x] Stepper drivers
  - [x] HX711 load cell reader
  - [x] Shiny new front panel display + LVGL driver
  - [x] touchscreen input (SPI breakout)
  - [x] Sharp LCD display + LVGL driver
- [ ] Finish the electronics hardware
  - [ ] Power everything from the 12v bus
    - remember to power pico thru VSYS, not VBUS, to avoid potentially sending power upstream to the usb host
  - [ ] Transfer from breadboard to permaproto, or ask wembly to whip up a pcb
  - [ ] supply 5V to HX711
  - [x] Add bus capacitors
- Nice-to-haves
  - [ ] audit codebase for portMAX_DELAY, CHECK, etc. and make sure everything that can fail "gracefully", does.
  - [ ] JTAG? is that a thing i can do?
  - [ ] rename all the lvgl_driver.cc to have distinct module names for VLOG
  - [ ] clear the display on boot. the random pixels look kinda cool but...
    - actually, no 'clear' command exists. best we can do is turn off the pixels (command 22h, pp.139), do a full memory write, and turn them back on.
  - [ ] TSC2007 lvgl driver should debounce, buffer, and interpolate
  - [ ] I2cController needs nonblocking operations, coalescing commands, general TLC
  - [ ] hook up tft backlight pin and add an adjustable brightness setting (PWM)
  - [ ] fix "bazel test //..." by adding target_compatible_with where appropriate
    - this may also involve fixing up rules_pico to make correct use of the defines PICO_NO_HARDWARE and PICO_ON_DEVICE
  - [ ] generate & examine .map file for the firmware blob
  - [ ] use bloaty to find things to trim off the firmware size
  - [ ] Vendor all 3rd party libraries
  - [ ] split out various things i'm proud of as their own librar(ies)
  - [ ] write stress tests for SpiController/I2cController. lotsa tasks, lotsa devices, all hammering away
- 32-bit aligned reads and writes are atomic. It would be nice to take advantage of that and avoid some locking where possible.

## Board configuration/pins

### Pinout / Peripherals

See `tplp/tplp_config.h` for pin and GPIO assignments.

| Bus  | Devices |
| ---- | ------- |
| SPI0 | -       |
| SPI1 | HX8357  |
| I2C0 | TSC2007 |
| I2C1 | -       |

### Stepper driver ribbon cable layout

- Pin 1 (red) - ENA
- Pin 2 - IN1
- Pin 3 - IN2
- Pin 4 - IN3
- Pin 5 - IN4
- Pin 6 - ENB

### Load cell wires

- Red - E+
- Black - E-
- Green - A-
- White - A+

## Debugging

Note that USB serial stdio is not available while debugging because pico-debug uses the USB port.

### Setup

1. You will need to have the standard GNU autotools/configure/make toolchain installed on your system, for building OpenOCD.
1. Make sure you'll have the appropriate permissions to access the pico-debug CMSIS-DAP USB device; this can be accomplished by adding a [udev rule](https://github.com/openocd-org/openocd/blob/master/contrib/60-openocd.rules) and adding yourself to the `plugdev` group mentioned in that rule (or edit the rule to use a different group).
1. If using Visual Studio Code, install the "Cortex-Debug" extension.
1. If not using Visual Studio Code, build OpenOCD: `cd third_party/openocd; bazel build //:openocd`. This will download and compile the appropriate version of OpenOCD within the Bazel sandbox, leaving the binaries in `third_party/openocd/bazel-bin/external/openocd/openocd/bin/openocd` and the scripts dir in `third_party/openocd/bazel-bin/external/openocd/openocd/share/openocd/scripts`. Isn't it funny how if you say openocd enough times it doesn't sound like a real word anymore?

### To debug under VSCode

See also: https://github.com/majbthrd/pico-debug/blob/master/howto/vscode1.md

1. Put Pico into bootloader mode (reset while holding BOOTSEL).
1. Run the `flash pico-debug` task in VSCode. This downloads a prebuilt pico-debug image and copies it to the device. Pico is now running the debugger on core 1.
1. Build tplp in debug mode by running the `[debug] build` task.
1. Launch the debug configuration in VSCode ("Run and Debug" tab). This is defined in `.vscode/launch.json`. This will also download and build OpenOCD first, if it hasn't already.
1. The program should automatically be loaded onto the Pico and start running in the debugger.
1. You can restart the program freely, but ending the debug session means you'll need to reflash pico-debug to start again.
   In order to make changes and restart, run the `[debug] build` task and then issue the `load` command in the debug console. Wait for the transfer to finish before hitting the Restart button (Ctrl+Shift+F5).

### To debug without IDE support

See also https://github.com/majbthrd/pico-debug/blob/master/howto/openocd.md.

1. Put Pico into bootloader mode (reset while holding BOOTSEL).
1. Run `bazel run //tools:flash-pico-debug`, which downloads a prebuilt [pico-debug](https://github.com/majbthrd/pico-debug) binary and copies it to the device. Pico is now running the debugger on core 1.
1. Start the OpenOCD server: `third_party/openocd/bazel-bin/external/openocd/openocd/bin/openocd -s third_party/openocd/bazel-bin/external/openocd/openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040-core0.cfg -c "transport select swd" -c "adapter speed 4000"`. It should connect to the Pico and wait for commands.
1. Build tplp in debug mode: `bazel build --config=picodebug //tplp:firmware.elf`
1. Start GDB: `arm-none-eabi-gdb bazel-bin/tplp/firmware.elf`.
1. Connect GDB to OpenOCD: `(gdb) target remote localhost:3333`.
1. Load the target into flash: `(gdb) load`.
1. Start it: `(gdb) monitor reset init`, `(gdb) continue`.

## Reference Materials

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

## todos whomst done

- [x] SpiController could use improvement - we're not getting full utilization of the bus
  - verdict: use the new DmaController with nonblocking operations where possible, and that's gonna be good enough.
- [x] Load cell time!
  - [x] hx711 pio driver
  - [x] gui for monitoring & calibration
  - [x] print some brackets to calibrate it against my digital scale
- [x] put the load cell period (100ms) in params.h
- [x] read about the hardware interpolators in rp2040
  - does what it says on the tin. good for linear scaling, affine texture mapping, that kinda thing.
- [x] Mission: DMA 9000
  - see new ideas in tplp/bus/dma.h !!!
  - SpiController needs to take advantage of the new Action stuff to be more nonblocking, and actually make use of the DmaManager queue.
    - to manage CS easily we might need before-Actions as well as after-Actions!
  - I2cController needs... uh... idk i don't want to look at it right now
  - after action report: the new ideas in tplp/bus/dma.h were waaaay too complicated and not actually a performance improvement. went back to the now-current implementation and fixed up SpiController/I2cController to take advantage of its async features.
    - I NEVER WANT TO LOOK AT DMA AGAIN >:(
- [x] Install jumpers on the stepper drivers (to configure internal 5v power supply)
- [x] figure out who's allowed to call xTaskCreate. centralize it.
- [x] forget the std::duration stuff.. really not worth the hassle.
- [x] rename SpiManager -> SpiController
- [x] insert appropriate delays in TSC2007 read cycle; we have noise problems
- [x] consider removing all deletions of FreeRTOS objects and switching to heap_1
  - I considered it. it's not gonna happen
- [x] move/redirect config headers to a config/ dir
- [x] (?!) display fails self-test during startup if usb-serial output not present
  - the issue was it needed some time to boot up after poweron before accepting commands
- [x] LVGL driver for HX8357
- [x] HX8357 self test :)
- [x] rework SpiController again again; it should be able to suport full duplex. instead of sendevent/receiveevent, let's have transferevent(naming???) which specifies optionally both buffers. I GUESS???? if both are specified, they would necessarily have to be the same length!
- [x] get a compile_commands going for //simulator
- [x] have FreeRTOS and LVGL running together in a simulator mode on the host, for UI development (and unit testing?!)
- [x] fix lvgl+simulator so it doesn't build SDL2 twice
- [x] it is time to reorganize the directory structure again
  - we've got /openocd, /lib, /picolog, /tplp... too many different kinds of things at the top level
- [x] bazel build for OpenOCD binary!
- [x] WONTFIX: run blaze with layering_check; but need to do it on Linux because it requires CC=clang. might also need to add clang support to rules_pico.
  - this definitely isn't gonna happen. it needs support from the toolchain definition and i don't want to research how to implement that.
- [x] fix compile_commands extraction, again (problem with lvgl build? maybe strip_include_prefix works correctly on Linux?)
- [x] Why are we spending so much time in Tmr Svc?
  - it's actually not that much time now, without SMP.
- [x] please let's replace tplp_assert() with CHECK(), CHECK_NOTNULL(), etc. likewise DebugLog() -> LOG(INFO), VLOG(1), etc.
  - Implemented "picolog" and broke it out into a library :)
- [x] switch to Linux because wow gosh heckie does windows suck for this kind of development
- [x] create a lint.sh or something. to cover cc and bzl files (using Trunk vscode extension)
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
