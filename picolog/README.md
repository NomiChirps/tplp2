# Picolog

Experimental, ambitiously-named [glog-style](https://github.com/google/glog) C++ logging library for the Raspberry Pi Pico under gcc + pico-sdk + FreeRTOS.

At present, does not function at all without the FreeRTOS scheduler running.

Not tested with FreeRTOS-SMP or with pico_multicore.

Add the `-mpoke-function-name` compiler flag to get function names in the backtrace for fatal errors.

## Wishlist

- [ ] per-file VLOG toggles at compile time (use a constexpr function for string comparison with `__FILE__`, probably)
- [ ] LOG level toggles at runtime
- [ ] make a "HAL" so we can run using _either_ FreeRTOS or pico-sdk sync primitives
- [ ] Trap hard faults so we can get stack traces for a nullptr dereference :)
  - https://github.com/yocto-8/yocto-8/blob/main/src/arch/pico/extmem/faulthandler.cpp#L12
  - https://forums.raspberrypi.com/viewtopic.php?t=318745
- [ ] tests????
- [ ] support other log targets than stdout, e.g. flash, sd, or custom.
