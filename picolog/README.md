# Picolog

Experimental, ambitiously-named [glog-style](https://github.com/google/glog) C++ logging library for the Raspberry Pi Pico under gcc + pico-sdk + FreeRTOS.

At present, does not function at all without the FreeRTOS scheduler running.

Not tested with FreeRTOS-SMP or with pico_multicore.

Add the `-mpoke-function-name` compiler flag to get function names in the backtrace for fatal errors.

## Wishlist

- [ ] arena storage for the non-inlined StatusRep. or maybe we can get rid of it and require that the message be a compile-time constant...? or even better, provide a special case to make that path alloc-free.
- StatusOr would be nice but not essential
- Maybe have a separate task handling FATAL errors so it can be given the highest priority.
- [ ] arena storage for LogMessageData to avoid malloc (still requires mutex)
- [ ] DCHECK
- [ ] LOG level toggles at runtime, maybe per-task and/or per-module
- [ ] soften the hard dependency on pico-sdk, so we can run tests and use it in the simulator, with FreeRTOS's posix port
- [ ] ^-- leads to: unit/etc tests
- [ ] Trap hard faults so we can get stack traces for a nullptr dereference :)
  - https://github.com/yocto-8/yocto-8/blob/main/src/arch/pico/extmem/faulthandler.cpp#L12
  - https://forums.raspberrypi.com/viewtopic.php?t=318745
- [ ] support other log targets than stdout, e.g. flash, sd, or custom.
- [x] Status object and CHECK_OK
- [x] per-file VLOG toggles at compile time (use a constexpr function for string comparison with `__FILE__`, probably)
- [x] test for VLOG_IS_ON
