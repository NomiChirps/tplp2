# Picolog

Experimental, ambitiously-named [glog-style](https://github.com/google/glog) C++ logging library for the Raspberry Pi Pico under gcc + pico-sdk + FreeRTOS.

Logs are collected in a FreeRTOS queue and flushed to the output asynchronously; unless the FreeRTOS scheduler hasn't been started yet, in which case they are processed immediately.

Not tested with FreeRTOS-SMP or with pico_multicore.

Add the `-mpoke-function-name` compiler flag to get function names in the backtrace for fatal errors.

## Wishlist

- [ ] track the maximum timestamp as logs are flushed, and make it visually obvious if it ever goes backwards (due to a task being interrupted while logging)
- [ ] arena storage for the non-inlined StatusRep. or maybe we can get rid of it and require that the message be a compile-time constant...? or even better, provide a special case to make that path alloc-free.
- StatusOr would be nice but not essential
- Maybe have a separate task handling FATAL errors so it can be given the highest priority.
- [ ] arena storage for LogMessageData to avoid malloc (still requires mutex)
- [ ] DCHECK
- [ ] LOG level toggles at runtime, maybe per-task and/or per-module
- [ ] ^-- leads to: unit/etc tests
- [ ] Trap hard faults so we can get stack traces for a nullptr dereference :)
  - https://github.com/yocto-8/yocto-8/blob/main/src/arch/pico/extmem/faulthandler.cpp#L12
  - https://forums.raspberrypi.com/viewtopic.php?t=318745
- [ ] support other log targets than stdout, e.g. flash, sd, or custom.
- [x] support immediate logging before the FreeRTOS scheduler starts
- [x] soften the hard dependency on pico-sdk, so we can run tests and use it in the simulator, with FreeRTOS's posix port
- [x] Status object and CHECK_OK
- [x] per-file VLOG toggles at compile time (use a constexpr function for string comparison with `__FILE__`, probably)
- [x] test for VLOG_IS_ON

## Hard Fault recovery, maybe

See section B1.5.6 of the ARMv6-M reference manual.

When pushing context to the stack, the hardware saves eight 32-bit words, comprising xPSR, ReturnAddress(), LR(R14), R12, R3, R2, R1, and R0.

B1.5.8 Exception return behavior
| EXC_RETURN | Behavior |
| - | - |
| 0xFFFFFFF1 | Return to Handler Mode. Exception return gets state from the Main stack. On return execution uses the Main Stack. |
| 0xFFFFFFF9 | Return to Thread Mode. Exception return gets state from the Main stack. On return execution uses the Main Stack. |
| 0xFFFFFFFD | Return to Thread Mode. Exception return gets state from the Process stack. On return execution uses the Process Stack. |

In the case of a memory fault when recovering the xPSR, if the EXC_RETURN value indicates that the exception return:

- must use the Main stack, SP_main, a lockup at HardFault priority occurs.
- must use the Process stack, SP_process, a Pending HardFault exception is generated

hmmmm

- what is xPSR?
- exception return operation checks SCR.SLEEPONEXIT
- pg. 200 is the handler pseudocode we must satisfy in order to restore into a normal stack
- also check out the freertos rp2040 port implementation!! maybe we can crib most of it off that.

"In ARMv6-M, faults are considered fatal. The only fault status information provided on entry to the HardFault
handler is the EXC_RETURN value, that indicates whether the fault originated from Thread or Handler mode. The
fault handler returns according to the rules defined in ReturnAddress(), see Exception entry behavior on
page B1-196 for details"

"While ARMv6-M does not provide
fault status information to the HardFault handler, it does permit the handler to perform an exception return and
continue execution, in cases where software has the ability to recover from the fault situation"

" Setting the AIRCR.SYSRESETREQ control bit to 1 requests a reset by an
external system resource. The system components that are reset by this request are IMPLEMENTATION DEFINED. A
Local reset is required as part of a system reset request.
Setting the SYSRESETREQ bit to 1 does not guarantee that the reset takes place immediately. A typical code
sequence to synchronize reset following a write to the relevant control bit is:"

```
     DSB;
Loop B Loop;
```

-> Could we have the task jump from the faulting instruction to a different function instead? One that calls vTaskSuspend(nullptr) forever. See: vPortStartFirstTask().

http://fastbitlab.com/free-rtos/ has some hints.
