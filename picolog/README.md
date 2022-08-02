Experimental glog-style C++ logging library for the Raspberry Pi Pico under gcc + pico-sdk + FreeRTOS.

At present, does not function at all without the FreeRTOS scheduler running.

Not tested with FreeRTOS-SMP or with pico_multicore.

Add the `-mpoke-function-name` compiler flag to get function names in the backtrace for fatal errors.