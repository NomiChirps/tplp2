# GCC_RP2040

This port requires the following flags in .bazelrc in order to inject important configuration changes related to mutexes and spinlocks.

The `configSUPPORT_PICO_SYNC_INTEROP` and `configSUPPORT_PICO_TIME_INTEROP` defines can be used to control some of this behavior from your FreeRTOSConfig.h. They are both enabled by default. See rp2040_config.h in the FreeRTOS-Kernel source tree for details.

```bazel
build --@rules_pico//pico/config:rtos_adapter_enable=True
build --@rules_pico//pico/config:rtos_adapter_header=@FreeRTOS//ports/GCC_RP2040:freertos_sdk_config
build --@rules_pico//pico/config:rtos_adapter_header_name=FreeRTOS/freertos_sdk_config.h
```
