package(default_visibility = ["@FreeRTOS-Kernel//:__subpackages__"])

cc_library(
    # Must be included at the end of the user's FreeRTOSConfig.h
    name = "rp2040_config",
    hdrs = ["include/rp2040_config.h"],
    include_prefix = "FreeRTOS",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    # Must be specified as PICO_CONFIG_RTOS_ADAPTER_HEADER to pico-sdk.
    name = "freertos_sdk_config",
    hdrs = ["include/freertos_sdk_config.h"],
    include_prefix = "FreeRTOS",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        ":rp2040_config_bareinclude",
        "@FreeRTOS//config:FreeRTOSConfig",
    ],
)

cc_library(
    name = "port_impl",
    srcs = [
        "idle_task_static_memory.c",
        "port.c",
    ],
    implementation_deps = [
        "@FreeRTOS-Kernel//:FreeRTOS-Kernel-headers-bareinclude",
        ":rp2040_config_bareinclude",
        ":config_check_1",
        ":config_check_2",
    ],
    deps = [
        "@FreeRTOS-Kernel",
        "@rules_pico//pico:hardware_clocks",
        "@rules_pico//pico:hardware_exception",
        "@rules_pico//pico:hardware_irq",
        "@rules_pico//pico:pico_multicore",
        "@rules_pico//pico:pico_stdlib",
        "@rules_pico//pico:pico_sync",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "@platforms//cpu:armv6-m",
    ],
    # Needed for idle_task_static_memory.c
    alwayslink = 1,
)

cc_library(
    name = "portmacro",
    hdrs = [
        "include/portmacro.h",
    ],
    include_prefix = "FreeRTOS",
    strip_include_prefix = "include",
    deps = [
        # This dependency could be narrower, but rules_pico doesn't expose anything smaller at the time of writing. It's not that important anyway.
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "rp2040_config_bareinclude",
    hdrs = ["include/rp2040_config.h"],
    strip_include_prefix = "include",
)

genrule(
    name = "port_srcs",
    srcs = ["@FreeRTOS-Kernel//:GCC_RP2040_all_srcs"],
    outs = [
        "idle_task_static_memory.c",
        "port.c",
    ],
    cmd_bash = "mkdir -p $(RULEDIR) && cp $(SRCS) $(RULEDIR)",
)

genrule(
    name = "port_hdrs",
    srcs = ["@FreeRTOS-Kernel//:GCC_RP2040_all_srcs"],
    outs = [
        "include/freertos_sdk_config.h",
        "include/portmacro.h",
        "include/rp2040_config.h",
    ],
    cmd_bash = "mkdir -p $(RULEDIR)/include && cp $(SRCS) $(RULEDIR)/include",
)

cc_library(
    name = "config_check_1",
    srcs = ["config_check_1.c"],
    implementation_deps = ["@FreeRTOS//config:FreeRTOSConfig"],
)

cc_library(
    name = "config_check_2",
    srcs = ["config_check_2.c"],
    implementation_deps = ["@rules_pico//pico:pico_stdlib"],
)
