cc_library(
    name = "DmaController",
    srcs = ["DmaController.cc"],
    hdrs = ["DmaController.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":types",
        "//picolog",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_dma",
    ],
)

cc_library(
    name = "SpiController",
    srcs = ["SpiController.cc"],
    hdrs = ["SpiController.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":DmaController",
        ":types",
        "//picolog",
        "//tplp:rtos_util",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_spi",
    ],
)

cc_library(
    name = "I2cController",
    srcs = ["I2cController.cc"],
    hdrs = ["I2cController.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":types",
        "//picolog",
        "//picolog:status_macros",
        "//tplp:rtos_util",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_i2c",
    ],
)

cc_library(
    name = "types",
    srcs = ["types.cc"],
    hdrs = ["types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@NamedType",
    ],
)

cc_library(
    name = "WIP_dma",
    srcs = [],
    hdrs = ["dma.h"],
    deps = [
        ":types",
        "//picolog",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_dma",
    ],
)
