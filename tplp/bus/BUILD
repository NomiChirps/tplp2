cc_library(
    name = "dma",
    srcs = ["dma.cc"],
    hdrs = ["dma.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":types",
        "//picolog",
        "//tplp:util",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_dma",
    ],
)

cc_library(
    name = "spi",
    srcs = ["spi.cc"],
    hdrs = ["spi.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":dma",
        ":types",
        "//picolog",
        "//tplp:rtos_util",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_spi",
    ],
)

cc_library(
    name = "i2c",
    srcs = ["i2c.cc"],
    hdrs = ["i2c.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":dma",
        ":types",
        "//picolog",
        "//picolog:status_macros",
        "//tplp:rtos_util",
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
