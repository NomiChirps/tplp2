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
    name = "dma",
    srcs = ["dma.cc"],
    hdrs = ["dma.h"],
    deps = [
        ":types",
        "@FreeRTOS",
    ],
)

cc_test(
    name = "dma_test",
    srcs = ["dma_test.cc"],
    deps = [
        ":dma",
        "@com_google_googletest//:gtest_main",
    ],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
)
