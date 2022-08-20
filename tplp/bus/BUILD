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
        ":dma_program",
        ":types",
        "//picolog",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_gpio",
    ],
)

cc_library(
    name = "dma_program",
    srcs = ["dma_program.cc"],
    hdrs = ["dma_program.h"],
    deps = [
        ":types",
        "//picolog",
        "@FreeRTOS",
    ],
)

cc_test(
    name = "dma_program_test",
    srcs = ["dma_program_test.cc"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":dma_program",
        "@com_google_googletest//:gtest_main",
    ],
)
