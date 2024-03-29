load("@rules_pico//pico:defs.bzl", "pico_add_map_output", "pico_add_uf2_output", "pico_binary", "pico_pio_header")

package(default_visibility = ["//tplp:__subpackages__"])

pico_binary(
    name = "firmware.elf",
    srcs = [],
    deps = [
        ":hook_impl",
        "//tplp/main",
    ],
)

cc_library(
    name = "pid",
    srcs = ["pid.cc"],
    hdrs = ["pid.h"],
    deps = [
        "//tplp:numbers",
        "//tplp:util",
        "//tplp/config:params",
    ],
)

cc_test(
    name = "pid_test",
    srcs = ["pid_test.cc"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":pid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "interrupts",
    hdrs = ["interrupts.h"],
    deps = [
        ":util",
        "@rules_pico//pico:hardware_timer",
    ],
)

cc_library(
    name = "numbers",
    hdrs = ["numbers.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//absl/strings",
    ],
)

cc_library(
    name = "paper_controller",
    srcs = ["paper_controller.cc"],
    hdrs = ["paper_controller.h"],
    deps = [
        ":interrupts",
        ":numbers",
        ":pid",
        "//picolog",
        "//tplp/config:constants",
        "//tplp/config:params",
        "//tplp/hx711",
        "//tplp/motor:stepper",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_timer",
    ],
)

cc_library(
    name = "hook_impl",
    srcs = ["hook_impl.cc"],
    deps = [
        "//picolog",
        "@FreeRTOS",
        "@rules_pico//pico:pico_time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "RuntimeStats",
    srcs = ["RuntimeStats.cc"],
    hdrs = ["RuntimeStats.h"],
    deps = [
        ":rtos_util",
        "//picolog",
        "@FreeRTOS",
        "@lvgl",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
    deps = [
        "@NamedType",
    ],
)

pico_add_uf2_output(
    name = "firmware.uf2",
    input = "firmware.elf",
)

# TODO: check this out
pico_add_map_output(
    name = "firmware.map",
    input = "firmware.elf",
)

pico_pio_header(
    name = "ws2812.pio.h",
    input = "ws2812.pio",
)

cc_library(
    name = "ws2812",
    srcs = [
        "ws2812.cc",
        "ws2812.pio.h",
    ],
    hdrs = [
        "ws2812.h",
    ],
    deps = [
        "@rules_pico//pico:hardware_clocks",
        "@rules_pico//pico:hardware_pio",
    ],
)

cc_library(
    name = "rtos_util",
    srcs = ["rtos_util.cc"],
    hdrs = ["rtos_util.h"],
    deps = [
        ":numbers",
        "//picolog",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "TplpInterfaceImpl",
    srcs = ["TplpInterfaceImpl.cc"],
    hdrs = ["TplpInterfaceImpl.h"],
    deps = [
        ":numbers",
        ":paper_controller",
        "//picolog",
        "//tplp/bus:i2c",
        "//tplp/config:params",
        "//tplp/config:params_storage",
        "//tplp/fs",
        "//tplp/graphics:lvgl_mutex",
        "//tplp/hx8357",
        "//tplp/motor:stepper",
        "//tplp/ui:TplpInterface",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "clkdiv",
    srcs = ["clkdiv.cc"],
    hdrs = ["clkdiv.h"],
    deps = ["//picolog"],
)

cc_test(
    name = "clkdiv_test",
    srcs = ["clkdiv_test.cc"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":clkdiv",
        "@com_google_googletest//:gtest_main",
    ],
)
