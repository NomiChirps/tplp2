load("@rules_pico//pico:defs.bzl", "pico_pio_header")

cc_library(
    name = "stepper",
    srcs = [
        "stepper.cc",
        "stepper.pio.h",
    ],
    hdrs = ["stepper.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//tplp/bus:types",
        "@rules_pico//pico:hardware_clocks",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_pio",
        "@rules_pico//pico:pico_time",
    ],
)

pico_pio_header(
    name = "stepper.pio.h",
    input = "stepper.pio",
)
