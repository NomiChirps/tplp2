load("@rules_pico//pico:defs.bzl", "pico_pio_header")

cc_library(
    name = "stepper",
    srcs = ["stepper.cc"],
    hdrs = ["stepper.h"],
    deps = [
        "//picolog",
        "//tplp/bus:types",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_pwm",
    ],
)

pico_pio_header(
    name = "stepper.pio.h",
    input = "stepper.pio",
)
