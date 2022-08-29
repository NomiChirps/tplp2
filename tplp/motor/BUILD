load("@rules_pico//pico:defs.bzl", "pico_pio_header")

cc_library(
    name = "stepper",
    srcs = ["stepper.cc"],
    hdrs = ["stepper.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//tplp/bus:types",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_pio",
    ],
)

pico_pio_header(
    name = "stepper.pio.h",
    input = "stepper.pio",
)
