load("@rules_pico//pico:defs.bzl", "pico_pio_header")

cc_library(
    name = "hx711",
    srcs = [
        "hx711.cc",
        "hx711.pio.h",
    ],
    hdrs = ["hx711.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//tplp/bus:types",
        "@rules_pico//pico:hardware_clocks",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_pio",
    ],
)

pico_pio_header(
    name = "hx711.pio.h",
    input = "hx711.pio",
)
