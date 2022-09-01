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
        ":sin_table",
        "//picolog",
        "//tplp/bus:dma",
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

cc_library(
    name = "sin_table",
    srcs = ["sin_table.cc"],
    hdrs = ["sin_table.h"],
)

genrule(
    name = "sin_table_genrule",
    outs = ["sin_table.cc"],
    cmd = "$(execpath generate_sin_table) 256 > $@",
    tools = [":generate_sin_table"],
)

py_binary(
    name = "generate_sin_table",
    srcs = ["generate_sin_table.py"],
)
