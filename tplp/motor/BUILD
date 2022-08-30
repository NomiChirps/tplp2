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
        ":cosine_table",
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

cc_library(
    name = "cosine_table",
    srcs = ["cosine_table.cc"],
    hdrs = ["cosine_table.h"],
)

genrule(
    name = "cosine_table_genrule",
    outs = ["cosine_table.cc"],
    cmd = "$(execpath generate_cosine_table) 1024 > $@",
    tools = [":generate_cosine_table"],
)

py_binary(
    name = "generate_cosine_table",
    srcs = ["generate_cosine_table.py"],
)
