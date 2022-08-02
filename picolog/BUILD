cc_library(
    name = "picolog",
    srcs = ["picolog.cc"],
    hdrs = ["picolog.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":backtrace",
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "backtrace",
    srcs = ["backtrace.cc"],
    hdrs = ["backtrace.h"],
)
