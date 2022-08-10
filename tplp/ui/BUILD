package(default_visibility = ["//tplp/ui:__subpackages__"])

# Targets in this package are used in both the embedded code and in the simulator.
# They MUST NOT depend on any libraries that are exclusive to one or the other.

cc_library(
    name = "main",
    srcs = ["main.c"],
    hdrs = ["main.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//tplp/ui/screens",
        "@lvgl",
    ],
)
