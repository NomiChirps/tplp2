package(default_visibility = ["//tplp/ui:__subpackages__"])

# Targets in this package are used in both the embedded code and in the simulator.
# They MUST NOT depend on any libraries that are exclusive to one or the other.

cc_library(
    name = "main",
    srcs = [
        "main.cc",
    ],
    hdrs = [
        "main.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":TplpInterface",
        ":all_screens",
        ":globals",
        "//picolog",
        "@lvgl",
    ],
)

cc_library(
    name = "all_screens",
    srcs = [
        "screen_home.cc",
        "screen_i2c_devices.cc",
        "screen_load_cell.cc",
        "screen_paper.cc",
        "screen_params.cc",
        "screen_steppers.cc",
    ],
    hdrs = [
        "screen_home.h",
        "screen_i2c_devices.h",
        "screen_load_cell.h",
        "screen_paper.h",
        "screen_params.h",
        "screen_steppers.h",
    ],
    deps = [
        ":TplpInterface",
        ":globals",
        ":popups",
        "//absl/strings",
        "//tplp/config:constants",
        "//tplp/config:params",
        "//tplp/config:public_params",
        "@lvgl",
    ],
)

cc_library(
    name = "TplpInterface",
    srcs = ["TplpInterface.cc"],
    hdrs = ["TplpInterface.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog:status",
    ],
)

cc_library(
    name = "globals",
    srcs = ["globals.cc"],
    hdrs = ["globals.h"],
    deps = [
        ":TplpInterface",
        "@lvgl",
    ],
)

cc_library(
    name = "popups",
    srcs = ["popups.cc"],
    hdrs = ["popups.h"],
    deps = [
        "//absl/strings",
        "//picolog:status",
        "@lvgl",
    ],
)
