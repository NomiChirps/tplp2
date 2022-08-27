package(default_visibility = ["//tplp/ui:__subpackages__"])

# Targets in this package are used in both the embedded code and in the simulator.
# They MUST NOT depend on any libraries that are exclusive to one or the other.

cc_library(
    name = "main",
    srcs = [
        "main.cc",
        "screen_home.cc",
        "screen_home.h",
        "screen_settings.cc",
        "screen_settings.h",
        "settings_i2c_devices.cc",
        "settings_i2c_devices.h",
        "settings_load_cell.cc",
        "settings_load_cell.h",
        "settings_steppers.cc",
        "settings_steppers.h",
        "settings_test_content.cc",
        "settings_test_content.h",
    ],
    hdrs = [
        "main.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":TplpInterface",
        ":globals",
        "//tplp/config:params",
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
    deps = [":TplpInterface"],
)
