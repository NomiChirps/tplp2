package(default_visibility = ["//tplp/ui:__subpackages__"])

# Targets in this package are used in both the embedded code and in the simulator.
# They MUST NOT depend on any libraries that are exclusive to one or the other.

cc_library(
    name = "main",
    srcs = [
        "main.cc",
        "screen_home.cc",
        "screen_settings.cc",
        "settings_i2c_devices.cc",
        "settings_test_content.cc",
    ],
    hdrs = [
        "main.h",
        "screen_home.h",
        "screen_settings.h",
        "settings_i2c_devices.h",
        "settings_test_content.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@lvgl",
    ],
)
