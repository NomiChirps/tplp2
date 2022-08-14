package(default_visibility = ["//tplp/ui:__subpackages__"])

# Targets in this package are used in both the embedded code and in the simulator.
# They MUST NOT depend on any libraries that are exclusive to one or the other.

cc_library(
    name = "main",
    srcs = [
        "main.c",
        "screen_home.c",
        "screen_settings.c",
        "settings_i2c_devices.c",
        "settings_test_content.c",
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
