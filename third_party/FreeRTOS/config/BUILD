load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

label_flag(
    name = "FreeRTOSConfig",
    build_setting_default = ":missing_freertos_config_header",
    visibility = ["//visibility:public"],
)

string_flag(
    name = "port",
    values = [
        "GCC_RP2040",
        "GCC_POSIX",
    ],
    build_setting_default = "GCC_POSIX",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "missing_freertos_config_header",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/config",
)
