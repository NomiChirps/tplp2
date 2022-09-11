cc_library(
    name = "ffconf",
    hdrs = ["ffconf.h"],
    strip_include_prefix = "/tplp/fs",
    visibility = ["@fatfs//:__pkg__"],
)

cc_library(
    name = "sdspi",
    srcs = ["sdspi.cc"],
    hdrs = ["sdspi.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":crc7",
        "//picolog",
        "//picolog:status",
        "//picolog:status_macros",
        "//picolog:statusor",
        "//tplp:rtos_util",
        "//tplp/bus:spi",
    ],
)

cc_library(
    name = "crc7",
    srcs = ["crc7.cc"],
    hdrs = ["crc7.h"],
)

cc_test(
    name = "crc7_test",
    srcs = ["crc7_test.cc"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":crc7",
        "@com_google_googletest//:gtest_main",
    ],
)
