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
        ":crc16",
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

cc_library(
    name = "crc16",
    srcs = ["crc16.cc"],
    hdrs = ["crc16.h"],
)

cc_test(
    name = "crc_test",
    srcs = ["crc_test.cc"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":crc16",
        ":crc7",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "fatfs_diskio_impl",
    srcs = ["fatfs_diskio_impl.cc"],
    hdrs = ["fatfs_diskio_impl.h"],
    deps = [
        ":sdspi",
        "//picolog",
        "@fatfs",
        "@rules_pico//pico:hardware_rtc",
    ],
)
