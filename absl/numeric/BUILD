package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bits",
    hdrs = [
        "bits.h",
        "internal/bits.h",
    ],
    deps = [
        "//absl/base:config",
        "//absl/base:core_headers",
    ],
)

cc_library(
    name = "int128",
    srcs = [
        "int128.cc",
        "int128_have_intrinsic.inc",
        "int128_no_intrinsic.inc",
    ],
    hdrs = ["int128.h"],
    deps = [
        ":bits",
        "//absl/base:config",
        "//absl/base:core_headers",
    ],
)
