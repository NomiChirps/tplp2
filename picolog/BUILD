cc_library(
    name = "picolog",
    srcs = ["picolog.cc"],
    hdrs = ["picolog.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":status",
        ":vlog_is_on",
        "//picolog/hal",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "vlog_is_on",
    hdrs = ["vlog_is_on.h"],
)

cc_test(
    name = "vlog_is_on_test",
    srcs = ["vlog_is_on_test.cc"],
    deps = [":vlog_is_on"],
)

cc_library(
    name = "status",
    srcs = [
        "status.cc",
        "status_internal.h",
    ],
    hdrs = ["status.h"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "status_macros",
    hdrs = ["status_macros.h"],
    visibility = ["//visibility:public"],
    deps = [":status"],
)
