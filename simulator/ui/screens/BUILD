# Argh! A dependency cycle in the headers!
cc_library(
    name = "screens",
    srcs = [
        "home_screen.c",
        "settings.c",
    ],
    hdrs = [
        "home_screen.h",
        "settings.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@lvgl",
    ],
)
