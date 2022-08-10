package(default_visibility = ["//tplp/ui:__subpackages__"])

# TODO: Argh! A dependency cycle!
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
    deps = [
        "@lvgl",
    ],
)
