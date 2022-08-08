load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

DEFAULT_LVGL_VERSION = "8.3.1"
DEFAULT_LV_DRIVERS_VERSION = "8.3.0"

def lvgl_dependencies(lvgl_version = DEFAULT_LVGL_VERSION, lv_drivers_version = DEFAULT_LV_DRIVERS_VERSION):
    maybe(
        http_archive,
        name = "lvgl-src",
        build_file = "@lvgl//:BUILD.lvgl-src",
        sha256 = "5600ac2f6a724788b46aea4ba3776e444cf3c2a11272c5b202c78c19d0000bd5",
        strip_prefix = "lvgl-" + lvgl_version,
        url = "https://github.com/lvgl/lvgl/archive/v" + lvgl_version + ".zip",
    )
    maybe(
        http_archive,
        name = "lv_drivers",
        build_file = "@lvgl//:BUILD.lv_drivers",
        sha256 = "6ef58aebbfe34b24b8170074e4d817a03ab9337e4f5f8f1cdf0f0a1c9a63d918",
        strip_prefix = "lv_drivers-" + lv_drivers_version,
        url = "https://github.com/lvgl/lv_drivers/archive/v" + lv_drivers_version + ".zip",
    )
