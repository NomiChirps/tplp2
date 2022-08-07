load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

DEFAULT_LVGL_VERSION = "8.3.1"

def lvgl_dependencies(lvgl_version = DEFAULT_LVGL_VERSION):
    maybe(
        http_archive,
        name = "lvgl-src",
        build_file = "@lvgl//:BUILD.lvgl-src",
        sha256 = "5600ac2f6a724788b46aea4ba3776e444cf3c2a11272c5b202c78c19d0000bd5",
        strip_prefix = "lvgl-" + lvgl_version,
        url = "https://github.com/lvgl/lvgl/archive/v" + lvgl_version + ".zip",
    )
