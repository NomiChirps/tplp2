load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

# Latest on the main branch.
FREERTOS_COMMIT = "c22f40d9a5e239fdfd98bfc210a33c26a627b9f6"

def freertos_dependencies():
    maybe(
        http_archive,
        name = "FreeRTOS-Kernel",
        build_file = "@FreeRTOS//:BUILD.FreeRTOS-Kernel",
        sha256 = "4b6b28745cb1f1755596b12bb04549a1dc0eacdbcb16facd8754a6f7cfb72546",
        strip_prefix = "FreeRTOS-Kernel-" + FREERTOS_COMMIT,
        url = "https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/" + FREERTOS_COMMIT + ".zip",
    )
