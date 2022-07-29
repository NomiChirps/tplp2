workspace(name = "tplp2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

RULES_PICO_COMMIT = "c1f498afbfe6403b447ede11420dc08233bc2147"

http_archive(
    name = "rules_pico",
    sha256 = "0bbbf8198219c1795dfa3e276abb198043036abbb6b8b7b32e2aba3052b42511",
    strip_prefix = "rules_pico-" + RULES_PICO_COMMIT,
    # TODO: switch back to dfr's repo later
    url = "https://github.com/NomiChirps/rules_pico/archive/" + RULES_PICO_COMMIT + ".zip",
)

load("@rules_pico//pico:repositories.bzl", "rules_pico_dependencies", "rules_pico_toolchains")

rules_pico_dependencies()

rules_pico_toolchains()

# Latest on the SMP branch.
FREERTOS_COMMIT = "13f034eb7480f11c9fc43e7b51a1e55656985bf4"

http_archive(
    name = "FreeRTOS-Kernel",
    build_file = "//:BUILD.FreeRTOS-Kernel",
    sha256 = "f25e2924fc503e4a0733cdc3a12ca6187d9f864b6e3dfd3d64282b5dc1ab7869",
    strip_prefix = "FreeRTOS-Kernel-" + FREERTOS_COMMIT,
    url = "https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/" + FREERTOS_COMMIT + ".zip",
)

ETL_COMMIT = "20.31.2"

http_archive(
    name = "etl",
    build_file = "//:BUILD.etl.include",
    sha256 = "8e290aabf5f3bf69a94ddce174d96b4fb21a3bde372fb64f80cf29e6f4e32c85",
    strip_prefix = "etl-" + ETL_COMMIT + "/include",
    url = "https://github.com/ETLCPP/etl/archive/" + ETL_COMMIT + ".zip",
)

# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
HEDRON_COMPILE_COMMANDS_COMMIT = "05610f52a2ea3cda6ac27133b96f71c36358adf9"

http_archive(
    name = "hedron_compile_commands",
    sha256 = "78776448d9684dc6d6f50f37c94014f1d1b054c392050214f61a3ddc5ece64e2",
    strip_prefix = "bazel-compile-commands-extractor-" + HEDRON_COMPILE_COMMANDS_COMMIT,
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/" + HEDRON_COMPILE_COMMANDS_COMMIT + ".tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

LVGL_VERSION = "8.3.1"

http_archive(
    name = "lvgl",
    build_file = "//:BUILD.lvgl",
    sha256 = "5600ac2f6a724788b46aea4ba3776e444cf3c2a11272c5b202c78c19d0000bd5",
    strip_prefix = "lvgl-" + LVGL_VERSION,
    url = "https://github.com/lvgl/lvgl/archive/v" + LVGL_VERSION + ".zip",
)
