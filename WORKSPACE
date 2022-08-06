workspace(name = "tplp2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

RULES_PICO_COMMIT = "main"

http_archive(
    name = "rules_pico",
    sha256 = "5d33e7d989252807a87dc57608da1b649c576a22bdbe1a8ec2fc781433fa10ac",
    strip_prefix = "rules_pico-" + RULES_PICO_COMMIT,
    url = "https://github.com/dfr/rules_pico/archive/" + RULES_PICO_COMMIT + ".zip",
)

load("@rules_pico//pico:repositories.bzl", "rules_pico_dependencies", "rules_pico_toolchains")

rules_pico_dependencies()

rules_pico_toolchains()

# Latest on the main branch.
FREERTOS_COMMIT = "c22f40d9a5e239fdfd98bfc210a33c26a627b9f6"

http_archive(
    name = "FreeRTOS",
    build_file = "//:BUILD.FreeRTOS",
    sha256 = "4b6b28745cb1f1755596b12bb04549a1dc0eacdbcb16facd8754a6f7cfb72546",
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

NAMEDTYPE_COMMIT = "77a95c8002a28f5cb48d2d0cd985904d66912af3"

http_archive(
    name = "NamedType",
    build_file = "//:BUILD.NamedType",
    sha256 = "708f0dcbc2d6321a9b696212e1f33dc0286ff942f93b09039884739f37050268",
    strip_prefix = "NamedType-" + NAMEDTYPE_COMMIT + "/include",
    url = "https://github.com/joboccara/NamedType/archive/" + NAMEDTYPE_COMMIT + ".zip",
)

http_file(
    name = "pico-debug",
    sha256 = "4c232340c7e0a276e0ea94d54c5de7d05e18500a1b5b3984b6dcb67617e9df55",
    urls = ["https://github.com/majbthrd/pico-debug/releases/download/v10.05/pico-debug-gimmecache.uf2"],
)
