workspace(name = "tplp2")

local_repository(
    name = "lvgl",
    path = "third_party/lvgl",
)

load("@lvgl//:repositories.bzl", "lvgl_dependencies")

lvgl_dependencies()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

RULES_PICO_COMMIT = "build-with-more-configs"

http_archive(
    name = "rules_pico",
    sha256 = "7676a5066449461ab61762cc4a660d4b3f6f1bff9219977cae08374bd20c1f88",
    strip_prefix = "rules_pico-" + RULES_PICO_COMMIT,
    url = "https://github.com/NomiChirps/rules_pico/archive/" + RULES_PICO_COMMIT + ".zip",
)
# local_repository(
#     name = "rules_pico",
#     path = "../rules_pico",
# )

load("@rules_pico//pico:repositories.bzl", "rules_pico_dependencies", "rules_pico_toolchains")

rules_pico_dependencies()

rules_pico_toolchains()

local_repository(
    name = "FreeRTOS",
    path = "third_party/FreeRTOS",
)

load("@FreeRTOS//:repositories.bzl", "freertos_dependencies")

freertos_dependencies()

ETL_COMMIT = "20.31.2"

http_archive(
    name = "etl",
    build_file = "//third_party:BUILD.etl.include",
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

NAMEDTYPE_COMMIT = "77a95c8002a28f5cb48d2d0cd985904d66912af3"

http_archive(
    name = "NamedType",
    build_file = "//third_party:BUILD.NamedType",
    sha256 = "708f0dcbc2d6321a9b696212e1f33dc0286ff942f93b09039884739f37050268",
    strip_prefix = "NamedType-" + NAMEDTYPE_COMMIT + "/include",
    url = "https://github.com/joboccara/NamedType/archive/" + NAMEDTYPE_COMMIT + ".zip",
)

http_file(
    name = "pico-debug",
    sha256 = "4c232340c7e0a276e0ea94d54c5de7d05e18500a1b5b3984b6dcb67617e9df55",
    urls = ["https://github.com/majbthrd/pico-debug/releases/download/v10.05/pico-debug-gimmecache.uf2"],
)

# Workaround for Bazel not ignoring subdirectories with a WORKSPACE file in them.
# openocd isn't part of our build, we just have it here so vscode can use it.
# See https://github.com/bazelbuild/bazel/issues/2460
local_repository(
    name = "do_not_use_ignore_openocd",
    path = "third_party/openocd",
)

SDL2_VERSION = "2.0.22"

http_archive(
    name = "SDL2",
    build_file = "//third_party:BUILD.SDL2",
    sha256 = "9a81ab724e6dcef96c61a4a2ebe7758e5b8bb191794650d276a20d5148fbd50c",
    strip_prefix = "SDL2-" + SDL2_VERSION,
    url = "https://github.com/libsdl-org/SDL/releases/download/release-" + SDL2_VERSION + "/SDL2-" + SDL2_VERSION + ".zip",
)

http_archive(
    name = "rules_foreign_cc",
    sha256 = "2a4d07cd64b0719b39a7c12218a3e507672b82a97b98c6a89d38565894cf7c51",
    strip_prefix = "rules_foreign_cc-0.9.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.9.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()
