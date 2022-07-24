workspace(name = "tplp2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

RULES_PICO_COMMIT = "main"

http_archive(
    name = "rules_pico",
    sha256 = "4df2a0e2a99425fda34503c687bc723d92fa1c906601354ead94fc5ca19e402d",
    strip_prefix = "rules_pico-" + RULES_PICO_COMMIT,
    # TODO: switch back to dfr's repo later
    url = "https://github.com/NomiChirps/rules_pico/archive/" + RULES_PICO_COMMIT + ".zip",
)

load("@rules_pico//pico:repositories.bzl", "rules_pico_dependencies", "rules_pico_toolchains")

rules_pico_dependencies()

rules_pico_toolchains()

FREERTOS_COMMIT = "859dbaf504176796eecc4b4f4eab5a2a9a7fce1c"

http_archive(
    name = "FreeRTOS-Kernel",
    build_file = "//:BUILD.FreeRTOS-Kernel",
    sha256 = "4cfe58b57d976a776c1a32584b4817cd6c674767710cb6b2d30df5c57f4ecc80",
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

FMTLIB_COMMIT = "9.0.0"

http_archive(
    name = "fmtlib",
    build_file = "//:BUILD.fmtlib",
    sha256 = "01867bffc0b30ac71d5f05854e62e451367fa1aceddef40cae965338a7e00a74",
    strip_prefix = "fmt-" + FMTLIB_COMMIT,
    url = "https://github.com/fmtlib/fmt/archive/" + FMTLIB_COMMIT + ".zip",
)

hedron_compile_commands_setup()
