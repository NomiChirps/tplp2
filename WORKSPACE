workspace(name = "tplp2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_pico",
    sha256 = "1092d6ba20fe95b4ddda33ee924c371c02353e12895faa09682cf01a2064d745",
    strip_prefix = "rules_pico-7f1789f71e3d757d163883038f8f9019969372c3",
    url = "https://github.com/dfr/rules_pico/archive/7f1789f71e3d757d163883038f8f9019969372c3.zip",
)

load("@rules_pico//pico:repositories.bzl", "rules_pico_dependencies", "rules_pico_toolchains")

rules_pico_dependencies()

rules_pico_toolchains()

http_archive(
    name = "FreeRTOS-Kernel",
    build_file = "//:BUILD.FreeRTOS-Kernel",
    sha256 = "577b650002e3f48d374c12102474784a7173fbfabf71aa1ce0758893297a49b9",
    strip_prefix = "FreeRTOS-Kernel-9eca3152bc84c4a2be6ee3a3be34d8a5ae50943f",
    # TODO: switch back to main FreeRTOS repo after https://github.com/FreeRTOS/FreeRTOS-Kernel/pull/525 is merged
    url = "https://github.com/NomiChirps/FreeRTOS-Kernel/archive/9eca3152bc84c4a2be6ee3a3be34d8a5ae50943f.zip",
)
