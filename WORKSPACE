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

FREERTOS_COMMIT = "2345b212acb8326387bef1c010ccce094eeb8149"

http_archive(
    name = "FreeRTOS-Kernel",
    build_file = "//:BUILD.FreeRTOS-Kernel",
    sha256 = "22e5fb3252b851ae72129625578421e30962673eb21ab177c2e39aee0a047312",
    strip_prefix = "FreeRTOS-Kernel-" + FREERTOS_COMMIT,
    # TODO: switch back to main FreeRTOS repo after https://github.com/FreeRTOS/FreeRTOS-Kernel/pull/525 is merged
    url = "https://github.com/NomiChirps/FreeRTOS-Kernel/archive/" + FREERTOS_COMMIT + ".zip",
)
#new_local_repository(
#    name = "FreeRTOS-Kernel",
#    build_file = "//:BUILD.FreeRTOS-Kernel",
#    path = "FreeRTOS-Kernel",
#)
