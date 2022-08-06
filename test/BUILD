load("@rules_pico//pico:defs.bzl", "pico_add_uf2_output", "pico_binary")

pico_binary(
    name = "blinky.elf",
    srcs = ["blinky.cc"],
    deps = [
        ":hook_impl",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:pico_stdlib",
    ],
)

pico_add_uf2_output(
    name = "blinky.uf2",
    input = "blinky.elf",
)

# Stubs so we don't have to change to a different :FreeRTOSConfig (in .bazelrc) just for this.
cc_library(
    name = "hook_impl",
    srcs = ["hook_impl.cc"],
    deps = [
        "@FreeRTOS",
        "@rules_pico//pico:pico_time",
    ],
    alwayslink = 1,
)
