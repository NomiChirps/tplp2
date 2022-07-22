load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

exports_files(["FreeRTOSConfig.h"])

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
)

# For VSCode
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        "//tplp:firmware": "",
        "//lib/SharpLCD": "",
    },
)
