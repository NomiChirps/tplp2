sh_binary(
    name = "flash-pico-debug",
    srcs = ["flash_usbmsc.sh"],
    args = ["$(execpath @pico-debug//file)"],
    data = ["@pico-debug//file"],
    target_compatible_with = [
        "@platforms//os:linux",
    ],
)
