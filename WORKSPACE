workspace(name = "tplp2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

RULES_PICO_COMMIT = "abf9efd5a80cbbaa7fe82fe986864618dfe90741"
http_archive(
    name = "rules_pico",
    sha256 = "82704ac51c05721346aeba93fca98ada8fb7384d09bd5eee69647a3d7cdb631e",
    strip_prefix = "rules_pico-" + RULES_PICO_COMMIT,
    # FIXME: switch back when https://github.com/dfr/rules_pico/pull/1 is merged
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

PICO_ARDUINO_COMPAT_COMMIT = "0c1173cb3ea407b2cc28244b46447214723f96ee"
http_archive(
	name = "pico-arduino-compat",
	build_file = "//:BUILD.pico-arduino-compat",
	sha256 = "0b8c941fb2e66c12ff2263d1f457f5e5975180d81186da4a991c70258f42788e",
	strip_prefix = "pico-arduino-compat-" + PICO_ARDUINO_COMPAT_COMMIT,
	url = "https://github.com/fhdm-dev/pico-arduino-compat/archive/" + PICO_ARDUINO_COMPAT_COMMIT + ".zip",
	patches = ["//:pico-arduino-compat.patch"],
	remote_patch_strip = 1,
)

ARDUINO_PICO_COMMIT = "cf63040c5aa8306cfdb34c66b5a9cb4e75a4fad8"
http_archive(
	name = "arduino-pico",
	build_file = "//:BUILD.arduino-pico",
	sha256 = "9b82b0a869d5dcd2c84f043173c3e6260f6a151f1a048a0e9e85f3bb79b4b9d2",
	strip_prefix = "arduino-pico-" + ARDUINO_PICO_COMMIT,
	url = "https://github.com/earlephilhower/arduino-pico/archive/" + ARDUINO_PICO_COMMIT + ".zip",
)

ADAFRUIT_BUSIO_COMMIT = "da6809b582f1b64eeafe44d58df8a90a2fa9b47c"
http_archive(
	name = "adafruit-busio",
	build_file = "//:BUILD.adafruit-busio",
	sha256 = "92e81de991c1891bfa2427c8ec0de26dd369dec17f9408e9f7930e433804104c",
	strip_prefix = "Adafruit_BusIO-" + ADAFRUIT_BUSIO_COMMIT,
	url = "https://github.com/adafruit/Adafruit_BusIO/archive/" + ADAFRUIT_BUSIO_COMMIT + ".zip",
)

ADAFRUIT_GFX_LIBRARY_COMMIT = "223f914d0e092cc24723182a2e3273e61c4b22ea"
http_archive(
	name = "adafruit-gfx-library",
	build_file = "//:BUILD.adafruit-gfx-library",
	sha256 = "9a16a4291c3e228d4d116bce156d34f97bea3dd4061f71ed8fa33b7607a9ce6f",
	strip_prefix = "Adafruit-GFX-Library-" + ADAFRUIT_GFX_LIBRARY_COMMIT,
	url = "https://github.com/adafruit/Adafruit-GFX-Library/archive/" + ADAFRUIT_GFX_LIBRARY_COMMIT + ".zip",
)

ADAFRUIT_SHARP_MEMORY_DISPLAY_COMMIT = "d0339a5b76762bdb90d356022fc5693c93790513"
http_archive(
	name = "adafruit-sharp-memory-display",
	build_file = "//:BUILD.adafruit-sharp-memory-display",
	sha256 = "0f0efbc20cff9870d91f07ad8e18d258690c593270857bccb254195da881d2e2",
	strip_prefix = "Adafruit_SHARP_Memory_Display-" + ADAFRUIT_SHARP_MEMORY_DISPLAY_COMMIT,
	url = "https://github.com/adafruit/Adafruit_SHARP_Memory_Display/archive/" + ADAFRUIT_SHARP_MEMORY_DISPLAY_COMMIT + ".zip",
)
