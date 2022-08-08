load("//:defs.bzl", "copy_port_srcs")

package(default_visibility = ["@FreeRTOS-Kernel//:__subpackages__"])

cc_library(
    name = "portmacro",
    hdrs = [
        "portmacro.h",
    ],
    include_prefix = "FreeRTOS",
)

cc_library(
    name = "port_impl",
    srcs = [
        "port.c",
        "portmacro.h",
    ],
    implementation_deps = [
        "@FreeRTOS-Kernel//:FreeRTOS-Kernel-headers-bareinclude",
        ":wait_for_event",
    ],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        "@FreeRTOS-Kernel",
    ],
    alwayslink = 1,
)

cc_library(
    name = "wait_for_event",
    srcs = ["utils/wait_for_event.c"],
    hdrs = ["utils/wait_for_event.h"],
)

copy_port_srcs(
    name = "copy_srcs",
    srcs = [
        "port.c",
        "portmacro.h",
        "utils/wait_for_event.c",
        "utils/wait_for_event.h",
    ],
    portdir = "portable/ThirdParty/GCC/Posix",
)
