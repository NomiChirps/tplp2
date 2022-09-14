package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        # "internal/cycleclock.cc",
        # "internal/spinlock.cc",
        # "internal/sysinfo.cc",
        # "internal/thread_identity.cc",
        # "internal/unscaledcycleclock.cc",
    ],
    hdrs = [
        # "call_once.h",
        "casts.h",
        # "internal/cycleclock.h",
        # "internal/low_level_scheduling.h",
        # "internal/per_thread_tls.h",
        # "internal/spinlock.h",
        # "internal/sysinfo.h",
        # "internal/thread_identity.h",
        # "internal/tsan_mutex_interface.h",
        # "internal/unscaledcycleclock.h",
    ],
    deps = [
        ":base_internal",
        # ":config",
        # ":core_headers",
        # ":dynamic_annotations",
        # ":log_severity",
        ":raw_logging_internal",
        # ":spinlock_wait",
        "//absl/meta:type_traits",
    ],
)

cc_library(
    name = "base_internal",
    hdrs = [
        # "internal/hide_ptr.h",
        "internal/identity.h",
        # "internal/inline_variable.h",
        # "internal/invoke.h",
        # "internal/scheduling_mode.h",
    ],
    visibility = [
        "//absl:__subpackages__",
    ],
    deps = [
        ":config",
        "//absl/meta:type_traits",
    ],
)

cc_library(
    name = "core_headers",
    srcs = [
        # "internal/thread_annotations.h",
    ],
    hdrs = [
        "attributes.h",
        "const_init.h",
        "macros.h",
        "optimization.h",
        "port.h",
        # "thread_annotations.h",
    ],
    deps = [
        ":config",
    ],
)

cc_library(
    name = "config",
    hdrs = [
        "config.h",
        "options.h",
        "policy_checks.h",
    ],
)

cc_library(
    name = "endian",
    hdrs = [
        "internal/endian.h",
        "internal/unaligned_access.h",
    ],
    deps = [
        ":base",
        ":config",
        ":core_headers",
    ],
)

cc_library(
    name = "raw_logging_internal",
    srcs = ["internal/raw_logging.cc"],
    hdrs = ["internal/raw_logging.h"],
    visibility = [
        "//absl:__subpackages__",
    ],
    deps = [
        ":config",
        ":core_headers",
        # ":errno_saver",
        ":log_severity",
    ],
)

cc_library(
    name = "log_severity",
    srcs = ["log_severity.cc"],
    hdrs = ["log_severity.h"],
    deps = [
        ":config",
        ":core_headers",
    ],
)
