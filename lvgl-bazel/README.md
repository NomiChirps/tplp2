# lvgl-bazel

Experimental Bazel-ified build for LVGL.

Usage:

```bazel
# WORKSPACE
load("@lvgl//:repositories.bzl", "lvgl_dependencies")

lvgl_dependencies()
```

```bazel
# BUILD
load("@lvgl//:defs.bzl", "lvgl_configured_library")

cc_library(
    name = "my_lv_conf",
    hdrs = ["lv_conf.h"],  # filename must match
    # it must be #include-able with no prefix, i.e.
    #    #include "lv_conf.h"
)

lvgl_configured_library(
    name = "my_lvgl",
    lv_conf = ":my_lv_conf",
)

cc_binary(
    name = "example",
    srcs = ["example_main.cc"],
    deps = [
        ":my_lvgl",
    ],
)
```