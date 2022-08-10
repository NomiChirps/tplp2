# lvgl-bazel

Experimental Bazel-ified build for LVGL.

## Usage

### WORKSPACE

```bazel

# local_repository or http_archive or git_repository or whatever rule to import this workspace as @lvgl.

load("@lvgl//:repositories.bzl", "lvgl_dependencies")

lvgl_dependencies()
```

### BUILD

```bazel
cc_library(
    name = "my_lv_conf",
    hdrs = ["lv_conf.h"],  # filename must match
    # it must be #include-able with no prefix, i.e.
    #    #include "lv_conf.h"
    strip_include_prefix = "/this/package",
    visibility = ["//visibility:public"],
)

# Optionally also provide an lv_drivers config.
cc_library(
    name = "my_lv_drv_conf",
    hdrs = ["lv_drv_conf.h"],  # filename must match
    # it must be #include-able with no prefix, i.e.
    #    #include "lv_drv_conf.h"
    strip_include_prefix = "/this/package",
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "example",
    srcs = ["example_main.cc"],
    deps = [
        "@lvgl",
    ],
)
```

### .bazelrc

```bazelrc
build --@lvgl//:lv_conf=//this/package:my_lv_conf

# Optional. lv_drivers will not be compiled if this flag is omitted.
build --@lvgl//:lv_drv_conf=//this/package:my_lv_drv_conf
```
