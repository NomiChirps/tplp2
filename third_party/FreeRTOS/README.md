# FreeRTOS-bazel

Experimental Bazel build for FreeRTOS-Kernel.

All headers exported by this library are placed under the "FreeRTOS/" prefix to avoid polluting the search path.

TODO: make the heap implementation configurable. currently it's always heap_4

## Usage example

You will of course also need to configure Bazel for cross-compilation, if you're targeting a port other than `GCC_POSIX`.

Be sure to check `ports/*/README.md` for usage instructions specific to your port.

### WORKSPACE

```bazel
local_repository(
    name = "FreeRTOS",
    path = "third_party/FreeRTOS",
)

load("@FreeRTOS//:repositories.bzl", "freertos_dependencies")

freertos_dependencies()
```

### .bazelrc

```bazel
build --@FreeRTOS//config:FreeRTOSConfig=//my/project:FreeRTOSConfig
build --@FreeRTOS//config:port=GCC_POSIX
```

### my/project/BUILD

```bazel
cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    # FreeRTOSConfig.h must be includeable with no prefix.
    strip_prefix = "/my/project",
)

cc_binary(
    name = "hello",
    srcs = ["main.c"],
    deps = [
        "@FreeRTOS",
    ],
)
```

### my/project/main.c

```c
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"

int main() {
    // and so on...
```
