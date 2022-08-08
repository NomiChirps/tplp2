def _config_override_impl(settings, attr):
    return {
        "@lvgl-src//:lv_conf": attr.lv_conf,
        "@lvgl-src//:lv_drv_conf": attr.lv_drv_conf,
    }

config_override = transition(
    implementation = _config_override_impl,
    inputs = [],
    outputs = [
        "@lvgl-src//:lv_conf",
        "@lvgl-src//:lv_drv_conf",
    ],
)

def _lvgl_configured_library_impl(ctx):
    return [ctx.attr._lvgl[CcInfo], ctx.attr._lvgl[DefaultInfo]]

lvgl_configured_library = rule(
    implementation = _lvgl_configured_library_impl,
    doc = "Build LVGL with the specified configuration options.",
    cfg = config_override,
    provides = [CcInfo, DefaultInfo],
    attrs = {
        # This attribute is required to use starlark transitions. It allows
        # allowlisting usage of this rule. For more information, see
        # https://docs.bazel.build/versions/master/skylark/config.html#user-defined-transitions
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
        "_lvgl": attr.label(
            default = "@lvgl-src//:lvgl",
        ),
        "lv_conf": attr.label(
            doc = "cc_library target that exposes your lv_conf.h header",
            mandatory = True,
        ),
        "lv_drv_conf": attr.label(
            doc = "optional cc_library target that exposes your lv_drv_conf.h header. lv_drivers will not be compiled if this is not provided.",
            mandatory = False,
            default = "@lvgl-src//:lv_drv_conf_none",
        ),
    },
)
