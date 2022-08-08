load("@bazel_skylib//lib:paths.bzl", "paths")

PORT_NAMES = [
    "GCC_RP2040",
    "GCC_POSIX",
]

def join_lists(xs):
    ret = []
    for x in xs:
        ret.extend(x)
    return ret

def copy_port_srcs(name, portdir, srcs):
    """Copies files from the named port's directory under portable/.../ into the current directory, preserving structure.

    This allows the BUILD files for port implementations to refer to all their files without that cumbersome prefix.

    Args:
        name: Prefix for all targets that this macro will generate.
        portdir: This port's directory relative to FreeRTOS project root, e.g. "portable/ThirdParty/GCC/Posix".
        srcs: List of filenames relative to portdir.
    """
    srcs_by_subdir = {}
    for src in srcs:
        subdir = paths.dirname(src) or ""
        if subdir in srcs_by_subdir:
            srcs_by_subdir[subdir].append(src)
        else:
            srcs_by_subdir[subdir] = [src]

    for subdir, subdir_srcs in srcs_by_subdir.items():
        filegroup_name = name + "_" + subdir + "_files"
        native.filegroup(
            name = filegroup_name,
            srcs = ["@FreeRTOS-Kernel//:{}/{}".format(portdir, src) for src in subdir_srcs],
        )
        native.genrule(
            name = name + "_" + subdir,
            srcs = [":" + filegroup_name],
            outs = subdir_srcs,
            cmd_bash = """
                mkdir -p $(RULEDIR)/{}
                cp $(execpaths {}) $(RULEDIR)/{}
            """.format(subdir, filegroup_name, subdir),
        )
