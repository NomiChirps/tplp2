load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# For VSCode
refresh_compile_commands(
    name = "refresh_compile_commands_pico",
    targets = {
        # In the absence of an explicit `-x` flag specifying the language,
        # clang/clangd uses the file extension to guess. In the case of C++
        # header files named "foo.h", it guesses C, and doesn't include the
        # C++ system headers.
        # Actually, it could be a totally different issue. See:
        # https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
        "//tplp/...": "--cxxopt=-xc++",
    },
)

refresh_compile_commands(
    name = "refresh_compile_commands_tests",
    targets = {
        "//tplp/bus:dma_program_test": "--cxxopt=-xc++",
    },
)