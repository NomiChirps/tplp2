bazel build :hello.elf :refresh_compile_commands --keep_going
$env:BUILD_WORKSPACE_DIRECTORY = $PSScriptRoot
python bazel-bin/refresh_compile_commands.py
