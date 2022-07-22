Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'
function ThrowOnNativeFailure {
    if (-not $?) {
        throw 'Native Failure'
    }
}

bazel build :refresh_compile_commands //tplp/... --keep_going

bazel build --nobuild :refresh_compile_commands
ThrowOnNativeFailure

$env:BUILD_WORKSPACE_DIRECTORY = $PSScriptRoot
python bazel-bin/refresh_compile_commands.py
ThrowOnNativeFailure
