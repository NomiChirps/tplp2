Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'
function ThrowOnNativeFailure {
    if (-not $?) {
        throw 'Native Failure'
    }
}

bazel build :refresh_compile_commands //tplp/... --keep_going

bazel run :refresh_compile_commands
ThrowOnNativeFailure
