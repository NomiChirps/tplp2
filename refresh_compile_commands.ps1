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

# Filter out command line options unrecognized by clang.
# It errors out otherwise.
$regex = "^.*-mpoke-function-name.*$"
( Get-Content -Path "compile_commands.json" ) -replace $regex, "" | Set-Content "compile_commands.json"