Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'
function ThrowOnNativeFailure {
    if (-not $?) {
        throw 'Native Failure'
    }
}

bazel build //tplp:firmware.uf2
ThrowOnNativeFailure
Copy-Item bazel-bin/tplp/firmware.uf2 D:/
ThrowOnNativeFailure