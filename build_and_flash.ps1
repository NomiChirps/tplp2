param ($UploadPath)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'
function ThrowOnNativeFailure {
    if (-not $?) {
        throw 'Native Failure'
    }
}

if (!(Test-Path $UploadPath)) {
    throw "UploadPath $UploadPath does not exist"
}

bazel build -c opt //tplp:firmware.uf2
ThrowOnNativeFailure
Copy-Item bazel-bin/tplp/firmware.uf2 $UploadPath
ThrowOnNativeFailure