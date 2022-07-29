param ($ComPort)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'
function ThrowOnNativeFailure {
    if (-not $?) {
        throw 'Native Failure'
    }
}

# Reset to bootloader using "magic" 1200 baud
# See: https://github.com/raspberrypi/pico-sdk/blob/2e6142b15b8a75c1227dd3edbe839193b2bf9041/src/rp2_common/pico_stdio_usb/include/pico/stdio_usb.h#L54
Write-Host "Attempting to reset on $ComPort"
$port = new-Object System.IO.Ports.SerialPort $ComPort, 1200, None, 8, one
try {
    $port.Open()
}
catch [System.IO.IOException] {
    if ($PSItem.ToString() -like "*A device which does not exist was specified*") {
        # That's OK; the device reset and disconnected before Open() finished
    }
    elseif ($PSItem.ToString() -like "*A device attached to the system is not functioning*") {
        # Sometimes this is the error, too.
    }
    else {
        throw $PSItem
    }
}
$port.Dispose()
Write-Host "Done."