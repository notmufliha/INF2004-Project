param([string]$SdkPath = $env:PICO_SDK_PATH)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $SdkPath) { $SdkPath = Join-Path $projectRoot '.codex_tmp/pico-sdk' }
if (-not (Test-Path (Join-Path $SdkPath 'pico_sdk_init.cmake'))) {
    throw 'Set PICO_SDK_PATH or pass -SdkPath pointing to the Pico SDK.'
}
$bashPath = 'C:\msys64\usr\bin\bash.exe'
if (-not (Test-Path $bashPath)) { throw 'Install MSYS2 at C:\msys64 (make and a host C++ compiler are required).' }
$env:PATH = 'C:\msys64\mingw64\bin;C:\msys64\usr\bin;' + $env:PATH
$env:PICO_SDK_PATH = $SdkPath.Replace('\', '/')
Push-Location $projectRoot
try {
    & $bashPath -c 'make -C build_make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 -j8'
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed ($LASTEXITCODE). Do not flash an old UF2." }
    Write-Host "Flash this file: $projectRoot\build_make\mtk3pico_smp0_usb_cdc.uf2"
    Get-FileHash (Join-Path $projectRoot 'build_make/mtk3pico_smp0_usb_cdc.uf2') -Algorithm SHA256
} finally { Pop-Location }
