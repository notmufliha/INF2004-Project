param([string]$SdkPath = $env:PICO_SDK_PATH,
      [ValidateSet(0, 1)][int]$Smp = 0,
      [switch]$Network,
      [switch]$ExampleNetwork)
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
$env:ROBOT_BUILD_SMP = "$Smp"
$env:ROBOT_BUILD_WIFI = 'none'
$env:ROBOT_BUILD_NET = 'none'
$env:ROBOT_BUILD_MQTT = '0'
$env:ROBOT_BUILD_EXAMPLE = '0'
$suffix = ''
if ($Network -or $ExampleNetwork) {
    $env:ROBOT_BUILD_WIFI = 'cyw43'
    $env:ROBOT_BUILD_NET = 'lwip'
    $env:ROBOT_BUILD_MQTT = '1'
    $suffix = '_wifi_lwip_mqtt'
    if ($ExampleNetwork) {
        $exampleDir = Join-Path $projectRoot '.codex_tmp/network-example'
        New-Item -ItemType Directory -Path $exampleDir -Force | Out-Null
        Copy-Item -LiteralPath (Join-Path $projectRoot 'config/wifi_secrets.example.h') -Destination (Join-Path $exampleDir 'wifi_secrets.h')
        $env:ROBOT_BUILD_EXAMPLE = '1'
        $suffix += '_example'
        Write-Host 'Network compilation check only: example credentials are used.'
    } elseif (-not (Test-Path (Join-Path $projectRoot 'config/wifi_secrets.h'))) {
        throw 'Configure config/wifi_secrets.h using the example file before building with -Network.'
    }
}
Push-Location $projectRoot
try {
    & $bashPath -c 'make -C build_make CONSOLE=usb_cdc WIFI="$ROBOT_BUILD_WIFI" NET="$ROBOT_BUILD_NET" MQTT="$ROBOT_BUILD_MQTT" SMP="$ROBOT_BUILD_SMP" NETWORK_EXAMPLE="$ROBOT_BUILD_EXAMPLE" TEST=0 -j8'
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed ($LASTEXITCODE). Do not flash an old UF2." }
    $image = Join-Path $projectRoot "build_make/mtk3pico_smp${Smp}_usb_cdc$suffix.uf2"
    Write-Host "Built firmware: $image"
    Get-FileHash $image -Algorithm SHA256
} finally { Pop-Location }
