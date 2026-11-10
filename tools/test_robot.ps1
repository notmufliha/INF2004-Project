param([string]$LogPath = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$env:PATH = 'C:\msys64\mingw64\bin;C:\msys64\usr\bin;' + $env:PATH
if (-not $LogPath) {
    $LogPath = Join-Path $projectRoot ('testing/integration/logs/' + (Get-Date -Format 'yyyy-MM-dd_HHmmss') + '_host.txt')
}
$LogPath = [IO.Path]::GetFullPath($LogPath)
New-Item -ItemType Directory -Force -Path (Split-Path $LogPath -Parent) | Out-Null
Push-Location $projectRoot
Start-Transcript -Path $LogPath | Out-Null
try {
    Write-Host "Source base: $(git rev-parse HEAD); includes current working tree changes"
    & 'C:\msys64\usr\bin\bash.exe' testing/subsystem1/host/run_host_tests.sh
    if ($LASTEXITCODE -ne 0) { throw 'Subsystem 1 tests failed' }
    & node testing/subsystem1/host/test_dashboard_ui.js
    if ($LASTEXITCODE -ne 0) { throw 'Dashboard UI tests failed' }
    & powershell -ExecutionPolicy Bypass -File testing/subsystem2/host/run_tests.ps1
    if ($LASTEXITCODE -ne 0) { throw 'Subsystem 2 tests failed' }
    & powershell -ExecutionPolicy Bypass -File testing/controller/run_tests.ps1
    if ($LASTEXITCODE -ne 0) { throw 'Controller/subsystem 3 tests failed' }
    & python testing/subsystem5/run_complete_tests.py
    if ($LASTEXITCODE -ne 0) { throw 'Subsystem 5 tests failed' }
    Write-Host 'PASS all requested subsystem host suites (hardware tests still required)'
} finally {
    Stop-Transcript | Out-Null
    Pop-Location
    Write-Host "Test log: $LogPath"
}
