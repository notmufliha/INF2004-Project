$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$sourceDirectory = Join-Path $projectRoot 'app_program/subsystem4'
$testExecutable = Join-Path $PSScriptRoot 'testboot_main.exe'
$testSources = @(
    (Join-Path $sourceDirectory 'subsystem4.c'),
    (Join-Path $PSScriptRoot 'testboot_main.c')
)

foreach ($robotMode in @(0, 1)) {
    foreach ($testMode in @(0, 1)) {
        foreach ($mockMode in @(0, 1)) {
            & gcc -std=c99 -Wall -Wextra -Wconversion -Wsign-conversion -Werror -pedantic "-I$sourceDirectory" -include (Join-Path $PSScriptRoot 'boot_stub.h') "-DROBOT_MODE=$robotMode" "-DAPP_TEST=$testMode" "-DS4_IMU_USE_MOCK=$mockMode" @testSources -o $testExecutable
            if ($LASTEXITCODE -ne 0) { throw 'Startup test compilation failed' }
            & $testExecutable
            if ($LASTEXITCODE -ne 0) { throw 'Startup test failed' }
            Write-Output "PASS: robot=$robotMode test=$testMode mock=$mockMode"
        }
    }
}
