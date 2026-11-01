param([string]$Compiler = 'arm-none-eabi-gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$sourceDirectory = Join-Path $projectRoot 'app_program/subsystem4'
$compilerArguments = @(
    '-mcpu=cortex-m0plus', '-mthumb', '-ffreestanding', '-std=c99',
    '-Wall', '-Wextra', '-Wconversion', '-Wsign-conversion', '-Werror',
    '-Wno-cast-function-type', '-D_PICO_RP2040_', '-DCNF_SMP=1',
    "-I$(Join-Path $projectRoot 'include')",
    "-I$(Join-Path $projectRoot 'config')",
    "-I$(Join-Path $projectRoot 'kernel/knlinc')", '-fsyntax-only'
)

foreach ($mockMode in @(0, 1)) {
    foreach ($testMode in @(0, 1)) {
        foreach ($sourceFile in (Get-ChildItem $sourceDirectory -Filter '*.c')) {
            & $Compiler @compilerArguments "-DAPP_TEST=$testMode" "-DS4_IMU_USE_MOCK=$mockMode" $sourceFile.FullName
            if ($LASTEXITCODE -ne 0) {
                throw "ARM C99 check failed: $($sourceFile.Name), mock=$mockMode, test=$testMode"
            }
        }
        Write-Output "PASS: ARM C99 B4 mock=$mockMode test=$testMode"
    }
}
