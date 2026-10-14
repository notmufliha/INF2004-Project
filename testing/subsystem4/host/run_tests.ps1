$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$sourceDir = Join-Path $projectRoot 'app_program/subsystem4'

foreach ($testName in @('imu_mock_test', 'imu_analysis_test')) {
    $testSources = @((Join-Path $sourceDir 'imu_mock.c'))
    if ($testName -eq 'imu_analysis_test') {
        $testSources += Join-Path $sourceDir 'imu_analysis.c'
    }
    $testSources += Join-Path $PSScriptRoot "$testName.c"
    $testExecutable = Join-Path $PSScriptRoot "$testName.exe"
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-I$sourceDir" @testSources -lm -o $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $testName" }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Tests failed: $testName" }
}
