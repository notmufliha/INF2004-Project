$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$sourceDir = Join-Path $projectRoot 'app_program/subsystem4'

foreach ($testName in @('imu_mock_test', 'imu_analysis_test', 'imu_demo_test', 'imu_processing_test', 'gy511_test')) {
    $testSources = @((Join-Path $sourceDir 'imu_mock.c'))
    if ($testName -ne 'imu_mock_test') {
        $testSources += Join-Path $sourceDir 'imu_analysis.c'
    }
    if ($testName -eq 'imu_demo_test') {
        $testSources += Join-Path $sourceDir 'imu_demo.c'
    }
    if ($testName -in @('imu_demo_test', 'imu_processing_test')) {
        $testSources += Join-Path $sourceDir 'imu_processing.c'
    }
    if ($testName -eq 'gy511_test') {
        $testSources += Join-Path $sourceDir 'drv_gy511.c'
    }
    $testSources += Join-Path $PSScriptRoot "$testName.c"
    $testExecutable = Join-Path $PSScriptRoot "$testName.exe"
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-I$sourceDir" @testSources -lm -o $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $testName" }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Tests failed: $testName" }
}
