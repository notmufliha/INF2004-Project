$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$sourceDir = Join-Path $projectRoot 'app_program/subsystem4'

foreach ($testName in @('testmock_main', 'testanal_main', 'testdemo_main', 'testproc_main', 'testgyro_main', 'testnum_main', 'testhigh_main')) {
    $testSources = @((Join-Path $PSScriptRoot 'imu_test_support.c'),
                     (Join-Path $sourceDir 'imu_mock.c'),
                     (Join-Path $sourceDir 'imu_numeric.c'))
    if ($testName -ne 'testmock_main') {
        $testSources += Join-Path $sourceDir 'imu_analysis.c'
    }
    if ($testName -eq 'testdemo_main') {
        $testSources += Join-Path $sourceDir 'imu_demo.c'
    }
    if ($testName -in @('testdemo_main', 'testproc_main', 'testhigh_main')) {
        $testSources += Join-Path $sourceDir 'imu_processing.c'
    }
    if ($testName -eq 'testgyro_main') {
        $testSources += Join-Path $sourceDir 'gy511.c'
    }
    if ($testName -eq 'testhigh_main') {
        $testSources += Join-Path $sourceDir 'imu_height.c'
    }
    $testSources += Join-Path $PSScriptRoot "$testName.c"
    $testExecutable = Join-Path $PSScriptRoot "$testName.exe"
    & gcc -std=c99 -Wall -Wextra -Wconversion -Wsign-conversion -Werror -pedantic "-I$sourceDir" @testSources -lm -o $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $testName" }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "Tests failed: $testName" }
}
