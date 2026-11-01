$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$executable = Join-Path $env:TEMP 'inf2004-subsystem2-motion-test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -pedantic `
    (Join-Path $PSScriptRoot 'motion_test.c') `
    (Join-Path $projectRoot 'app_program/subsystem2/motion_profile.c') `
    -lm -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Subsystem 2 host compilation failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Subsystem 2 host tests failed' }
foreach ($binary in @(0, 1)) {
    $integrationExe = Join-Path $env:TEMP "inf2004-b2-integration-$binary.exe"
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-DB2_TEST_BINARY=$binary" `
        (Join-Path $PSScriptRoot 'integration_test.c') -o $integrationExe
    if ($LASTEXITCODE -ne 0) { throw 'B2 integration compilation failed' }
    & $integrationExe
    if ($LASTEXITCODE -ne 0) { throw 'B2 integration test failed' }
}
