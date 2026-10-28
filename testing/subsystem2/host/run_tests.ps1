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
