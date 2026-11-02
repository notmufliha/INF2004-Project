$ErrorActionPreference = 'Stop'
$binary = Join-Path $env:TEMP 'inf2004-controller-test.exe'
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type (Join-Path $PSScriptRoot 'controller_test.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller compilation failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed' }
