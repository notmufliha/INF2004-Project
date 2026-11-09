$ErrorActionPreference = 'Stop'
$binary = Join-Path $env:TEMP 'inf2004-controller-test.exe'
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type (Join-Path $PSScriptRoot 'controller_test.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller compilation failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed' }

$code39Binary = Join-Path $env:TEMP 'inf2004-code39-test.exe'
$code39Source = Join-Path $PSScriptRoot '..\subsystem3\code39_test.c'
$code39Module = Join-Path $PSScriptRoot '..\..\app_program\subsystem3\b3_code39.c'
$code39Include = Join-Path $PSScriptRoot '..\..\app_program\subsystem3'
& gcc -std=c11 -Wall -Wextra -Werror "-I$code39Include" $code39Source $code39Module -o $code39Binary
if ($LASTEXITCODE -ne 0) { throw 'Code 39 compilation failed' }
& $code39Binary
if ($LASTEXITCODE -ne 0) { throw 'Code 39 tests failed' }
