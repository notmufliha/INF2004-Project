$ErrorActionPreference = 'Stop'
$binary = Join-Path $env:TEMP 'inf2004-controller-test.exe'
& gcc -std=c11 -DROBOT_CLEAR_PATH_FORWARD=0 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type (Join-Path $PSScriptRoot 'controller_test.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller compilation failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed' }
$forwardBinary = Join-Path $env:TEMP 'inf2004-clear-forward-test.exe'
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type (Join-Path $PSScriptRoot 'clear_forward_test.c') -o $forwardBinary
if ($LASTEXITCODE -ne 0) { throw 'Clear-forward compilation failed' }
& $forwardBinary
if ($LASTEXITCODE -ne 0) { throw 'Clear-forward tests failed' }

$code39Binary = Join-Path $env:TEMP 'inf2004-code39-test.exe'
$code39Source = Join-Path $PSScriptRoot '..\subsystem3\code39_test.c'
$code39Module = Join-Path $PSScriptRoot '..\..\app_program\subsystem3\b3_code39.c'
$code39Include = Join-Path $PSScriptRoot '..\..\app_program\subsystem3'
& gcc -std=c11 -Wall -Wextra -Werror "-I$code39Include" $code39Source $code39Module -o $code39Binary
if ($LASTEXITCODE -ne 0) { throw 'Code 39 compilation failed' }
& $code39Binary
if ($LASTEXITCODE -ne 0) { throw 'Code 39 tests failed' }

$lineBinary = Join-Path $env:TEMP 'inf2004-line-cal-test.exe'
& gcc -std=c11 -Wall -Wextra -Werror "-I$code39Include" `
    (Join-Path $PSScriptRoot '../subsystem3/line_cal_test.c') `
    (Join-Path $code39Include 'b3_line_cal.c') -o $lineBinary
if ($LASTEXITCODE -ne 0) { throw 'Line calibration compilation failed' }
& $lineBinary
if ($LASTEXITCODE -ne 0) { throw 'Line calibration tests failed' }

$lineInputBinary = Join-Path $env:TEMP 'inf2004-line-input-test.exe'
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type `
    (Join-Path $PSScriptRoot '..\subsystem5\test_line_input.c') `
    (Join-Path $code39Include 'b3_line_cal.c') $code39Module -o $lineInputBinary
if ($LASTEXITCODE -ne 0) { throw 'Line input compilation failed' }
& $lineInputBinary
if ($LASTEXITCODE -ne 0) { throw 'Line input tests failed' }
