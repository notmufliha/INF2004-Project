$ErrorActionPreference = 'Stop'
$recorderBinary = Join-Path $env:TEMP 'inf2004-run-log-test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-cast-function-type `
    (Join-Path $PSScriptRoot 'run_log_test.c') -o $recorderBinary
if ($LASTEXITCODE -ne 0) { throw 'Run recorder compilation failed' }
& $recorderBinary
if ($LASTEXITCODE -ne 0) { throw 'Run recorder tests failed' }
$barcodeBinary = Join-Path $env:TEMP 'inf2004-v4-barcode-test.exe'
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type -Wno-unused-function `
    -DVC_FOLLOW_V4=1 -DROBOT_BOX_BYPASS=0 `
    (Join-Path $PSScriptRoot 'v4_barcode_test.c') -o $barcodeBinary
if ($LASTEXITCODE -ne 0) { throw 'V4 barcode compilation failed' }
& $barcodeBinary
if ($LASTEXITCODE -ne 0) { throw 'V4 barcode tests failed' }

$binary = Join-Path $env:TEMP 'inf2004-controller-test.exe'
# controller_test.c covers the previous controller: build it with v4 off.
& gcc -std=c11 -mno-ms-bitfields -Wall -Wextra -Werror -Wno-cast-function-type -DVC_FOLLOW_V4=0 -DROBOT_BOX_BYPASS=0 (Join-Path $PSScriptRoot 'controller_test.c') -o $binary
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

$v4Binary = Join-Path $env:TEMP 'inf2004-vc-follow-test.exe'
& gcc -std=c11 -Wall -Wextra -Werror (Join-Path $PSScriptRoot 'vc_follow_test.c') (Join-Path $PSScriptRoot '..\..\app_program\vc_follow.c') -o $v4Binary
if ($LASTEXITCODE -ne 0) { throw 'vc_follow compilation failed' }
& $v4Binary
if ($LASTEXITCODE -ne 0) { throw 'vc_follow tests failed' }
