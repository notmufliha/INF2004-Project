<#
.SYNOPSIS
  Save the car's run recording (app_program/run_log.c) to logs/.

.DESCRIPTION
  1. Drive the car (no USB cable needed), stop it.
  2. Plug the Pico into this PC. Close any other serial monitor
     (PuTTY, VS Code serial monitor, Arduino, ...).
  3. From the repository folder run:
        powershell -ExecutionPolicy Bypass -File tools\capture_run_log.ps1
     (add -Port COM5 if it picks the wrong port).
  4. Commit and push the new files in logs\ :
        git add logs
        git commit -m "Run log: <what you tested>"
        git push

  Opening the port makes the car print its recent ~16-24 s of driving as CSV
  (one row per 20 ms plus events: BARCODE_READ, JUNCTION, NAV_START, ...).
  Writes logs\run-<time>.csv (the recording) and logs\run-<time>.txt
  (everything the car printed, including the [B3]/[VC] messages).
  Diagnostic event rows LINE_AGE, LOOP_TIME and MOTION_AGE use milliseconds;
  MEAS_LEFT / MEAS_RIGHT use signed counts/s. Match their t_ms to the nearby
  sensor/steering rows. Ages or loop intervals above 40 ms indicate a timing
  delay to investigate, not proof of a sensor failure.
#>
param(
    [string]$Port = "",
    [int]$TimeoutSeconds = 60,
    [string]$Note = ""
)
$ErrorActionPreference = 'Stop'

if ($Port -eq "") {
    # Raspberry Pi Pico USB serial: vendor ID 2E8A.
    $dev = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.PNPDeviceID -like 'USB\VID_2E8A*' -and $_.Name -match '\((COM\d+)\)' } |
        Select-Object -First 1
    if ($null -eq $dev) {
        $ports = [System.IO.Ports.SerialPort]::GetPortNames() -join ', '
        throw "No Pico found. Plug it in, or pass -Port COMx (ports now: $ports)"
    }
    $null = $dev.Name -match '\((COM\d+)\)'
    $Port = $Matches[1]
}

$root = Split-Path -Parent $PSScriptRoot
$logs = Join-Path $root 'logs'
New-Item -ItemType Directory -Force $logs | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$csvPath = Join-Path $logs "run-$stamp.csv"
$txtPath = Join-Path $logs "run-$stamp.txt"

Write-Host "Opening $Port ... (the car dumps its recording when the port opens)"
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.DtrEnable = $true     # the car starts the dump when DTR goes on
$sp.RtsEnable = $true
$sp.ReadTimeout = 500
$sp.NewLine = "`n"
$sp.Open()

$all = New-Object System.Collections.Generic.List[string]
$csv = New-Object System.Collections.Generic.List[string]
$started = $false
$done = $false
$deadline = (Get-Date).AddSeconds($TimeoutSeconds)
try {
    while (-not $done -and (Get-Date) -lt $deadline) {
        try { $line = $sp.ReadLine().TrimEnd("`r") } catch [System.TimeoutException] { continue }
        $all.Add($line)
        # Subsystem output may precede the marker on the same serial line.
        $markerIndex = $line.IndexOf('#RUNLOG')
        if ($markerIndex -ge 0) {
            $started = $true
            $marker = $line.Substring($markerIndex)
            $csv.Add($marker)
            Write-Host $marker
        } elseif ($line.StartsWith('t_ms,travel_mm,')) {
            # Recover even when the start marker itself was lost.
            $started = $true
            $csv.Add($line)
        } elseif ($started -and $line.StartsWith('#END')) {
            $csv.Add($line)
            Write-Host $line
            $done = $true
        } elseif ($started -and ($line.StartsWith('t_ms,') -or
                 ($line -match '^\d+,' -and ($line.Split(',').Count -eq 23)))) {
            $csv.Add($line)   # recording row (console messages are skipped)
            if (($csv.Count % 200) -eq 0) { Write-Host "  $($csv.Count) rows..." }
        }
    }
} finally {
    $sp.Close()
}

if ($Note -ne "") { $csv.Insert(0, "# note: $Note") }
[System.IO.File]::WriteAllLines($csvPath, $csv)
[System.IO.File]::WriteAllLines($txtPath, $all)
if (-not $done) {
    Write-Warning "No #END from the car (old firmware, port already open at boot, or timeout). Saved what arrived."
}
Write-Host "Saved $csvPath ($($csv.Count) lines) and $txtPath"
Write-Host "Now: git add logs; git commit -m `"Run log: ...`"; git push"
