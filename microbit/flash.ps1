# Compileert main.ts en zet het op de micro:bit (Windows, zonder wmic).
# Gebruik (in deze map, micro:bit via USB aangesloten):
#   powershell -ExecutionPolicy Bypass -File .\flash.ps1

makecode build
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compileren mislukt, zie de fout hierboven." -ForegroundColor Red
    exit 1
}

$vol = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'MICROBIT' } | Select-Object -First 1
if (-not $vol -or -not $vol.DriveLetter) {
    Write-Host "MICROBIT-schijf niet gevonden. Hangt de micro:bit met de USB-kabel aan de computer?" -ForegroundColor Red
    exit 1
}

$doel = "$($vol.DriveLetter):\"
Copy-Item -Path "built\binary.hex" -Destination $doel
Write-Host "Geflasht naar $doel  - wacht tot het gele lampje stopt met knipperen." -ForegroundColor Green
