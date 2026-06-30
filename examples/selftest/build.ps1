# build.ps1 — Baut alle Selbsttest-Programme zu Intel-HEX (.hex).
#
# Nutzt die arm-none-eabi-Toolchain der Pico-VS-Code-Erweiterung
# (~/.pico-sdk/toolchain/14_2_Rel1/bin); fällt sonst auf den PATH zurück.
#
# Aufruf:   .\build.ps1            # alle Tests bauen
#           .\build.ps1 t3_timer_irq
#
# Ergebnis: <name>.elf und <name>.hex je Testprogramm.

$ErrorActionPreference = "Stop"

# --- Toolchain finden -------------------------------------------------------
$tc = Join-Path $env:USERPROFILE ".pico-sdk\toolchain\14_2_Rel1\bin"
if (Test-Path (Join-Path $tc "arm-none-eabi-gcc.exe")) {
    $gcc     = Join-Path $tc "arm-none-eabi-gcc.exe"
    $objcopy = Join-Path $tc "arm-none-eabi-objcopy.exe"
    $size    = Join-Path $tc "arm-none-eabi-size.exe"
} else {
    $gcc     = "arm-none-eabi-gcc"
    $objcopy = "arm-none-eabi-objcopy"
    $size    = "arm-none-eabi-size"
}

$cflags = @(
    "-mcpu=cortex-m0", "-mthumb", "-Os", "-g",
    "-ffunction-sections", "-fdata-sections",
    "-Wall", "-Wextra",
    "-nostartfiles", "-Wl,--gc-sections",
    "-T", "lpc1115.ld"
)
$startup = "startup_lpc1115.c"

# --- Zu bauende Tests -------------------------------------------------------
if ($args.Count -gt 0) {
    $tests = $args | ForEach-Object { [System.IO.Path]::GetFileNameWithoutExtension($_) }
} else {
    $tests = Get-ChildItem -Filter "t*.c" | ForEach-Object { $_.BaseName }
}

foreach ($t in $tests) {
    $src = "$t.c"
    if (-not (Test-Path $src)) { Write-Warning "fehlt: $src"; continue }
    Write-Host "==> $t" -ForegroundColor Cyan
    & $gcc @cflags $startup $src -o "$t.elf"
    & $objcopy -O ihex "$t.elf" "$t.hex"
    & $size "$t.elf"
    Write-Host "    -> $t.hex" -ForegroundColor Green
}
