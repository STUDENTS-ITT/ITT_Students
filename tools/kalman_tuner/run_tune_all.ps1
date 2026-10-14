# run_tune_all.ps1 — ночной полный (не quick) подбор коэффициентов на наборах dataset_shum.
#
# Последовательно запускает tune_kalman.py в штатном режиме (без --quick, N=64/80/32)
# для каждого набора. При сбое отдельного набора продолжает со следующего.
# Логи: <repo>/tools/kalman_tuner/full_runs/full_<набор>.log; сводка — _summary.txt.
#
# Запуск (из корня репо):
#   pwsh -File tools/kalman_tuner/run_tune_all.ps1
# Подмножество наборов (повтор прогона после сбоя/исправления конфига):
#   pwsh -File tools/kalman_tuner/run_tune_all.ps1 -SetsCsv static300,square650
#
# ВНИМАНИЕ: имена конфигов — по историческому соглашению без подчёркивания
# (tuner_config.static300.json, tuner_config.square650.json), каталоги данных —
# с подчёркиванием (static_300, square_650). Несовпадение имени конфига молча
# уводит tune_kalman.py на дефолтный tuner_config.json (fallback run_hk).

param(
    [string]$SetsCsv = ""
)

$ErrorActionPreference = "Continue"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = (Resolve-Path (Join-Path $scriptDir "..\..")).Path
$logDir = Join-Path $scriptDir "full_runs"
New-Item -ItemType Directory -Path $logDir -Force | Out-Null

$env:PYTHONIOENCODING = "utf-8"
$env:PATH = "C:/Users/anton/.mingw64/mingw64/bin;$env:PATH"

if ($SetsCsv.Trim()) {
    $sets = @($SetsCsv -split ',')
} else {
    $sets = @(
        "static300",
        "static_lownoise_align",
        "square650",
        "square_noisefree",
        "square_harsh",
        "straight_10ms",
        "figure8_8ms",
        "spiral_climb",
        "high_dynamics"
    )
}

$manifest = @()
foreach ($s in $sets) {
  $cfg = Join-Path $scriptDir "tuner_config.$s.json"
  $log = Join-Path $logDir "full_$s.log"
  $tuner = Join-Path $scriptDir "tune_kalman.py"
  "`n========== START $s $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ==========" | Add-Content $log

  $t0 = Get-Date
  Write-Host "[$(Get-Date -Format 'HH:mm:ss')] START $s"
  python -u $tuner --config=$cfg *>> $log
  $code = $LASTEXITCODE
  $dur = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)

  if ($code -eq 0) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] DONE $s за ${dur} мин"
    $manifest += "OK   $s  ${dur} мин"
  } else {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] FAIL $s exit=$code за ${dur} мин"
    $manifest += "FAIL $s exit=$code за ${dur} мин"
  }
  "========== END $s exit=$code ${dur} мин ==========" | Add-Content $log
}

$manifest | Set-Content (Join-Path $logDir "_summary.txt")
"`nВСЁ ГОТОВО. Сводка: $(Join-Path $logDir '_summary.txt')"