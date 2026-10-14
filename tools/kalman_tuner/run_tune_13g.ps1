# run_tune_13g.ps1 — подбор с ФИКСИРОВАННЫМИ УНИВЕРСАЛЬНЫМИ симметричными
# интервалами (спаны 13A) + wall penalty (этап 13G).
#
# Конфиги — truth_cfg_13g/; метрика по наземной истине (--ref-truth). Пресеты
# пишутся в presets/truth_13g/tuned_<набор>.ini (13A/13B-пресеты не тронуты).
# Набор square_noisefree исключён (повреждён truth.txt).
#
# Запуск:
#   pwsh -File tools/kalman_tuner/run_tune_13g.ps1
# Подмножество:
#   pwsh -File tools/kalman_tuner/run_tune_13g.ps1 -SetsCsv static300,square650

param(
    [string]$SetsCsv = ""
)

$ErrorActionPreference = "Continue"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
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
        "square_harsh",
        "straight_10ms",
        "figure8_8ms",
        "spiral_climb",
        "high_dynamics"
    )
}

$manifest = @()
foreach ($s in $sets) {
  $cfg = Join-Path $scriptDir "truth_cfg_13g\cfg_$s.json"
  $log = Join-Path $logDir "full_13g_$s.log"
  $tuner = Join-Path $scriptDir "tune_kalman.py"
  $ts = Get-Date -Format "HH:mm:ss"
  "[$ts] START 13g $s" | Write-Output
  "==== START 13g $s ($ts) ====" | Add-Content $log
  $t0 = Get-Date
  & "C:\Users\anton\AppData\Local\Programs\Python\Python310\python.exe" $tuner "--config=$cfg" "--ref-truth" *>> $log
  $code = $LASTEXITCODE
  $dur = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)
  "[$(Get-Date -Format HH:mm:ss)] END 13g $s exit=$code ${dur} min" | Write-Output
  "==== END 13g $s exit=$code ${dur} min ====" | Add-Content $log
  $manifest += ("{0} 13g exit={1} {2}min" -f $s, $code, $dur)
}

$manifest | Set-Content (Join-Path $logDir "_summary_13g.txt")
"`nВСЁ ГОТОВО. Сводка: $(Join-Path $logDir '_summary_13g.txt')"
