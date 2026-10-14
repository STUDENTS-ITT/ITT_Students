# run_tune_truth.ps1 — полный подбор коэффициентов по НАЗЕМНОЙ ИСТИНЕ (этап 13A).
#
# Отличие от run_tune_all.ps1: запускает tune_kalman.py с --ref-truth и
# конфигами из truth_cfg/ (work_root=temp_tune_truth_<набор>, пресеты в
# presets/truth/tuned_<набор>.ini — штатные presets/tuned_<набор>.ini не
# перезаписываются). Набор square_noisefree исключён (повреждён truth.txt).
#
# Запуск:
#   pwsh -File tools/kalman_tuner/run_tune_truth.ps1
# Подмножество:
#   pwsh -File tools/kalman_tuner/run_tune_truth.ps1 -SetsCsv static300,square650

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
  $cfg = Join-Path $scriptDir "truth_cfg\cfg_$s.json"
  $log = Join-Path $logDir "full_truth_$s.log"
  $tuner = Join-Path $scriptDir "tune_kalman.py"
  $ts = Get-Date -Format "HH:mm:ss"
  "[$ts] START truth $s" | Write-Output
  "==== START truth $s ($ts) ====" | Add-Content $log
  $t0 = Get-Date
  & "C:\Users\anton\AppData\Local\Programs\Python\Python310\python.exe" $tuner "--config=$cfg" "--ref-truth" *>> $log
  $code = $LASTEXITCODE
  $dur = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)
  "[$(Get-Date -Format HH:mm:ss)] END truth $s exit=$code ${dur} min" | Write-Output
  "==== END truth $s exit=$code ${dur} min ====" | Add-Content $log
  $manifest += ("{0} truth {1} exit={2} {3}min" -f $s, "", $code, $dur)
}

$manifest | Set-Content (Join-Path $logDir "_summary_truth.txt")
"`nВСЁ ГОТОВО. Сводка: $(Join-Path $logDir '_summary_truth.txt')"
