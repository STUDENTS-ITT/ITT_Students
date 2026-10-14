# run_tune_13b.ps1 — подбор с АСИММЕТРИЧНЫМИ интервалами и wall penalty (этап 13B).
#
# Использует конфиги из truth_cfg_13b/ (u_lo/u_hi + optim.wall_penalty) и
# --ref-truth (метрика по ground truth). Пресеты пишутся в
# presets/truth_13b/tuned_<набор>.ini (13A-пресеты not touched).
# Набор square_noisefree исключён (повреждён truth.txt).
#
# Запуск:
#   pwsh -File tools/kalman_tuner/run_tune_13b.ps1
# Подмножество:
#   pwsh -File tools/kalman_tuner/run_tune_13b.ps1 -SetsCsv static300,square650

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
  $cfg = Join-Path $scriptDir "truth_cfg_13b\cfg_$s.json"
  $log = Join-Path $logDir "full_13b_$s.log"
  $tuner = Join-Path $scriptDir "tune_kalman.py"
  $ts = Get-Date -Format "HH:mm:ss"
  "[$ts] START 13b $s" | Write-Output
  "==== START 13b $s ($ts) ====" | Add-Content $log
  $t0 = Get-Date
  & "C:\Users\anton\AppData\Local\Programs\Python\Python310\python.exe" $tuner "--config=$cfg" "--ref-truth" *>> $log
  $code = $LASTEXITCODE
  $dur = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)
  "[$(Get-Date -Format HH:mm:ss)] END 13b $s exit=$code ${dur} min" | Write-Output
  "==== END 13b $s exit=$code ${dur} min ====" | Add-Content $log
  $manifest += ("{0} 13b exit={1} {2}min" -f $s, $code, $dur)
}

$manifest | Set-Content (Join-Path $logDir "_summary_13b.txt")
"`nВСЁ ГОТОВО. Сводка: $(Join-Path $logDir '_summary_13b.txt')"
