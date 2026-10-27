# VO + INS evaluation: prepare inputs, run imitator, copy result, plot.
# Prints only the metrics table (same layout as tools\plot_fusion_results.py).
#
# Usage (from anywhere):
#   .\add-kalman-report\tools\run_eval.ps1
#   .\add-kalman-report\tools\run_eval.ps1 -Rebuild
param(
    [switch]$Rebuild
)

$ErrorActionPreference = "Continue"
$env:PYTHONIOENCODING = "utf-8"

# Компилятор не фиксируется. Берётся тот, что уже в PATH (g++, clang++ или cl).
# Если в PATH пусто, ищутся типичные установки: MSYS2, MinGW-w64, TDM-GCC, Dev-C++, LLVM.
function Add-KnownCompilerPath {
    foreach ($name in @("g++", "clang++", "cl")) {
        if (Get-Command $name -ErrorAction SilentlyContinue) { return }
    }
    $dirs = @(
        "C:\msys64\ucrt64\bin",
        "C:\msys64\mingw64\bin",
        "C:\msys64\clang64\bin",
        "C:\msys64\mingw32\bin",
        "C:\mingw64\bin",
        "C:\mingw32\bin",
        "C:\MinGW\bin",
        "C:\TDM-GCC-64\bin",
        "$env:ProgramFiles\LLVM\bin",
        "${env:ProgramFiles(x86)}\LLVM\bin"
    )
    foreach ($root in @(
        "$env:ProgramFiles\Dev-Cpp",
        "${env:ProgramFiles(x86)}\Dev-Cpp",
        "$env:ProgramFiles\Embarcadero\Dev-Cpp",
        "${env:ProgramFiles(x86)}\Embarcadero\Dev-Cpp",
        "C:\Dev-Cpp"
    )) {
        if (-not (Test-Path $root)) { continue }
        $hit = Get-ChildItem $root -Filter "g++.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($hit) { $dirs += $hit.DirectoryName }
    }
    foreach ($dir in $dirs) {
        if (-not $dir) { continue }
        foreach ($name in @("g++.exe", "clang++.exe", "cl.exe")) {
            if (Test-Path (Join-Path $dir $name)) {
                $env:Path = "$dir;" + $env:Path
                Write-Host "Compiler path: $dir"
                return
            }
        }
    }
}
Add-KnownCompilerPath

$ToolsDir = $PSScriptRoot
$ReportDir = Split-Path -Parent $ToolsDir
$Root = Split-Path -Parent $ReportDir
$BuildDir = Join-Path $ReportDir "build"
function Find-Python {
    $tries = @(
        @{ Exe = "py"; Args = @("-3") },
        @{ Exe = "python"; Args = @() }
    )
    foreach ($t in $tries) {
        if (-not (Get-Command $t.Exe -ErrorAction SilentlyContinue)) { continue }
        & $t.Exe @($t.Args) -c "import numpy, matplotlib" 2>$null
        if ($LASTEXITCODE -eq 0) { return $t }
    }
    if (Get-Command py -ErrorAction SilentlyContinue) {
        foreach ($line in @(& py -0p 2>$null)) {
            if ($line -match '([A-Za-z]:\\.*python\.exe)') {
                & $Matches[1] -c "import numpy, matplotlib" 2>$null
                if ($LASTEXITCODE -eq 0) { return @{ Exe = $Matches[1]; Args = @() } }
            }
        }
    }
    Write-Host "Python with numpy and matplotlib was not found. Install dependencies:"
    Write-Host "  python -m pip install -r requirements.txt"
    exit 1
}
$PyCmd = Find-Python
$Py = $PyCmd.Exe
$PyArgs = $PyCmd.Args
$DataDir = Join-Path $ReportDir "data\vo_hk03"
$ParamsIni = Join-Path $DataDir "params.ini"
$Imitator = Join-Path $BuildDir "imitator.exe"
$FusionDir = Join-Path $Root "results_fusion"
$ResultSrc = Join-Path $ToolsDir "result.txt"
$ResultDst = Join-Path $FusionDir "result.txt"

function Show-Log($Log) {
    foreach ($line in @($Log)) {
        if ($null -ne $line -and "$line" -ne "") {
            Write-Host $line
        }
    }
}

function Stop-Eval($Log, [int]$Code) {
    Show-Log $Log
    exit $Code
}

function Invoke-CMakeConfigure {
    # Первая удачная связка: Ninja, затем Makefiles для g++, затем генератор CMake по умолчанию
    # (Visual Studio, если он установлен и компилятор GCC/Clang не нашёлся).
    $cxx = $null
    if (Get-Command g++ -ErrorAction SilentlyContinue) { $cxx = (Get-Command g++).Source }
    elseif (Get-Command clang++ -ErrorAction SilentlyContinue) { $cxx = (Get-Command clang++).Source }

    $attempts = @()
    if (Get-Command ninja -ErrorAction SilentlyContinue) {
        $attempts += @{ Name = "Ninja"; Multi = $false }
    }
    if ($cxx -and ((Split-Path -Leaf $cxx) -like "g++*")) {
        $attempts += @{ Name = "MinGW Makefiles"; Multi = $false }
    }
    $attempts += @{ Name = ""; Multi = $true }

    $last = @()
    foreach ($attempt in $attempts) {
        if (Test-Path $BuildDir) { Remove-Item $BuildDir -Recurse -Force -ErrorAction SilentlyContinue }
        $cmakeArgs = @("-S", $ReportDir, "-B", $BuildDir)
        if ($attempt.Name) { $cmakeArgs += @("-G", $attempt.Name) }
        if (-not $attempt.Multi) { $cmakeArgs += "-DCMAKE_BUILD_TYPE=Release" }
        if ($cxx -and $attempt.Name) { $cmakeArgs += "-DCMAKE_CXX_COMPILER=$cxx" }
        $label = $(if ($attempt.Name) { $attempt.Name } else { "default" })
        Write-Host "cmake -G $label"
        $last = & cmake @cmakeArgs 2>&1
        if ($LASTEXITCODE -eq 0) { return $attempt.Multi }
    }
    Stop-Eval $last $LASTEXITCODE
}

if ($Rebuild -or -not (Test-Path $Imitator)) {
    $multiConfig = $false
    if (-not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))) {
        $multiConfig = Invoke-CMakeConfigure
    } else {
        $cache = Get-Content (Join-Path $BuildDir "CMakeCache.txt") -ErrorAction SilentlyContinue
        $multiConfig = @($cache | Where-Object { $_ -match '^CMAKE_CONFIGURATION_TYPES:[^=]*=.+' }).Count -gt 0
    }
    $buildArgs = @("--build", $BuildDir)
    if ($multiConfig) { $buildArgs += @("--config", "Release") }
    $log = & cmake @buildArgs 2>&1
    if ($LASTEXITCODE -ne 0) { Stop-Eval $log $LASTEXITCODE }
    if (-not (Test-Path $Imitator)) {
        $releaseExe = Join-Path $BuildDir "Release\imitator.exe"
        if (Test-Path $releaseExe) { $Imitator = $releaseExe }
    }
}

$log = & $Py @PyArgs (Join-Path $ToolsDir "prepare_vo_inputs.py") 2>&1
if ($LASTEXITCODE -ne 0) { Stop-Eval $log $LASTEXITCODE }

Push-Location $BuildDir
try {
    # argv[2] is the only place main.cpp loads params.ini.
    $log = & $Imitator $DataDir $ParamsIni 2>&1
    $imitatorCode = $LASTEXITCODE
}
finally {
    Pop-Location
}
if ($imitatorCode -ne 0) { Stop-Eval $log $imitatorCode }
if (-not (Test-Path $ResultSrc)) {
    Stop-Eval "imitator did not write $ResultSrc" 1
}

New-Item -ItemType Directory -Force -Path $FusionDir | Out-Null
Copy-Item $ResultSrc $ResultDst -Force

$plotStarted = Get-Date
$log = & $Py @PyArgs -X utf8 (Join-Path $Root "tools\plot_fusion_results.py") 2>&1
$plotCode = $LASTEXITCODE
$jsonPath = Join-Path $FusionDir "metrics_summary.json"
$jsonFresh = (Test-Path $jsonPath) -and ((Get-Item $jsonPath).LastWriteTime -ge $plotStarted.AddSeconds(-2))
if ($plotCode -ne 0 -or -not $jsonFresh) { Stop-Eval $log $(if ($plotCode) { $plotCode } else { 1 }) }

$s = Get-Content $jsonPath -Raw -Encoding UTF8 | ConvertFrom-Json
$vo = $s.vo_only
$ins = $s.vo_ins_kalman
$imp = $s.improvement_pct

# Invariant culture: plot_fusion_results.py always prints a dot, not a locale comma.
$inv = [System.Globalization.CultureInfo]::InvariantCulture
function Format-Num([double]$Value) {
    return $Value.ToString("0.00", $inv).PadLeft(8)
}
function Format-Pct([double]$Value) {
    return ($Value.ToString("+0.0;-0.0", $inv).PadLeft(6) + "%")
}

# Column spacing matches the f-string in tools\plot_fusion_results.py.
Write-Output @"
╔══════════════════════════════════════════════════════╗
║  Метрики качества                                    ║
╚══════════════════════════════════════════════════════╝

                     ВО (только)  ВО+ИНС+Калман  Улучшение
────────────────────────────────────────────────────────
RMS ошибка, м        $(Format-Num $vo.rms)      $(Format-Num $ins.rms)     $(Format-Pct $imp.rms)
Макс. ошибка, м      $(Format-Num $vo.max)      $(Format-Num $ins.max)     $(Format-Pct $imp.max)
Ошибка на финише, м  $(Format-Num $vo.end)      $(Format-Num $ins.end)     $(Format-Pct $imp.end)
────────────────────────────────────────────────────────
"@
