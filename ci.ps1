# ci.ps1 — локальный непрерывный прогон: configure + build + ctest.
#
# Запуск из корня репозитория:
#   pwsh -File ci.ps1
#
# Параметры:
#   -BuildDir   каталог сборки (по умолчанию build)
#   -Generator  генератор для свежей сборки (по умолчанию Ninja); существующий
#               каталог сборки переиспользуется без смены генератора
#   -Jobs       параллельность (по умолчанию 8)

param(
    [string]$BuildDir = "build",
    [string]$Generator = "Ninja",
    [int]$Jobs = 8
)

$ErrorActionPreference = "Stop"

$root = $PSScriptRoot
if (-not $PSScriptRoot) { $root = Get-Location }
$build = Join-Path $root $BuildDir
$cache = Join-Path $build "CMakeCache.txt"

Write-Host "== ci: configure =="
$args = @("-S", $root, "-B", $build)
if (-not (Test-Path -LiteralPath $cache)) {
    $args += @("-G", $Generator, "-DCMAKE_BUILD_TYPE=Release")
    Write-Host "  свежая сборка: генератор $Generator"
} else {
    Write-Host "  переиспользуем существующий каталог сборки"
}
& cmake @args
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "== ci: build =="
& cmake --build $build --parallel $Jobs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "== ci: test =="
& ctest --test-dir $build --output-on-failure
$exit = $LASTEXITCODE
if ($exit -eq 0) {
    Write-Host "== ci: OK =="
} else {
    Write-Host "== ci: FAILED (exit $exit) =="
}
exit $exit