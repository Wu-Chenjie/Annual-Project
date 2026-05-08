<#
    scripts/reproduce.ps1
    一键复现 (Windows PowerShell)：环境检查 → 全量 pytest → 关键场景仿真 → benchmark → 结果汇总。

    用法:
        powershell -ExecutionPolicy Bypass -File scripts/reproduce.ps1
        powershell -ExecutionPolicy Bypass -File scripts/reproduce.ps1 -SkipTests
        powershell -ExecutionPolicy Bypass -File scripts/reproduce.ps1 -Quick

    设计原则：
    - 与 reproduce.sh 完全一致的步骤与参数；
    - 任何一步失败立即退出，所有产物落到同一时间戳目录。
#>

[CmdletBinding()]
param(
    [switch]$SkipTests,
    [switch]$Quick
)

$ErrorActionPreference = 'Stop'

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
Set-Location $ProjectRoot

$RunName = Get-Date -Format 'yyyyMMdd-HHmmss'
$SummaryDir = Join-Path 'outputs' (Join-Path '_reproduce' $RunName)
New-Item -ItemType Directory -Force -Path $SummaryDir | Out-Null
$SummaryFile = Join-Path $SummaryDir 'summary.txt'

if ($Quick) {
    $Presets = @('basic')
    $BenchRuns = 1
} else {
    $Presets = @('basic', 'warehouse_danger', 'school_corridor_online')
    $BenchRuns = 3
}

function Write-Log($msg) {
    $line = "[reproduce] $msg"
    Write-Host $line
    Add-Content -Path $SummaryFile -Value $line
}

function Invoke-Step($cmd) {
    Write-Log "  -> $cmd"
    $output = & cmd /c "$cmd 2>&1"
    $output | ForEach-Object { Add-Content -Path $SummaryFile -Value $_ }
    $output | Write-Host
    if ($LASTEXITCODE -ne 0) {
        Write-Log "[FAIL] exit=$LASTEXITCODE while running: $cmd"
        throw "Step failed: $cmd"
    }
}

Write-Log "PROJECT_ROOT = $ProjectRoot"
Write-Log "RUN_NAME     = $RunName"

# ---- 1) 环境检查 ----
Write-Log 'step 1/4: 环境检查'
Invoke-Step 'python -c "import sys, numpy, scipy, matplotlib; print(''python'', sys.version.split()[0]); print(''numpy'', numpy.__version__); print(''scipy'', scipy.__version__)"'

# ---- 2) 全量测试 ----
if (-not $SkipTests) {
    Write-Log 'step 2/4: pytest'
    Invoke-Step 'python -m pytest -q'
} else {
    Write-Log 'step 2/4: pytest (skipped)'
}

# ---- 3) 关键场景仿真 ----
Write-Log "step 3/4: 关键场景仿真 (presets: $($Presets -join ', '))"
foreach ($preset in $Presets) {
    Invoke-Step "python main.py --preset $preset --run-name $RunName --no-plot"
}

# ---- 4) benchmark ----
Write-Log "step 4/4: benchmark (runs=$BenchRuns)"
$benchOut = "outputs/benchmark_default/$RunName/benchmark_results.json"
$benchPy = @"
from simulations.benchmark import run_benchmark
import pathlib
out = pathlib.Path(r'$benchOut')
res = run_benchmark(runs=$BenchRuns, output_file=str(out), preset='benchmark_default')
print('benchmark wrote:', out)
print('worst_case_max_error =', res['summary']['worst_case_max_error'])
"@
$benchPy | Set-Content -Path (Join-Path $SummaryDir '_run_benchmark.py') -Encoding utf8
Invoke-Step "python `"$($SummaryDir.Replace('\','/'))/_run_benchmark.py`""

Write-Log '全部完成。结果根目录:'
Write-Log "  outputs/<preset>/$RunName/sim_result.json"
Write-Log "  $benchOut"
Write-Log "  $SummaryFile"
