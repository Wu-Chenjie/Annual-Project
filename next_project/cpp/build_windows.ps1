param(
    [string]$Drive = "Y",
    [string]$BuildDirName = "build_windows",
    [string]$Target = "sim_dynamic_replay",
    [int]$Jobs = 4
)

$ErrorActionPreference = "Stop"

$CppDir = Split-Path -Parent $PSCommandPath
$ProjectRoot = Split-Path -Parent $CppDir
$driveName = $Drive.TrimEnd(":")
$driveRoot = "${driveName}:\"
$createdSubst = $false

$toolRoot = "C:\tools\winlibs-gcc-clang\mingw64\bin"
$cmake = Join-Path $toolRoot "cmake.exe"
$make = Join-Path $toolRoot "mingw32-make.exe"
$cxx = Join-Path $toolRoot "g++.exe"

function Invoke-Native {
    param(
        [string]$FilePath,
        [string[]]$Arguments
    )
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $FilePath $($Arguments -join ' ')"
    }
}

foreach ($tool in @($cmake, $make, $cxx)) {
    if (-not (Test-Path -LiteralPath $tool)) {
        throw "Required build tool not found: $tool"
    }
}

try {
    if (-not (Test-Path -LiteralPath $driveRoot)) {
        & cmd.exe /C "subst ${driveName}: `"$ProjectRoot`""
        $createdSubst = $true
    }

    if (-not (Test-Path -LiteralPath "$driveRoot\cpp\CMakeLists.txt")) {
        throw "Mapped drive ${driveName}: does not point to next_project: $driveRoot"
    }

    $src = "${driveName}:/cpp"
    $build = "${driveName}:/cpp/$BuildDirName"
    Invoke-Native $cmake @(
        "-S", $src,
        "-B", $build,
        "-G", "MinGW Makefiles",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_MAKE_PROGRAM=$make",
        "-DCMAKE_CXX_COMPILER=$cxx"
    )
    Invoke-Native $cmake @(
        "--build", $build,
        "--target", $Target,
        "--config", "Release",
        "-j", "$Jobs"
    )

    $exeName = if ($Target.EndsWith(".exe")) { $Target } else { "$Target.exe" }
    $builtExe = Join-Path (Join-Path $CppDir $BuildDirName) $exeName
    $defaultBuildDir = Join-Path $CppDir "build"
    New-Item -ItemType Directory -Force -Path $defaultBuildDir | Out-Null
    Copy-Item -LiteralPath $builtExe -Destination (Join-Path $defaultBuildDir $exeName) -Force
    Write-Host "Built $Target and copied to cpp/build/$exeName"
}
finally {
    if ($createdSubst) {
        & cmd.exe /C "subst ${driveName}: /D" | Out-Null
    }
}
