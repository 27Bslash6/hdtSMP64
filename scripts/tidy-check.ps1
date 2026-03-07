# clang-tidy check script for hdtSMP64
# Usage: .\scripts\tidy-check.ps1 [--fix]
# Exit codes: 0 = clean, 1 = issues found, 2 = setup error

param(
    [switch]$Fix
)

$projectRoot = Split-Path -Parent $PSScriptRoot

# Find clang-tidy
$clangTidy = Get-Command clang-tidy -ErrorAction SilentlyContinue
if (-not $clangTidy) {
    $llvmPaths = @(
        "C:\Program Files\LLVM\bin\clang-tidy.exe",
        "C:\Program Files (x86)\LLVM\bin\clang-tidy.exe",
        "$env:LOCALAPPDATA\Programs\LLVM\bin\clang-tidy.exe"
    )
    foreach ($path in $llvmPaths) {
        if (Test-Path $path) {
            $clangTidy = $path
            break
        }
    }
}

if (-not $clangTidy) {
    Write-Host "clang-tidy not found. Install: winget install LLVM.LLVM" -ForegroundColor Red
    exit 2
}

# Source files to check (project code only)
$sourcePatterns = @(
    "$projectRoot\hdtSMP64\*.cpp",
    "$projectRoot\hdtSMP64\hdtSkinnedMesh\*.cpp",
    "$projectRoot\hdtSSEUtils\*.cpp"
)

# Exclude patterns
$excludePatterns = @(
    "*Bullet*",
    "*LinearMath*",
    "*external*"
)

# Gather files
$files = @()
foreach ($pattern in $sourcePatterns) {
    $matches = Get-ChildItem -Path $pattern -ErrorAction SilentlyContinue
    foreach ($file in $matches) {
        $skip = $false
        foreach ($exclude in $excludePatterns) {
            if ($file.FullName -like $exclude) {
                $skip = $true
                break
            }
        }
        if (-not $skip) {
            $files += $file.FullName
        }
    }
}

if ($files.Count -eq 0) {
    exit 0
}

# Build common clang-tidy arguments
# Note: Without compile_commands.json, we need to provide basic flags
$commonArgs = @(
    "--config-file=$projectRoot\.clang-tidy",
    "--header-filter=(hdtSMP64|hdtSSEUtils)/.*\.h$",
    "--quiet"
)

# Add MSVC-compatible flags (since we don't have compile_commands.json)
$extraArgs = @(
    "--extra-arg=-std=c++17",
    "--extra-arg=-DWIN32",
    "--extra-arg=-D_WINDOWS",
    "--extra-arg=-DNDEBUG",
    "--extra-arg=-I$projectRoot\hdtSMP64",
    "--extra-arg=-I$projectRoot\hdtSSEUtils",
    "--extra-arg=-I$projectRoot\external\skse\skse64",
    "--extra-arg=-I$projectRoot\external\skse\common",
    "--extra-arg=-I$projectRoot\external\detours\include",
    "--extra-arg=-Wno-unknown-pragmas"
)

if ($Fix) {
    $commonArgs += "--fix"
}

$hasErrors = $false
$errorCount = 0
$warningCount = 0

foreach ($file in $files) {
    $output = & $clangTidy $file $commonArgs $extraArgs 2>&1

    if ($output) {
        foreach ($line in $output) {
            if ($line -match "error:") {
                Write-Host $line -ForegroundColor Red
                $errorCount++
                $hasErrors = $true
            } elseif ($line -match "warning:") {
                Write-Host $line -ForegroundColor Yellow
                $warningCount++
            }
        }
    }
}

if ($hasErrors) {
    Write-Host "clang-tidy: $errorCount errors, $warningCount warnings" -ForegroundColor Red
    exit 1
}
exit 0
