# Format check script - verifies code follows .clang-format style
# Usage: .\scripts\format-check.ps1 [--fix]
# Exit codes: 0 = formatted correctly, 1 = formatting issues found

param(
    [switch]$Fix
)

$projectRoot = Split-Path -Parent $PSScriptRoot

# Files to check (project code only, not external dependencies)
$sourceFiles = @(
    "$projectRoot\hdtSMP64\*.cpp",
    "$projectRoot\hdtSMP64\*.h",
    "$projectRoot\hdtSMP64\hdtSkinnedMesh\*.cpp",
    "$projectRoot\hdtSMP64\hdtSkinnedMesh\*.h",
    "$projectRoot\hdtSSEUtils\*.cpp",
    "$projectRoot\hdtSSEUtils\*.h"
)

# Exclude patterns (Bullet library, UTF-16 encoded files)
$excludePatterns = @(
    "*Bullet*",
    "*LinearMath*",
    "*external*",
    "*stdafx*",
    "*targetver*",
    "*hdtConstraintGroup*"
)

# Find clang-format
$clangFormat = Get-Command clang-format -ErrorAction SilentlyContinue
if (-not $clangFormat) {
    # Try common LLVM installation paths
    $llvmPaths = @(
        "C:\Program Files\LLVM\bin\clang-format.exe",
        "C:\Program Files (x86)\LLVM\bin\clang-format.exe",
        "$env:LOCALAPPDATA\Programs\LLVM\bin\clang-format.exe"
    )
    foreach ($path in $llvmPaths) {
        if (Test-Path $path) {
            $clangFormat = $path
            break
        }
    }
}

if (-not $clangFormat) {
    Write-Host "clang-format not found. Install: winget install LLVM.LLVM" -ForegroundColor Red
    exit 2
}

# Gather files
$files = @()
foreach ($pattern in $sourceFiles) {
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

$hasErrors = $false
$failedFiles = @()

foreach ($file in $files) {
    $relativePath = $file.Replace($projectRoot + "\", "")

    if ($Fix) {
        & $clangFormat -i $file
    } else {
        $null = & $clangFormat --dry-run --Werror $file 2>&1
        if ($LASTEXITCODE -ne 0) {
            $failedFiles += $relativePath
            $hasErrors = $true
        }
    }
}

if ($hasErrors) {
    Write-Host "Format errors:" -ForegroundColor Red
    $failedFiles | ForEach-Object { Write-Host "  $_" }
    Write-Host "Run 'just fmt' to fix" -ForegroundColor Yellow
    exit 1
}
exit 0
