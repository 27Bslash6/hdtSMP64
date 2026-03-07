#!/bin/bash
# clang-tidy check script for hdtSMP64
# Usage: ./scripts/tidy-check.sh [--fix]
# Exit codes: 0 = clean, 1 = issues found, 2 = setup error

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

FIX=""
if [[ "$1" == "--fix" ]]; then
    FIX="--fix"
fi

# Find clang-tidy
CLANG_TIDY=$(command -v clang-tidy 2>/dev/null || true)
if [[ -z "$CLANG_TIDY" ]]; then
    echo "ERROR: clang-tidy not found. Install LLVM or add to PATH."
    echo "  Ubuntu/Debian: sudo apt install clang-tidy"
    echo "  macOS: brew install llvm && export PATH=\"/usr/local/opt/llvm/bin:\$PATH\""
    exit 2
fi

echo "Using clang-tidy: $CLANG_TIDY"

# Source files (project code only)
FILES=$(find "$PROJECT_ROOT/hdtSMP64" "$PROJECT_ROOT/hdtSSEUtils" \
    -type f -name "*.cpp" \
    ! -path "*Bullet*" \
    ! -path "*LinearMath*" \
    ! -path "*external*" \
    2>/dev/null || true)

if [[ -z "$FILES" ]]; then
    echo "No source files found to check"
    exit 0
fi

FILE_COUNT=$(echo "$FILES" | wc -l)
echo "Checking $FILE_COUNT files..."

ERROR_COUNT=0
WARNING_COUNT=0

while IFS= read -r file; do
    relative_path="${file#$PROJECT_ROOT/}"

    output=$($CLANG_TIDY "$file" \
        --config-file="$PROJECT_ROOT/.clang-tidy" \
        --header-filter="(hdtSMP64|hdtSSEUtils)/.*\.h$" \
        --quiet \
        --extra-arg=-std=c++17 \
        --extra-arg=-DWIN32 \
        --extra-arg=-D_WINDOWS \
        --extra-arg=-DNDEBUG \
        --extra-arg="-I$PROJECT_ROOT/hdtSMP64" \
        --extra-arg="-I$PROJECT_ROOT/hdtSSEUtils" \
        --extra-arg=-Wno-unknown-pragmas \
        $FIX 2>&1 || true)

    if [[ -n "$output" ]]; then
        while IFS= read -r line; do
            if [[ "$line" == *"error:"* ]]; then
                echo "$line"
                ((ERROR_COUNT++))
            elif [[ "$line" == *"warning:"* ]]; then
                echo "$line"
                ((WARNING_COUNT++))
            fi
        done <<< "$output"
    fi
done <<< "$FILES"

echo ""
echo "Summary: $ERROR_COUNT errors, $WARNING_COUNT warnings"

if [[ $ERROR_COUNT -gt 0 ]]; then
    echo "clang-tidy: Issues found"
    [[ -z "$FIX" ]] && echo "Run with --fix to apply automatic fixes"
    exit 1
else
    echo "clang-tidy: All checks passed"
    exit 0
fi
