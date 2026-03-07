#!/bin/bash
# Format check script - verifies code follows .clang-format style
# Usage: ./scripts/format-check.sh [--fix]
# Exit codes: 0 = formatted correctly, 1 = formatting issues found

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

FIX=false
if [[ "$1" == "--fix" ]]; then
    FIX=true
fi

# Find clang-format
CLANG_FORMAT=$(command -v clang-format 2>/dev/null || true)
if [[ -z "$CLANG_FORMAT" ]]; then
    echo "ERROR: clang-format not found. Install LLVM or add to PATH."
    echo "  Ubuntu/Debian: sudo apt install clang-format"
    echo "  macOS: brew install clang-format"
    exit 2
fi

echo "Using clang-format: $CLANG_FORMAT"

# Source file patterns (project code only)
FILES=$(find "$PROJECT_ROOT/hdtSMP64" "$PROJECT_ROOT/hdtSSEUtils" \
    -type f \( -name "*.cpp" -o -name "*.h" \) \
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

HAS_ERRORS=false

while IFS= read -r file; do
    relative_path="${file#$PROJECT_ROOT/}"

    if [[ "$FIX" == true ]]; then
        $CLANG_FORMAT -i "$file"
        echo "  Formatted: $relative_path"
    else
        if ! $CLANG_FORMAT --dry-run --Werror "$file" 2>/dev/null; then
            echo "  FAIL: $relative_path"
            HAS_ERRORS=true
        fi
    fi
done <<< "$FILES"

if [[ "$FIX" == true ]]; then
    echo "Formatting complete!"
    exit 0
elif [[ "$HAS_ERRORS" == true ]]; then
    echo ""
    echo "Formatting issues found. Run with --fix to auto-format:"
    echo "  ./scripts/format-check.sh --fix"
    echo "  or: just fmt"
    exit 1
else
    echo "All files formatted correctly!"
    exit 0
fi
