#!/bin/bash
# Run tests with various sanitizers
# Usage: ./scripts/run-sanitizers.sh [asan|ubsan|tsan|all]
#
# Requirements:
#   - CMake 3.20+
#   - Ninja build system
#   - Clang compiler (clang/clang++)
#
# Exit codes:
#   0 - All tests passed
#   1 - Test failures or sanitizer errors found
#   2 - Build/setup error

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check dependencies
check_deps() {
    local missing=()

    if ! command -v cmake &> /dev/null; then
        missing+=("cmake")
    fi

    if ! command -v ninja &> /dev/null; then
        missing+=("ninja")
    fi

    if ! command -v clang++ &> /dev/null; then
        missing+=("clang++")
    fi

    if [ ${#missing[@]} -ne 0 ]; then
        log_error "Missing dependencies: ${missing[*]}"
        echo "Install with:"
        echo "  Ubuntu/Debian: sudo apt install cmake ninja-build clang"
        echo "  macOS: brew install cmake ninja llvm"
        exit 2
    fi
}

# Run a specific sanitizer build
run_sanitizer() {
    local preset=$1
    local name=$2

    log_info "Building with $name..."

    cd "$PROJECT_ROOT"

    # Configure
    if ! cmake --preset="$preset" 2>&1; then
        log_error "$name configure failed"
        return 1
    fi

    # Build
    if ! cmake --build "build/$preset" 2>&1; then
        log_error "$name build failed"
        return 1
    fi

    # Run tests
    log_info "Running tests with $name..."
    cd "build/$preset"

    # Set sanitizer options
    case "$preset" in
        asan|asan-ubsan|ci-asan)
            export ASAN_OPTIONS="detect_leaks=1:halt_on_error=1:print_stats=1:check_initialization_order=1"
            ;;
        ubsan)
            export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
            ;;
        tsan|ci-tsan)
            export TSAN_OPTIONS="halt_on_error=1:second_deadlock_stack=1"
            ;;
    esac

    if ./hdtSMP64_tests; then
        log_info "$name: All tests PASSED"
        return 0
    else
        log_error "$name: Tests FAILED or sanitizer found issues"
        return 1
    fi
}

# Main
main() {
    local target="${1:-all}"
    local failed=0

    check_deps

    case "$target" in
        asan)
            run_sanitizer "asan" "AddressSanitizer" || failed=1
            ;;
        ubsan)
            run_sanitizer "ubsan" "UndefinedBehaviorSanitizer" || failed=1
            ;;
        tsan)
            run_sanitizer "tsan" "ThreadSanitizer" || failed=1
            ;;
        asan-ubsan)
            run_sanitizer "asan-ubsan" "ASan+UBSan" || failed=1
            ;;
        all)
            log_info "Running all sanitizers..."
            echo ""

            run_sanitizer "asan" "AddressSanitizer" || failed=1
            echo ""

            run_sanitizer "ubsan" "UndefinedBehaviorSanitizer" || failed=1
            echo ""

            # TSan is incompatible with ASan, run separately
            run_sanitizer "tsan" "ThreadSanitizer" || failed=1
            echo ""
            ;;
        *)
            echo "Usage: $0 [asan|ubsan|tsan|asan-ubsan|all]"
            echo ""
            echo "Sanitizers:"
            echo "  asan       - AddressSanitizer (memory errors)"
            echo "  ubsan      - UndefinedBehaviorSanitizer"
            echo "  tsan       - ThreadSanitizer (data races)"
            echo "  asan-ubsan - Both ASan and UBSan"
            echo "  all        - Run all sanitizers (default)"
            exit 2
            ;;
    esac

    echo ""
    if [ $failed -eq 0 ]; then
        log_info "All sanitizer checks PASSED"
        exit 0
    else
        log_error "Some sanitizer checks FAILED"
        exit 1
    fi
}

main "$@"
