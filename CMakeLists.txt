# CMakeLists.txt for hdtSMP64 - Linux TSan builds only
# Minimal build for ThreadSanitizer race detection
# Tests pure C++ std::atomic correctness without Windows/SKSE dependencies
#
# For full testing: Use Windows ASan workflow (sanitizers-windows.yml)
#
# Usage:
#   cmake --preset=ci-tsan
#   cmake --build build/ci-tsan
#   ./build/ci-tsan/hdtSMP64_tests

cmake_minimum_required(VERSION 3.20)
project(hdtSMP64_minimal_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Options
# =============================================================================
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

# =============================================================================
# Sanitizer flags
# =============================================================================
set(SANITIZER_FLAGS "")
set(SANITIZER_LINK_FLAGS "")

if(ENABLE_TSAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        list(APPEND SANITIZER_FLAGS -fsanitize=thread -fno-omit-frame-pointer)
        list(APPEND SANITIZER_LINK_FLAGS -fsanitize=thread)
        message(STATUS "ThreadSanitizer ENABLED")
    else()
        message(FATAL_ERROR "TSan requires Clang or GCC")
    endif()
endif()

# =============================================================================
# Minimal test executable - pure threading tests only
# =============================================================================
set(TEST_SOURCES
    tests/test_main.cpp
    # Pure std::atomic tests - no external dependencies
    tests/unit/test_scheduler_init.cpp
    tests/unit/test_shutdown_atomic.cpp
)

add_executable(hdtSMP64_tests ${TEST_SOURCES})

target_include_directories(hdtSMP64_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/include
)

# Threading support
find_package(Threads REQUIRED)
target_link_libraries(hdtSMP64_tests PRIVATE Threads::Threads)

# Apply sanitizer flags
if(SANITIZER_FLAGS)
    target_compile_options(hdtSMP64_tests PRIVATE ${SANITIZER_FLAGS})
    target_link_options(hdtSMP64_tests PRIVATE ${SANITIZER_LINK_FLAGS})
endif()

# Compiler flags
target_compile_options(hdtSMP64_tests PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -g  # Debug symbols for sanitizer reports
)

# =============================================================================
# CTest integration
# =============================================================================
enable_testing()
add_test(NAME tsan_tests COMMAND hdtSMP64_tests)

if(ENABLE_TSAN)
    # Use suppression file if it exists
    if(EXISTS "${CMAKE_SOURCE_DIR}/tsan.supp")
        set_tests_properties(tsan_tests PROPERTIES
            ENVIRONMENT "TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1:suppressions=${CMAKE_SOURCE_DIR}/tsan.supp"
        )
    else()
        set_tests_properties(tsan_tests PROPERTIES
            ENVIRONMENT "TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1"
        )
    endif()
endif()
