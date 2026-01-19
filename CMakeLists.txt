# CMakeLists.txt for hdtSMP64
# Primary purpose: Sanitizer builds for testing
# Production builds still use Visual Studio .sln
#
# Usage:
#   cmake -B build -G Ninja --preset=asan
#   cmake --build build
#   ./build/hdtSMP64_tests

cmake_minimum_required(VERSION 3.20)
project(hdtSMP64_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Options
# =============================================================================
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ENABLE_MSAN "Enable MemorySanitizer" OFF)

# =============================================================================
# Compiler detection and flags
# =============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(COMPILER_IS_CLANG_OR_GCC TRUE)
elseif(MSVC)
    set(COMPILER_IS_MSVC TRUE)
endif()

# Sanitizer flags
set(SANITIZER_FLAGS "")
set(SANITIZER_LINK_FLAGS "")

if(ENABLE_ASAN)
    if(COMPILER_IS_CLANG_OR_GCC)
        list(APPEND SANITIZER_FLAGS -fsanitize=address -fno-omit-frame-pointer)
        list(APPEND SANITIZER_LINK_FLAGS -fsanitize=address)
    elseif(COMPILER_IS_MSVC)
        list(APPEND SANITIZER_FLAGS /fsanitize=address)
    endif()
    message(STATUS "AddressSanitizer ENABLED")
endif()

if(ENABLE_UBSAN)
    if(COMPILER_IS_CLANG_OR_GCC)
        list(APPEND SANITIZER_FLAGS -fsanitize=undefined -fno-omit-frame-pointer)
        list(APPEND SANITIZER_LINK_FLAGS -fsanitize=undefined)
        message(STATUS "UndefinedBehaviorSanitizer ENABLED")
    else()
        message(WARNING "UBSan not supported on this compiler")
    endif()
endif()

if(ENABLE_TSAN)
    if(COMPILER_IS_CLANG_OR_GCC)
        # TSan is incompatible with ASan
        if(ENABLE_ASAN)
            message(FATAL_ERROR "ThreadSanitizer cannot be used with AddressSanitizer")
        endif()
        list(APPEND SANITIZER_FLAGS -fsanitize=thread -fno-omit-frame-pointer)
        list(APPEND SANITIZER_LINK_FLAGS -fsanitize=thread)
        message(STATUS "ThreadSanitizer ENABLED")
    else()
        message(WARNING "TSan not supported on this compiler/platform")
    endif()
endif()

if(ENABLE_MSAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT WIN32)
        # MSan is incompatible with ASan and TSan
        if(ENABLE_ASAN OR ENABLE_TSAN)
            message(FATAL_ERROR "MemorySanitizer cannot be used with ASan or TSan")
        endif()
        list(APPEND SANITIZER_FLAGS -fsanitize=memory -fno-omit-frame-pointer -fsanitize-memory-track-origins)
        list(APPEND SANITIZER_LINK_FLAGS -fsanitize=memory)
        message(STATUS "MemorySanitizer ENABLED")
    else()
        message(WARNING "MSan only supported on Clang/Linux")
    endif()
endif()

# =============================================================================
# External dependencies
# =============================================================================

# Highway SIMD library (header-only with some compiled pieces)
add_subdirectory(external/highway EXCLUDE_FROM_ALL)

# enkiTS task scheduler (required for integration tests)
set(ENKITS_SOURCES
    external/enkiTS/src/TaskScheduler.cpp
)

# =============================================================================
# Test executable
# =============================================================================
set(TEST_SOURCES
    tests/test_main.cpp
    tests/test_stubs.cpp
    # Core algorithm tests
    tests/unit/test_collision.cpp
    tests/unit/test_config.cpp
    tests/unit/test_dispatcher.cpp
    tests/unit/test_non_hookean.cpp
    tests/unit/test_serialization.cpp
    tests/unit/test_aabb_simd.cpp
    # Concurrency tests (critical for TSan)
    tests/unit/test_thread_local_stale_data.cpp
    tests/unit/test_suspend_sync.cpp
    tests/unit/test_scheduler_init.cpp
    tests/unit/test_shutdown_atomic.cpp
    # Integration tests (use real production code)
    tests/integration/test_dispatcher_integration.cpp
    # Additional unit tests
    tests/unit/test_collision_index.cpp
    tests/unit/test_cuda_double_buffer.cpp
    tests/unit/test_frame_counter.cpp
    tests/unit/test_logging.cpp
    tests/unit/test_security.cpp
    tests/unit/test_solver_edge_cases.cpp
    # Highway SIMD tests
    tests/unit/test_highway_bridge_integration.cpp
    tests/unit/test_highway_debug.cpp
    tests/unit/test_highway_skinning.cpp
    tests/unit/test_highway_solver_concurrent.cpp
    # Benchmarks (excluded from regular runs, use [.benchmark] tag)
    tests/unit/test_bullet_solver_configs.cpp
    tests/unit/test_solver_benchmark.cpp
    tests/unit/test_solver_performance.cpp
)

add_executable(hdtSMP64_tests ${TEST_SOURCES} ${ENKITS_SOURCES})

target_include_directories(hdtSMP64_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/include
    ${CMAKE_SOURCE_DIR}/hdtSMP64
    ${CMAKE_SOURCE_DIR}/hdtSMP64/hdtSkinnedMesh
    ${CMAKE_SOURCE_DIR}/hdtSSEUtils
    ${CMAKE_SOURCE_DIR}/external/enkiTS/src
    ${CMAKE_SOURCE_DIR}/external/highway
)

# Threading support (required for enkiTS)
find_package(Threads REQUIRED)
target_link_libraries(hdtSMP64_tests PRIVATE
    Threads::Threads
    hwy
    hwy_contrib
)

# Apply sanitizer flags
if(SANITIZER_FLAGS)
    target_compile_options(hdtSMP64_tests PRIVATE ${SANITIZER_FLAGS})
    target_link_options(hdtSMP64_tests PRIVATE ${SANITIZER_LINK_FLAGS})
endif()

# Platform-specific settings
if(COMPILER_IS_CLANG_OR_GCC)
    target_compile_options(hdtSMP64_tests PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
        -g  # Debug symbols for sanitizer reports
        -mavx2  # Enable AVX2 for SIMD tests (test_aabb_simd.cpp)
        -mfma   # Enable FMA for AVX2
    )
elseif(COMPILER_IS_MSVC)
    target_compile_options(hdtSMP64_tests PRIVATE
        /W4
        /wd4100  # unreferenced formal parameter
        /Zi      # Debug symbols
    )
endif()

# =============================================================================
# CTest integration
# =============================================================================
enable_testing()
add_test(NAME unit_tests COMMAND hdtSMP64_tests)

# Set sanitizer environment variables for tests
if(ENABLE_ASAN)
    set_tests_properties(unit_tests PROPERTIES
        ENVIRONMENT "ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:print_stats=1"
    )
endif()

if(ENABLE_UBSAN)
    set_tests_properties(unit_tests PROPERTIES
        ENVIRONMENT "UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1"
    )
endif()

if(ENABLE_TSAN)
    # Use suppression file if it exists
    if(EXISTS "${CMAKE_SOURCE_DIR}/tsan.supp")
        set_tests_properties(unit_tests PROPERTIES
            ENVIRONMENT "TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1:suppressions=${CMAKE_SOURCE_DIR}/tsan.supp"
        )
    else()
        set_tests_properties(unit_tests PROPERTIES
            ENVIRONMENT "TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1"
        )
    endif()
endif()
