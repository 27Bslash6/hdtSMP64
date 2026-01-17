// Wrapper for enkiTS TaskScheduler.cpp
// This file exists to ensure min/max macros don't conflict with std::min/std::max

#ifndef NOMINMAX
#define NOMINMAX
#endif

// Undef Windows min/max macros if they were already defined
// This handles the case where Windows.h was included via precompiled headers
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

// Include the actual enkiTS implementation
#include "../../external/enkiTS/src/TaskScheduler.cpp"
