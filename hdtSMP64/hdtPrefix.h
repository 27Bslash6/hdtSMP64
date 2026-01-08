#pragma once

// hdtSMP64 prefix header - includes SKSE's IPrefix.h then adds timestamps to logging

#include "common/IPrefix.h"

// Now override the SKSE logging macros with timestamped versions
// IDebugLog.h was included by IPrefix.h, so gLog exists
// We use macros (not functions) so they override even after re-includes

#include <ctime>
#include <chrono>

namespace hdt::logging
{
    inline void LogWithTimestamp(IDebugLog::LogLevel level, const char* fmt, ...)
    {
        // Get time with microsecond precision
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()) % 1000000;

        struct tm tm;
        localtime_s(&tm, &time_t_now);

        char timeBuf[32];
        std::strftime(timeBuf, sizeof(timeBuf), "[%H:%M:%S", &tm);
        snprintf(timeBuf + 9, sizeof(timeBuf) - 9, ".%06lld] ", micros.count());

        char msgBuf[8192];
        va_list args;
        va_start(args, fmt);
        vsnprintf(msgBuf, sizeof(msgBuf), fmt, args);
        va_end(args);

        char finalBuf[8300];
        snprintf(finalBuf, sizeof(finalBuf), "%s%s", timeBuf, msgBuf);

        gLog.Message(finalBuf);

        if (level <= IDebugLog::kLevel_Message)
            printf("%s\n", finalBuf);
    }
}

// Use macros to completely shadow the SKSE inline functions
// Macros take precedence over functions with same name
#define _FATALERROR(...) hdt::logging::LogWithTimestamp(IDebugLog::kLevel_FatalError, __VA_ARGS__)
#define _ERROR(...)      hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Error, __VA_ARGS__)
#define _WARNING(...)    hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Warning, __VA_ARGS__)
#define _MESSAGE(...)    hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Message, __VA_ARGS__)
#define _VMESSAGE(...)   hdt::logging::LogWithTimestamp(IDebugLog::kLevel_VerboseMessage, __VA_ARGS__)
#define _DMESSAGE(...)   hdt::logging::LogWithTimestamp(IDebugLog::kLevel_DebugMessage, __VA_ARGS__)
