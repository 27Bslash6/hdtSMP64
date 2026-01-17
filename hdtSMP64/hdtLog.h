#pragma once

// Unified logging for hdtSMP64
// All logging goes through hdtPrefix.h macros which:
// - Respect configured log level from configs.xml
// - Add timestamps with microsecond precision
// - Add level prefixes ([INFO], [WARN], etc.)
//
// The HDT_LOG_* macros below are aliases for the _MESSAGE/_WARNING/etc. macros.
// This ensures a single source of truth for log level and consistent formatting.

#include "hdtPrefix.h"

namespace hdt
{
	// LogLevel enum preserved for backwards compatibility
	// Maps to IDebugLog::LogLevel values
	enum class LogLevel
	{
		Debug = 5,	 // IDebugLog::kLevel_DebugMessage
		Info = 3,	 // IDebugLog::kLevel_Message
		Warning = 2, // IDebugLog::kLevel_Warning
		Error = 1	 // IDebugLog::kLevel_Error
	};

	// Deprecated: Logger class no longer used
	// All logging now goes through hdtPrefix.h's unified system
	// Kept for any code that might reference Logger::getInstance()
	class Logger
	{
	public:
		static Logger& getInstance()
		{
			static Logger instance;
			return instance;
		}

		// No-op: log level is now controlled via configs.xml
		void setLevel(LogLevel /*level*/) {}
		void setEnabled(bool /*enabled*/) {}

		// No-op: initialization handled by SKSE
		void init(const char* /*filename*/) {}
		void close() {}

		// Redirect to unified logging
		void log(LogLevel level, const char* format, ...)
		{
			va_list args;
			va_start(args, format);

			char msgBuf[2048];
			vsnprintf(msgBuf, sizeof(msgBuf), format, args);
			va_end(args);

			switch (level) {
			case LogLevel::Debug:
				_DMESSAGE("%s", msgBuf);
				break;
			case LogLevel::Info:
				_MESSAGE("%s", msgBuf);
				break;
			case LogLevel::Warning:
				_WARNING("%s", msgBuf);
				break;
			case LogLevel::Error:
				_ERROR("%s", msgBuf);
				break;
			}
		}

	private:
		Logger() = default;
	};
} // namespace hdt

// All logging uses the unified _MESSAGE family from hdtPrefix.h
// No separate HDT_LOG_* macros - use _DMESSAGE, _MESSAGE, _WARNING, _ERROR directly
