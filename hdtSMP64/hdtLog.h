#pragma once

// Unified timestamped logging for hdtSMP64
// Uses SKSE's gLog (writes to hdtSMP64.log) with microsecond timestamps

#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdarg>
#include <mutex>

// Forward declare gLog from SKSE
extern IDebugLog gLog;

namespace hdt
{
	enum class LogLevel
	{
		Debug = 0,
		Info = 1,
		Warning = 2,
		Error = 3
	};

	// Shared timestamp formatting with microsecond precision
	inline void formatTimestamp(char* buf, size_t bufSize)
	{
		auto now = std::chrono::system_clock::now();
		auto time_t_now = std::chrono::system_clock::to_time_t(now);
		auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
			now.time_since_epoch()) % 1000000;

		struct tm tm;
		localtime_s(&tm, &time_t_now);
		std::strftime(buf, bufSize, "[%H:%M:%S", &tm);
		snprintf(buf + 9, bufSize - 9, ".%06lld]", micros.count());
	}

	class Logger
	{
	public:
		static Logger& getInstance()
		{
			static Logger instance;
			return instance;
		}

		void init(const char* /*filename*/) { m_initialized = true; }
		void setLevel(LogLevel level) { m_level = level; }
		void setEnabled(bool enabled) { m_enabled = enabled; }

		void log(LogLevel level, const char* format, ...)
		{
			if (!m_enabled || level < m_level)
				return;

			std::lock_guard<std::mutex> lock(m_mutex);

			try
			{
				char timeBuf[32];
				formatTimestamp(timeBuf, sizeof(timeBuf));

				const char* levelStr = "";
				switch (level)
				{
				case LogLevel::Debug:   levelStr = "[DEBUG]"; break;
				case LogLevel::Info:    levelStr = "[INFO] "; break;
				case LogLevel::Warning: levelStr = "[WARN] "; break;
				case LogLevel::Error:   levelStr = "[ERROR]"; break;
				}

				char msgBuf[2048];
				va_list args;
				va_start(args, format);
				vsnprintf(msgBuf, sizeof(msgBuf), format, args);
				va_end(args);

				char finalBuf[2200];
				snprintf(finalBuf, sizeof(finalBuf), "%s %s %s", timeBuf, levelStr, msgBuf);
				gLog.Message(finalBuf);
			}
			catch (...)
			{
				// Silently fail - don't crash the game over logging
			}
		}

		void close() { /* gLog handles its own cleanup */ }

	private:
		Logger() : m_enabled(true), m_level(LogLevel::Info), m_initialized(false) {}
		~Logger() = default;

		std::mutex m_mutex;
		bool m_enabled;
		bool m_initialized;
		LogLevel m_level;
	};

	// Primary logging macros - use these throughout hdtSMP64
	#define HDT_LOG_DEBUG(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Debug, fmt, ##__VA_ARGS__)
	#define HDT_LOG_INFO(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Info, fmt, ##__VA_ARGS__)
	#define HDT_LOG_WARN(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Warning, fmt, ##__VA_ARGS__)
	#define HDT_LOG_ERROR(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Error, fmt, ##__VA_ARGS__)
}
