#pragma once

// hdtSMP64 prefix header - includes SKSE's IPrefix.h then adds timestamped async logging

#include "common/IPrefix.h"

// Now override the SKSE logging macros with timestamped versions
// IDebugLog.h was included by IPrefix.h, so gLog exists
// We use macros (not functions) so they override even after re-includes

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hdt::logging
{
	// Configured log level - mirrors gLog's private logLevel
	// Must be set when reading config (see config.cpp)
	// Default to kLevel_Message (3) - same as SKSE default
	inline std::atomic<IDebugLog::LogLevel> configuredLogLevel{IDebugLog::kLevel_Message};

	// Global frame counter - updated by SkinnedMeshWorld::incrementFrame()
	// Avoids circular dependency with hdtSkinnedMeshWorld.h
	inline std::atomic<uint32_t> currentFrameNumber{0};

	// Human-readable level names for logging
	inline const char* GetLevelPrefix(IDebugLog::LogLevel level)
	{
		switch (level) {
		case IDebugLog::kLevel_FatalError:
			return "[FATAL]";
		case IDebugLog::kLevel_Error:
			return "[ERROR]";
		case IDebugLog::kLevel_Warning:
			return "[WARN] ";
		case IDebugLog::kLevel_Message:
			return "[INFO] ";
		case IDebugLog::kLevel_VerboseMessage:
			return "[VERB] ";
		case IDebugLog::kLevel_DebugMessage:
			return "[DEBUG]";
		default:
			return "[?????]";
		}
	}

	inline const char* GetLevelName(IDebugLog::LogLevel level)
	{
		switch (level) {
		case IDebugLog::kLevel_FatalError:
			return "FatalError";
		case IDebugLog::kLevel_Error:
			return "Error";
		case IDebugLog::kLevel_Warning:
			return "Warning";
		case IDebugLog::kLevel_Message:
			return "Message";
		case IDebugLog::kLevel_VerboseMessage:
			return "VerboseMessage";
		case IDebugLog::kLevel_DebugMessage:
			return "DebugMessage";
		default:
			return "Unknown";
		}
	}

	// Async logger singleton - queues messages for background thread
	class AsyncLogger
	{
	public:
		static AsyncLogger& getInstance()
		{
			static AsyncLogger instance;
			return instance;
		}

		void enqueue(const std::string& message, bool alsoConsole)
		{
			// Start background thread on first use
			if (!m_running.load(std::memory_order_acquire)) {
				startBackgroundThread();
			}

			{
				std::lock_guard<std::mutex> lock(m_queueMutex);
				m_messageQueue.emplace_back(message, alsoConsole);
			}
			m_queueCV.notify_one();
		}

		// For fatal errors - write immediately and synchronously
		void writeImmediate(const std::string& message)
		{
			gLog.Message(message.c_str());
			printf("%s\n", message.c_str());
		}

		void shutdown()
		{
			if (m_running.exchange(false)) {
				m_queueCV.notify_one();
				if (m_backgroundThread.joinable()) {
					m_backgroundThread.join();
				}
				flushQueue();
			}
		}

	private:
		AsyncLogger() : m_running(false) {}

		~AsyncLogger() { shutdown(); }

		void startBackgroundThread()
		{
			bool expected = false;
			if (!m_running.compare_exchange_strong(expected, true))
				return; // Already started

			m_backgroundThread = std::thread([this]() { backgroundWorker(); });
		}

		void backgroundWorker()
		{
			std::vector<std::pair<std::string, bool>> localBatch;
			localBatch.reserve(64);

			while (m_running.load(std::memory_order_relaxed)) {
				{
					std::unique_lock<std::mutex> lock(m_queueMutex);
					// Wait for messages or shutdown, 100ms timeout for periodic flush
					m_queueCV.wait_for(lock, std::chrono::milliseconds(100),
									   [this]() { return !m_messageQueue.empty() || !m_running; });

					if (!m_messageQueue.empty()) {
						localBatch.swap(m_messageQueue);
					}
				}

				// Write batch outside lock
				for (const auto& [msg, alsoConsole] : localBatch) {
					gLog.Message(msg.c_str());
					if (alsoConsole) {
						printf("%s\n", msg.c_str());
					}
				}
				localBatch.clear();
			}
		}

		void flushQueue()
		{
			std::lock_guard<std::mutex> lock(m_queueMutex);
			for (const auto& [msg, alsoConsole] : m_messageQueue) {
				gLog.Message(msg.c_str());
				if (alsoConsole) {
					printf("%s\n", msg.c_str());
				}
			}
			m_messageQueue.clear();
		}

		std::mutex m_queueMutex;
		std::condition_variable m_queueCV;
		std::vector<std::pair<std::string, bool>> m_messageQueue;
		std::thread m_backgroundThread;
		std::atomic<bool> m_running;
	};

	// Fast inline check - avoids function call overhead when logging is disabled
	[[nodiscard]] inline bool shouldLog(IDebugLog::LogLevel level) noexcept
	{
		return level <= configuredLogLevel.load(std::memory_order_relaxed);
	}

	inline void LogWithTimestamp(IDebugLog::LogLevel level, const char* fmt, ...)
	{
		// Check log level BEFORE doing any work (redundant if called via macro, but safe for direct calls)
		if (level > configuredLogLevel.load(std::memory_order_relaxed))
			return;

		// Get time with microsecond precision
		auto now = std::chrono::system_clock::now();
		auto time_t_now = std::chrono::system_clock::to_time_t(now);
		auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

		struct tm tm = {};
		localtime_s(&tm, &time_t_now);

		char timeBuf[32];
		std::strftime(timeBuf, sizeof(timeBuf), "[%H:%M:%S", &tm);
		snprintf(timeBuf + 9, sizeof(timeBuf) - 9, ".%06lld]", micros.count());

		// Get frame number and thread ID
		uint32_t frame = currentFrameNumber.load(std::memory_order_relaxed);
		DWORD tid = GetCurrentThreadId();

		char msgBuf[8192];
		va_list args;
		va_start(args, fmt);
		vsnprintf(msgBuf, sizeof(msgBuf), fmt, args);
		va_end(args);

		char finalBuf[8400];
		snprintf(finalBuf, sizeof(finalBuf), "%s %s [F:%u T:%lu] %s", timeBuf, GetLevelPrefix(level), frame, tid,
				 msgBuf);

		// Fatal errors write synchronously, everything else is async
		if (level == IDebugLog::kLevel_FatalError) {
			AsyncLogger::getInstance().writeImmediate(finalBuf);
		}
		else {
			bool alsoConsole = (level <= IDebugLog::kLevel_Message);
			AsyncLogger::getInstance().enqueue(finalBuf, alsoConsole);
		}
	}
} // namespace hdt::logging

// Forcibly undef any existing macros/names, then define our versions
// This ensures our logging functions are ALWAYS used, not SKSE's inline functions
#undef _FATALERROR
#undef _ERROR
#undef _WARNING
#undef _MESSAGE
#undef _VMESSAGE
#undef _DMESSAGE

// Critical messages always log - no check needed
#define _FATALERROR(...) hdt::logging::LogWithTimestamp(IDebugLog::kLevel_FatalError, __VA_ARGS__)
#define _ERROR(...) hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Error, __VA_ARGS__)
#define _WARNING(...) hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Warning, __VA_ARGS__)
#define _MESSAGE(...) hdt::logging::LogWithTimestamp(IDebugLog::kLevel_Message, __VA_ARGS__)

// Verbose/Debug messages check BEFORE evaluating arguments - avoids overhead in hot paths
#define _VMESSAGE(...)                                                                     \
	do {                                                                                   \
		if (hdt::logging::shouldLog(IDebugLog::kLevel_VerboseMessage))                     \
			hdt::logging::LogWithTimestamp(IDebugLog::kLevel_VerboseMessage, __VA_ARGS__); \
	} while (0)

#define _DMESSAGE(...)                                                                   \
	do {                                                                                 \
		if (hdt::logging::shouldLog(IDebugLog::kLevel_DebugMessage))                     \
			hdt::logging::LogWithTimestamp(IDebugLog::kLevel_DebugMessage, __VA_ARGS__); \
	} while (0)
