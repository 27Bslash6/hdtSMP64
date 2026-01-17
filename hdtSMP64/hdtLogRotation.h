#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <shlobj_core.h>
#include <string>
#include <vector>

namespace hdt
{
	namespace log_rotation
	{
		constexpr size_t MAX_LOG_FILES = 10;
		constexpr const char* LOG_SUBDIR = "hdtSMP64_logs";
		constexpr const char* LOG_FILENAME = "hdtSMP64.log";

		/**
		 * Get the SKSE logs directory path.
		 * Returns empty string on failure.
		 */
		inline std::filesystem::path getLogsBasePath(bool isVR = false)
		{
			wchar_t documentsPath[MAX_PATH];
			if (FAILED(SHGetFolderPathW(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documentsPath))) {
				return {};
			}

			std::filesystem::path basePath = documentsPath;
			if (isVR) {
				basePath /= "My Games\\Skyrim VR\\SKSE";
			}
			else {
				basePath /= "My Games\\Skyrim Special Edition\\SKSE";
			}
			return basePath;
		}

		/**
		 * Generate a timestamp string for log rotation.
		 * Format: YYYYMMDD_HHMMSS
		 */
		inline std::string getTimestamp()
		{
			auto now = std::chrono::system_clock::now();
			auto time = std::chrono::system_clock::to_time_t(now);
			std::tm tm_buf;
			localtime_s(&tm_buf, &time);

			char buf[32];
			std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
			return buf;
		}

		/**
		 * Rotate the current log file to the logs subdirectory and clean up old logs.
		 * Called before opening a new log file.
		 *
		 * @param isVR Whether this is a VR build (affects path)
		 * @return true if rotation succeeded or no rotation needed, false on error
		 */
		inline bool rotateLogFile(bool isVR = false)
		{
			try {
				std::filesystem::path basePath = getLogsBasePath(isVR);
				if (basePath.empty()) {
					return false;
				}

				std::filesystem::path currentLog = basePath / LOG_FILENAME;
				std::filesystem::path logsDir = basePath / LOG_SUBDIR;

				// If current log doesn't exist, nothing to rotate
				if (!std::filesystem::exists(currentLog)) {
					return true;
				}

				// Create logs subdirectory if needed
				if (!std::filesystem::exists(logsDir)) {
					std::filesystem::create_directories(logsDir);
				}

				// Generate rotated filename with timestamp
				std::string timestamp = getTimestamp();
				std::filesystem::path rotatedLog = logsDir / ("hdtSMP64_" + timestamp + ".log");

				// Move current log to subdirectory
				std::filesystem::rename(currentLog, rotatedLog);

				// Clean up old logs, keeping only MAX_LOG_FILES most recent
				std::vector<std::filesystem::path> logFiles;
				for (const auto& entry : std::filesystem::directory_iterator(logsDir)) {
					if (entry.is_regular_file() && entry.path().extension() == ".log") {
						logFiles.push_back(entry.path());
					}
				}

				if (logFiles.size() > MAX_LOG_FILES) {
					// Sort by filename (which includes timestamp, so chronological order)
					std::sort(logFiles.begin(), logFiles.end());

					// Delete oldest files
					size_t toDelete = logFiles.size() - MAX_LOG_FILES;
					for (size_t i = 0; i < toDelete; ++i) {
						std::filesystem::remove(logFiles[i]);
					}
				}

				return true;
			}
			catch (...) {
				// Silently fail - don't prevent plugin from loading
				return false;
			}
		}

	} // namespace log_rotation
} // namespace hdt
