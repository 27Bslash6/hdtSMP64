#pragma once

#include <cctype>
#include <string>

namespace hdt
{
	namespace security
	{
		/**
		 * Validates save file names to prevent path traversal attacks (CWE-22).
		 *
		 * @param name The save file name to validate
		 * @return true if the name is safe to use in file paths, false otherwise
		 *
		 * Rules:
		 * - Must not be empty
		 * - Must not exceed 255 characters
		 * - Must contain only: alphanumeric, underscore, hyphen, space, period
		 * - Must not contain path traversal patterns (..)
		 * - Must not contain path separators (/, \)
		 * - Must not start with a period (hidden files)
		 * - Only ASCII characters allowed (0x20-0x7E)
		 */
		inline bool isValidSaveName(const std::string& name)
		{
			// Empty check
			if (name.empty()) {
				return false;
			}

			// Length check (255 is typical filesystem max for filename)
			if (name.length() > 255) {
				return false;
			}

			// Cannot start with period (hidden files, also catches "." and "..")
			if (name[0] == '.') {
				return false;
			}

			// Check for path traversal pattern (..) anywhere in the string
			if (name.find("..") != std::string::npos) {
				return false;
			}

			// Validate each character
			for (size_t i = 0; i < name.length(); ++i) {
				unsigned char c = static_cast<unsigned char>(name[i]);

				// Only allow printable ASCII (0x20-0x7E)
				if (c < 0x20 || c > 0x7E) {
					return false;
				}

				// Allow: alphanumeric, underscore, hyphen, space, period
				bool isAlnum = std::isalnum(c) != 0;
				bool isAllowedSpecial = (c == '_' || c == '-' || c == ' ' || c == '.');

				if (!isAlnum && !isAllowedSpecial) {
					// Reject path separators and other special chars
					// This includes: / \ : * ? " < > |
					return false;
				}
			}

			return true;
		}
	} // namespace security
} // namespace hdt
