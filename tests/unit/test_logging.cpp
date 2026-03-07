#include "catch.hpp"

// Simulate the SKSE log levels
namespace IDebugLog
{
	enum LogLevel
	{
		kLevel_FatalError = 0,
		kLevel_Error = 1,
		kLevel_Warning = 2,
		kLevel_Message = 3,
		kLevel_VerboseMessage = 4,
		kLevel_DebugMessage = 5
	};
} // namespace IDebugLog

// Test the log level filtering logic
TEST_CASE("Log level filtering logic", "[logging]")
{
	SECTION("Level check: level > configured filters correctly")
	{
		int configuredLogLevel = 3; // kLevel_Message

		// Should pass (not filtered) - level <= configured
		REQUIRE_FALSE(IDebugLog::kLevel_FatalError > configuredLogLevel); // 0 > 3 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Error > configuredLogLevel);	  // 1 > 3 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Warning > configuredLogLevel);	  // 2 > 3 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Message > configuredLogLevel);	  // 3 > 3 = false, SHOULD LOG

		// Should be filtered - level > configured
		REQUIRE(IDebugLog::kLevel_VerboseMessage > configuredLogLevel); // 4 > 3 = true
		REQUIRE(IDebugLog::kLevel_DebugMessage > configuredLogLevel);	// 5 > 3 = true
	}

	SECTION("With logLevel=0, only FatalError logs")
	{
		int configuredLogLevel = 0;

		REQUIRE_FALSE(IDebugLog::kLevel_FatalError > configuredLogLevel); // 0 > 0 = false, logs
		REQUIRE(IDebugLog::kLevel_Error > configuredLogLevel);			  // 1 > 0 = true, filtered
		REQUIRE(IDebugLog::kLevel_Message > configuredLogLevel);		  // 3 > 0 = true, filtered
	}

	SECTION("With logLevel=5, everything logs")
	{
		int configuredLogLevel = 5;

		REQUIRE_FALSE(IDebugLog::kLevel_FatalError > configuredLogLevel);	  // 0 > 5 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Error > configuredLogLevel);		  // 1 > 5 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Warning > configuredLogLevel);		  // 2 > 5 = false
		REQUIRE_FALSE(IDebugLog::kLevel_Message > configuredLogLevel);		  // 3 > 5 = false
		REQUIRE_FALSE(IDebugLog::kLevel_VerboseMessage > configuredLogLevel); // 4 > 5 = false
		REQUIRE_FALSE(IDebugLog::kLevel_DebugMessage > configuredLogLevel);	  // 5 > 5 = false
	}
}

// This test documents the macro resolution problem
TEST_CASE("Macro resolution explanation", "[logging][!mayfail]")
{
	INFO("The _MESSAGE macro issue:");
	INFO("1. hdtPrefix.h is force-included FIRST");
	INFO("2. hdtPrefix.h includes common/IPrefix.h");
	INFO("3. common/IPrefix.h includes common/IDebugLog.h");
	INFO("4. IDebugLog.h defines: inline void _MESSAGE(const char* fmt, ...)");
	INFO("5. THEN hdtPrefix.h defines: #define _MESSAGE(...) hdt::logging::LogWithTimestamp(...)");
	INFO("6. The MACRO should shadow the inline function");
	INFO("");
	INFO("BUT: If _MESSAGE is called before the macro definition in the same");
	INFO("     translation unit, or if some header undefines it, the inline");
	INFO("     function is used instead.");
	INFO("");
	INFO("SKSE's inline _MESSAGE calls gLog.Log() which has its own level check.");
	INFO("Our macro calls hdt::logging::LogWithTimestamp() which has a different check.");

	// This always passes - it's just documentation
	REQUIRE(true);
}
