#include "config.h"

#include "hdtPrefix.h" // For hdt::logging::configuredLogLevel
#include "hdtSkyrimPhysicsWorld.h"

#include "XmlReader.h"
#ifdef CUDA
#include "hdtSkinnedMesh/hdtCudaInterface.h"
#endif

#include <algorithm>
#include <clocale>

namespace hdt
{
	// Case-insensitive tag comparison for XML config parsing
	static bool tagEquals(const std::string& tag, const char* expected)
	{
		if (tag.length() != strlen(expected))
			return false;
		return std::equal(tag.begin(), tag.end(), expected, [](char a, char b) {
			return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
		});
	}
	static void solver(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag: {
				auto tag = reader.GetLocalName();
				if (tagEquals(tag, "numIterations"))
					SkyrimPhysicsWorld::get()->getSolverInfo().m_numIterations = btClamped(reader.readInt(), 4, 128);
				else if (tagEquals(tag, "groupIterations"))
					ConstraintGroup::MaxIterations = btClamped(reader.readInt(), 0, 4096);
				else if (tagEquals(tag, "groupEnableMLCP"))
					ConstraintGroup::EnableMLCP = reader.readBool();
				else if (tagEquals(tag, "erp"))
					SkyrimPhysicsWorld::get()->getSolverInfo().m_erp = btClamped(reader.readFloat(), 0.01f, 1.0f);
				else if (tagEquals(tag, "min-fps")) {
					SkyrimPhysicsWorld::get()->min_fps = (btClamped(reader.readInt(), 1, 300));
					SkyrimPhysicsWorld::get()->m_timeTick = 1.0f / SkyrimPhysicsWorld::get()->min_fps;
				}
				else if (tagEquals(tag, "maxSubSteps"))
					SkyrimPhysicsWorld::get()->m_maxSubSteps = btClamped(reader.readInt(), 1, 60);
				else {
					_WARNING("Unknown config : %s", tag.c_str());
					reader.skipCurrentElement();
				}
				break;
			}
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void wind(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag: {
				auto tag = reader.GetLocalName();
				if (tagEquals(tag, "windStrength"))
					SkyrimPhysicsWorld::get()->m_windStrength = btClamped(reader.readFloat(), 0.f, 1000.f);
				else if (tagEquals(tag, "enabled"))
					SkyrimPhysicsWorld::get()->m_enableWind = reader.readBool();
				else if (tagEquals(tag, "distanceForNoWind"))
					SkyrimPhysicsWorld::get()->m_distanceForNoWind = btClamped(reader.readFloat(), 0.f, 10000.f);
				else if (tagEquals(tag, "distanceForMaxWind"))
					SkyrimPhysicsWorld::get()->m_distanceForMaxWind = btClamped(reader.readFloat(), 0.f, 10000.f);
				else {
					_WARNING("Unknown config : %s", tag.c_str());
					reader.skipCurrentElement();
				}
				break;
			}
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void smp(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag: {
				auto tag = reader.GetLocalName();
				if (tagEquals(tag, "logLevel")) {
					auto level = static_cast<IDebugLog::LogLevel>(reader.readInt());
					gLog.SetLogLevel(level);
					hdt::logging::configuredLogLevel.store(level, std::memory_order_relaxed);
				}
				else if (tagEquals(tag, "enableNPCFaceParts"))
					ActorManager::instance()->m_skinNPCFaceParts = reader.readBool();
				else if (tagEquals(tag, "disableSMPHairWhenWigEquipped"))
					ActorManager::instance()->m_disableSMPHairWhenWigEquipped = reader.readBool();
				else if (tagEquals(tag, "clampRotations"))
					SkyrimPhysicsWorld::get()->m_clampRotations = reader.readBool();
				else if (tagEquals(tag, "rotationSpeedLimit"))
					SkyrimPhysicsWorld::get()->m_rotationSpeedLimit = reader.readFloat();
				else if (tagEquals(tag, "unclampedResets"))
					SkyrimPhysicsWorld::get()->m_unclampedResets = reader.readBool();
				else if (tagEquals(tag, "unclampedResetAngle"))
					SkyrimPhysicsWorld::get()->m_unclampedResetAngle = reader.readFloat();
				else if (tagEquals(tag, "percentageOfFrameTime"))
					SkyrimPhysicsWorld::get()->m_percentageOfFrameTime = std::clamp(reader.readInt() * 10, 1, 1000);
				else if (tagEquals(tag, "useRealTime"))
					SkyrimPhysicsWorld::get()->m_useRealTime = reader.readBool();
#ifdef CUDA
				else if (tagEquals(tag, "enableCuda"))
					CudaInterface::enableCuda = reader.readBool();
				else if (tagEquals(tag, "cudaDevice")) {
					int device = reader.readInt();
					if (device >= 0 && device < CudaInterface::instance()->deviceCount())
						CudaInterface::currentDevice = device;
				}
#else
				else if (tagEquals(tag, "enableCuda")) {
					if (reader.readBool())
						_MESSAGE("CUDA isn't built into this version.");
				}
				else if (tagEquals(tag, "cudaDevice")) {
					reader.readInt();
				}
#endif
				else if (tagEquals(tag, "minCullingDistance"))
					ActorManager::instance()->m_minCullingDistance = reader.readFloat();
				else if (tagEquals(tag, "maximumActiveSkeletons"))
					ActorManager::instance()->m_maxActiveSkeletons = reader.readInt();
				else if (tagEquals(tag, "autoAdjustMaxSkeletons"))
					ActorManager::instance()->m_autoAdjustMaxSkeletons = reader.readBool();
				else if (tagEquals(tag, "sampleSize"))
					SkyrimPhysicsWorld::get()->m_sampleSize = std::max(reader.readInt(), 1);
				else if (tagEquals(tag, "disable1stPersonViewPhysics"))
					ActorManager::instance()->m_disable1stPersonViewPhysics = reader.readBool();
				else {
					_WARNING("Unknown config : %s", tag.c_str());
					reader.skipCurrentElement();
				}
				break;
			}
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void config(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag: {
				auto tag = reader.GetLocalName();
				if (tagEquals(tag, "solver"))
					solver(reader);
				else if (tagEquals(tag, "wind"))
					wind(reader);
				else if (tagEquals(tag, "smp"))
					smp(reader);
				else {
					_WARNING("Unknown config : %s", tag.c_str());
					reader.skipCurrentElement();
				}
				break;
			}
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	void loadConfig()
	{
		auto bytes = readAllFile2("data/skse/plugins/hdtSkinnedMeshConfigs/configs.xml");
		if (bytes.empty()) {
			_WARNING("Config file not found: data/skse/plugins/hdtSkinnedMeshConfigs/configs.xml");
			return;
		}

		// Store original locale
		char saved_locale[32];
		strcpy_s(saved_locale, std::setlocale(LC_NUMERIC, nullptr));

		// Set locale to en_US
		std::setlocale(LC_NUMERIC, "en_US");

		XMLReader reader((uint8_t*)bytes.data(), bytes.size());

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				auto tag = reader.GetLocalName();
				if (tagEquals(tag, "configs"))
					config(reader);
				else {
					_WARNING("Unknown config : %s", tag.c_str());
					reader.skipCurrentElement();
				}
			}
		}

		// Restore original locale
		std::setlocale(LC_NUMERIC, saved_locale);

		// Report configured log level - ALWAYS show this regardless of level setting
		auto level = hdt::logging::configuredLogLevel.load(std::memory_order_relaxed);
		char buf[256];
		snprintf(buf, sizeof(buf), "[CONFIG] logLevel=%d (%s) - messages above level %d filtered",
				 static_cast<int>(level), hdt::logging::GetLevelName(level), static_cast<int>(level));
		gLog.Message(buf);
	}
} // namespace hdt
