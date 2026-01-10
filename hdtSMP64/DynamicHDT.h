#pragma once

#include "hdtSkyrimPhysicsWorld.h"

#include "ActorManager.h"
#include "config.h"
#include "dhdtOverrideManager.h"
#include "dhdtPapyrusFunctions.h"
#include "EventDebugLogger.h"
#include "HookEvents.h"
#include "Hooks.h"
#include "skse64/PluginAPI.h"

#include <functional>
#include <stdexcept>
#include <string>

constexpr auto PAPYRUS_CLASS_NAME = "DynamicHDT";

constexpr auto OVERRIDE_SAVE_PATH = "Data/SKSE/Plugins/hdtOverrideSaves/";

namespace hdt
{
	namespace util
	{
		UInt32 splitArmorAddonFormID(std::string nodeName);

		std::string UInt32toString(UInt32 formID);

		void transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst);
	} // namespace util
} // namespace hdt
