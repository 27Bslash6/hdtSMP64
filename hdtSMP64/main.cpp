#include "hdtSkyrimPhysicsWorld.h"

#include "ActorManager.h"
#include "config.h"
#include "EventDebugLogger.h"
#include "HookEvents.h"
#include "Hooks.h"
#include "Offsets.h"
#include "PluginInterfaceImpl.h"
#include "skse64/GameMenus.h"
#include "skse64/GameReferences.h"
#include "skse64/ObScript.h"
#include "skse64/PluginAPI.h"
#include "skse64_common/SafeWrite.h"
#include "skse64_common/skse_version.h"

#include <DbgHelp.h>
#include <fstream>
#include <numeric>
#include <regex>
#include <shlobj_core.h>
#include <sstream>
#pragma comment(lib, "DbgHelp.lib")
#include "hdtSkinnedMesh/hdtFrameTimer.h"

#include "skse64/GameRTTI.h"
#include "skse64_common/BranchTrampoline.h"
#ifdef CUDA
#include "hdtSkinnedMesh/hdtCudaInterface.h"
#endif

#include "hdtLog.h"
#include "hdtSaveNameValidator.h"

#include "BuildInfo.h"
#include "WeatherManager.h"

namespace hdt
{
	constexpr UInt32 hdtSMP64Version = 200500; // patch version + 10^2 * minor version + 10^5 * major version

	IDebugLog gLog;
	EventDebugLogger g_eventDebugLogger;
	PluginHandle g_PluginHandle;

	class FreezeEventHandler : public BSTEventSink<MenuOpenCloseEvent>
	{
	public:
		FreezeEventHandler() {}

		EventResult ReceiveEvent(MenuOpenCloseEvent* evn, EventDispatcher<MenuOpenCloseEvent>* dispatcher) override
		{
			auto mm = MenuManager::GetSingleton();

			if (evn && evn->opening &&
				(!strcmp(evn->menuName.data, "Loading Menu") || !strcmp(evn->menuName.data, "RaceSex Menu") ||
				 !strcmp(evn->menuName.data, "Main Menu")))
			{
				_VMESSAGE("FreezeHandler: %s opening, calling suspend(true)", evn->menuName.data);
				SkyrimPhysicsWorld::get()->suspend(true);
			}

			if (evn && !evn->opening && !strcmp(evn->menuName.data, "RaceSex Menu")) {
				_VMESSAGE("FreezeHandler: RaceSex Menu closed, reloading meshes");
				ActorManager::instance()->onEvent(*evn);
			}

			return kEvent_Continue;
		}
	} g_freezeEventHandler;

	void checkOldPlugins()
	{
		auto framework = GetModuleHandleA("hdtSSEFramework");
		auto physics = GetModuleHandleA("hdtSSEPhysics");
		auto hh = GetModuleHandleA("hdtSSEHighHeels");

		if (physics) {
			MessageBox(nullptr,
					   TEXT("hdtSSEPhysics.dll is loaded. This is an older verson of HDT-SMP and conflicts with "
							"hdtSMP64.dll. Please remove it."),
					   TEXT("hdtSMP64"), MB_OK);
		}

		if (framework && !hh) {
			MessageBox(nullptr,
					   TEXT("hdtSSEFramework.dll is loaded but hdtSSEHighHeels.dll is not being used. You no longer "
							"need hdtSSEFramework.dll with this version of SMP. Please remove it."),
					   TEXT("hdtSMP64"), MB_OK);
		}
	}

	NiTexturePtr* GetTextureFromIndex(BSLightingShaderMaterial* material, UInt32 index)
	{
		switch (index) {
		case 0:
			return &material->texture1;
			break;
		case 1:
			return &material->texture2;
			break;
		case 2: {
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen) {
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkB0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_GlowMap) {
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkB0;
			}
			return &material->texture3;
		} break;
		case 3: {
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen) {
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkA8;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Parallax) {
				return &static_cast<BSLightingShaderMaterialParallax*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Parallax ||
				material->GetShaderType() == BSShaderMaterial::kShaderType_ParallaxOcc)
			{
				return &static_cast<BSLightingShaderMaterialParallaxOcc*>(material)->unkA0;
			}
		} break;
		case 4: {
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Eye) {
				return &static_cast<BSLightingShaderMaterialEye*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_EnvironmentMap) {
				return &static_cast<BSLightingShaderMaterialEnvmap*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax) {
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkA8;
			}
		} break;
		case 5: {
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Eye) {
				return &static_cast<BSLightingShaderMaterialEye*>(material)->unkA8;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_EnvironmentMap) {
				return &static_cast<BSLightingShaderMaterialEnvmap*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax) {
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkB0;
			}
		} break;
		case 6: {
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen) {
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->renderedTexture;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax) {
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkA0;
			}
		} break;
		case 7:
			return &material->texture4;
			break;
		}

		return nullptr;
	}

	void DumpNodeChildren(NiAVObject* node)
	{
		_MESSAGE("{%s} {%s} {%X} [%f, %f, %f]", node->GetRTTI()->name, node->m_name, node, node->m_worldTransform.pos.x,
				 node->m_worldTransform.pos.y, node->m_worldTransform.pos.z);
		if (node->m_extraDataLen > 0) {
			gLog.Indent();
			for (UInt16 i = 0; i < node->m_extraDataLen; i++) {
				_MESSAGE("{%s} {%s} {%X}", node->m_extraData[i]->GetRTTI()->name, node->m_extraData[i]->m_pcName, node);
			}
			gLog.Outdent();
		}

		NiNode* niNode = node->GetAsNiNode();
		if (niNode && niNode->m_children.m_emptyRunStart > 0) {
			gLog.Indent();
			for (int i = 0; i < niNode->m_children.m_emptyRunStart; i++) {
				NiAVObject* object = niNode->m_children.m_data[i];
				if (object) {
					NiNode* childNode = object->GetAsNiNode();
					BSGeometry* geometry = object->GetAsBSGeometry();
					if (geometry) {
						_MESSAGE("{%s} {%s} {%X} [%f, %f, %f] - Geometry", object->GetRTTI()->name, object->m_name,
								 object, geometry->m_worldTransform.pos.x, geometry->m_worldTransform.pos.y,
								 geometry->m_worldTransform.pos.z);
						if (geometry->m_spSkinInstance && geometry->m_spSkinInstance->m_spSkinData) {
							gLog.Indent();
							for (int i = 0; i < geometry->m_spSkinInstance->m_spSkinData->m_uiBones; i++) {
								auto bone = geometry->m_spSkinInstance->m_ppkBones[i];
								_MESSAGE("Bone %d - {%s} {%s} {%X} [%f, %f, %f]", i, bone->GetRTTI()->name,
										 bone->m_name, bone, bone->m_worldTransform.pos.x, bone->m_worldTransform.pos.y,
										 bone->m_worldTransform.pos.z);
							}
							gLog.Outdent();
						}
						NiPointer<BSShaderProperty> shaderProperty =
							niptr_cast<BSShaderProperty>(geometry->m_spEffectState);
						if (shaderProperty) {
							BSLightingShaderProperty* lightingShader =
								ni_cast(shaderProperty, BSLightingShaderProperty);
							if (lightingShader) {
								BSLightingShaderMaterial* material =
									static_cast<BSLightingShaderMaterial*>(lightingShader->material);

								gLog.Indent();
								for (int i = 0; i < BSTextureSet::kNumTextures; ++i) {
									const char* texturePath = material->textureSet->GetTexturePath(i);
									if (!texturePath) {
										continue;
									}

									const char* textureName = "";
									NiTexturePtr* texture = GetTextureFromIndex(material, i);
									if (texture && texture->get()) {
										textureName = texture->get()->name;
									}

									_MESSAGE("Texture %d - %s (%s)", i, texturePath, textureName);
								}
								_MESSAGE("Flags - %08X %08X", lightingShader->shaderFlags1,
										 lightingShader->shaderFlags2);
								gLog.Outdent();
							}
						}
					}
					else if (childNode) {
						DumpNodeChildren(childNode);
					}
					else {
						_MESSAGE("{%s} {%s} {%X} [%f, %f, %f]", object->GetRTTI()->name, object->m_name, object,
								 object->m_worldTransform.pos.x, object->m_worldTransform.pos.y,
								 object->m_worldTransform.pos.z);
					}
				}
			}
			gLog.Outdent();
		}
	}

	void SMPDebug_PrintDetailed(bool includeItems)
	{
		static std::map<ActorManager::SkeletonState, char*> stateStrings = {
			{ActorManager::SkeletonState::e_InactiveNotInScene, "Not in scene"},
			{ActorManager::SkeletonState::e_InactiveUnseenByPlayer, "Unseen by player"},
			{ActorManager::SkeletonState::e_InactiveTooFar, "Deactivated for performance"},
			{ActorManager::SkeletonState::e_ActiveIsPlayer, "Is player character"},
			{ActorManager::SkeletonState::e_ActiveNearPlayer, "Is near player"}};

		auto skeletons = ActorManager::instance()->getSkeletons();
		std::vector<int> order(skeletons.size());
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) { return skeletons[a].state < skeletons[b].state; });

		for (int i : order) {
			auto& skeleton = skeletons[i];

			TESObjectREFR* skelOwner = nullptr;
			TESFullName* ownerName = nullptr;

			if (skeleton.skeleton->m_owner) {
				skelOwner = skeleton.skeleton->m_owner;
				if (skelOwner->baseForm)
					ownerName = DYNAMIC_CAST(skelOwner->baseForm, TESForm, TESFullName);
			}

			Console_Print("[HDT-SMP] %s skeleton - owner %s (refr formid %08x, base formid %08x) - %s",
						  skeleton.state > ActorManager::SkeletonState::e_SkeletonActive ? "active" : "inactive",
						  ownerName ? ownerName->GetName() : "unk_name", skelOwner ? skelOwner->formID : 0x00000000,
						  skelOwner && skelOwner->baseForm ? skelOwner->baseForm->formID : 0x00000000,
						  stateStrings[skeleton.state]);

			if (includeItems) {
				for (auto armor : skeleton.getArmors()) {
					Console_Print("[HDT-SMP] -- tracked armor addon %s, %s", armor.armorWorn->m_name,
								  armor.state() != ActorManager::ItemState::e_NoPhysics
									  ? armor.state() == ActorManager::ItemState::e_Active ? "has active physics system"
																						   : "has inactive physics "
																							 "system"
									  : "has no physics system");

					if (armor.state() != ActorManager::ItemState::e_NoPhysics) {
						for (auto mesh : armor.meshes())
							Console_Print("[HDT-SMP] ---- has collision mesh %s", mesh->m_name->cstr());
					}
				}

				if (skeleton.head.headNode) {
					for (auto headPart : skeleton.head.headParts) {
						Console_Print("[HDT-SMP] -- tracked headpart %s, %s", headPart.headPart->m_name,
									  headPart.state() != ActorManager::ItemState::e_NoPhysics
										  ? headPart.state() == ActorManager::ItemState::e_Active ? "has active "
																									"physics system"
																								  : "has inactive "
																									"physics system"
										  : "has no physics system");

						if (headPart.state() != ActorManager::ItemState::e_NoPhysics) {
							for (auto mesh : headPart.meshes())
								Console_Print("[HDT-SMP] ---- has collision mesh %s", mesh->m_name->cstr());
						}
					}
				}
			}
		}
	}

	// Helper to update a single XML tag value (line-based for robustness)
	// Handles whitespace variations and preserves indentation
	bool updateXmlTag(std::string& content, const char* tag, const std::string& value)
	{
		std::string openTag = std::string("<") + tag + ">";
		std::string closeTag = std::string("</") + tag + ">";

		std::istringstream stream(content);
		std::ostringstream result;
		std::string line;
		bool found = false;
		bool firstLine = true;

		while (std::getline(stream, line)) {
			if (!firstLine)
				result << "\n";
			firstLine = false;

			// Check if this line contains our tag (not in a comment)
			size_t openPos = line.find(openTag);
			size_t closePos = line.find(closeTag);
			size_t commentPos = line.find("<!--");

			// Only match if both tags found and not inside a comment
			if (openPos != std::string::npos && closePos != std::string::npos && openPos < closePos &&
				(commentPos == std::string::npos || openPos < commentPos))
			{
				// Preserve leading whitespace
				size_t indent = line.find_first_not_of(" \t");
				if (indent == std::string::npos)
					indent = 0;
				result << line.substr(0, indent) << openTag << value << closeTag;
				found = true;
			}
			else {
				result << line;
			}
		}

		if (found) {
			content = result.str();
		}
		return found;
	}

	bool saveCurrentConfig()
	{
		const char* configPath = "data/skse/plugins/hdtSkinnedMeshConfigs/configs.xml";

		// Read current config
		std::ifstream inFile(configPath);
		if (!inFile.is_open()) {
			return false;
		}
		std::stringstream buffer;
		buffer << inFile.rdbuf();
		std::string content = buffer.str();
		inFile.close();

		auto world = SkyrimPhysicsWorld::get();
		auto actor = ActorManager::instance();

		// Update values
		int percentageForConfig = world->m_percentageOfFrameTime / 10; // Convert back to 1-100 scale
		updateXmlTag(content, "percentageOfFrameTime", std::to_string(percentageForConfig));
		updateXmlTag(content, "maximumActiveSkeletons", std::to_string(actor->m_maxActiveSkeletons));
		updateXmlTag(content, "autoAdjustMaxSkeletons", actor->m_autoAdjustMaxSkeletons ? "true" : "false");

		// Write back
		std::ofstream outFile(configPath);
		if (!outFile.is_open()) {
			return false;
		}
		outFile << content;
		outFile.close();

		return true;
	}

	bool SMPDebug_Execute(const ObScriptParam* paramInfo, ScriptData* scriptData, TESObjectREFR* thisObj,
						  TESObjectREFR* containingObj, Script* scriptObj, ScriptLocals* locals, double& result,
						  UInt32& opcodeOffsetPtr)
	{
		char buffer[MAX_PATH];
		memset(buffer, 0, MAX_PATH);
		char buffer2[MAX_PATH];
		memset(buffer2, 0, MAX_PATH);

		if (!ObScript_ExtractArgs(paramInfo, scriptData, opcodeOffsetPtr, thisObj, containingObj, scriptObj, locals,
								  buffer, buffer2))
		{
			return false;
		}

		// Show help when called with no arguments
		if (buffer[0] == '\0') {
			Console_Print("[HDT-SMP] Commands:");
			Console_Print("  smp fps [N]     - Set/show target FPS (30-240)");
			Console_Print("  smp skeletons [N] - Set/show max skeletons");
			Console_Print("  smp autoscale   - Toggle auto-scaling");
			Console_Print("  smp save        - Save settings to configs.xml");
			Console_Print("  smp stats       - Show performance stats");
			Console_Print("  smp reset|reload|on|off - Control physics");
#ifdef CUDA
			Console_Print("  smp gpu|cuda|gputiming - CUDA controls");
#endif
			Console_Print("  smp timing|metrics|list|detail - Diagnostics");
			return true;
		}

		if (_strnicmp(buffer, "reset", MAX_PATH) == 0) {
			Console_Print("running full smp reset");
			_VMESSAGE("=== SMP RESET: Starting full physics reset ===");
			// Suspend physics to prevent new frames from starting while we reset
			// This waits for current async tasks AND blocks new physics work
			_VMESSAGE("SMP RESET: calling suspend()...");
			SkyrimPhysicsWorld::get()->suspend();
			_VMESSAGE("SMP RESET: suspend() complete, loading config...");
			hdt::loadConfig();
			_VMESSAGE("SMP RESET: config loaded, calling resetTransformsToOriginal()...");
			SkyrimPhysicsWorld::get()->resetTransformsToOriginal();
			_VMESSAGE("SMP RESET: transforms reset, triggering ActorManager menu close event...");
			const MenuOpenCloseEvent e{false};
			ActorManager::instance()->onEvent(e);
			_VMESSAGE("SMP RESET: ActorManager event complete, calling resetSystems()...");
			SkyrimPhysicsWorld::get()->resetSystems();
			_VMESSAGE("SMP RESET: resetSystems() complete, calling resume()...");
			// Resume physics now that reset is complete
			SkyrimPhysicsWorld::get()->resume();
			_VMESSAGE("=== SMP RESET: Complete ===");
			return true;
		}
		if (_strnicmp(buffer, "reload", MAX_PATH) == 0) {
			Console_Print("[hdtSMP64] Reloading configs.xml...");
			hdt::loadConfig();
			Console_Print("[hdtSMP64] Config reloaded. Changes to solver/wind/smp settings now active.");
			_MESSAGE("Console command: config reloaded via 'smp reload'");
			return true;
		}
		if (_strnicmp(buffer, "fps", MAX_PATH) == 0) {
			auto world = SkyrimPhysicsWorld::get();
			auto actor = ActorManager::instance();

			if (buffer2[0] != '\0') {
				int targetFps = atoi(buffer2);
				if (targetFps >= 30 && targetFps <= 240) {
					// Calculate percentage to give 30% of target frame time to SMP
					// percentage = (min_fps / target_fps) * 300
					int newPercentage = static_cast<int>((world->min_fps * 300.0f) / targetFps);
					newPercentage = std::clamp(newPercentage, 50, 1000);
					world->m_percentageOfFrameTime = newPercentage;

					float budgetMs = world->m_timeTick * newPercentage;
					Console_Print("[HDT-SMP] Target: %d FPS (%.1fms budget, %d%% of frame)", targetFps, budgetMs,
								  newPercentage / 10);
				}
				else {
					Console_Print("[HDT-SMP] Invalid FPS (use 30-240)");
				}
			}
			else {
				// Show current target FPS equivalent
				float budgetMs = world->m_timeTick * world->m_percentageOfFrameTime;
				int effectiveFps = static_cast<int>((world->min_fps * 300.0f) / world->m_percentageOfFrameTime);
				Console_Print("[HDT-SMP] Current: ~%d FPS target (%.1fms budget)", effectiveFps, budgetMs);
				Console_Print("[HDT-SMP] Usage: smp fps <30-240>");
			}
			return true;
		}
		if (_strnicmp(buffer, "skeletons", MAX_PATH) == 0) {
			auto actor = ActorManager::instance();

			if (buffer2[0] != '\0') {
				int maxSkel = atoi(buffer2);
				if (maxSkel >= 1 && maxSkel <= 100) {
					actor->m_maxActiveSkeletons = maxSkel;
					Console_Print("[HDT-SMP] Max skeletons: %d (auto-adjust: %s)", maxSkel,
								  actor->m_autoAdjustMaxSkeletons ? "ON" : "OFF");
				}
				else {
					Console_Print("[HDT-SMP] Invalid count (use 1-100)");
				}
			}
			else {
				Console_Print("[HDT-SMP] Max skeletons: %d (auto-adjust: %s)", actor->m_maxActiveSkeletons,
							  actor->m_autoAdjustMaxSkeletons ? "ON" : "OFF");
				Console_Print("[HDT-SMP] Usage: smp skeletons <1-100>");
			}
			return true;
		}
		if (_strnicmp(buffer, "autoscale", MAX_PATH) == 0) {
			auto actor = ActorManager::instance();
			actor->m_autoAdjustMaxSkeletons = !actor->m_autoAdjustMaxSkeletons;
			Console_Print("[HDT-SMP] Auto-scale: %s", actor->m_autoAdjustMaxSkeletons ? "ON" : "OFF");
			return true;
		}
		if (_strnicmp(buffer, "save", MAX_PATH) == 0) {
			auto world = SkyrimPhysicsWorld::get();
			auto actor = ActorManager::instance();

			if (saveCurrentConfig()) {
				int effectiveFps = static_cast<int>((world->min_fps * 300.0f) / world->m_percentageOfFrameTime);
				Console_Print("[HDT-SMP] Saved to configs.xml:");
				Console_Print("  Target: ~%d FPS (%d%% of frame)", effectiveFps, world->m_percentageOfFrameTime / 10);
				Console_Print("  Max skeletons: %d", actor->m_maxActiveSkeletons);
				Console_Print("  Auto-scale: %s", actor->m_autoAdjustMaxSkeletons ? "ON" : "OFF");
			}
			else {
				Console_Print("[HDT-SMP] Failed to save config (file not found or read-only)");
			}
			return true;
		}
#ifdef CUDA
		if (_strnicmp(buffer, "gpu", MAX_PATH) == 0) {
			CudaInterface::enableCuda = !CudaInterface::enableCuda;
			if (CudaInterface::instance()->hasCuda()) {
				Console_Print("CUDA collision enabled");
			}
			else {
				Console_Print("CUDA collision disabled");
			}
			return true;
		}
		if (_strnicmp(buffer, "cuda", MAX_PATH) == 0) {
			// Toggle CUDA metrics collection or show report
			if (buffer2[0] != '\0' && _strnicmp(buffer2, "reset", MAX_PATH) == 0) {
				CudaInterface::resetMetrics();
				Console_Print("[CUDA] Metrics reset");
			}
			else if (CudaInterface::collectMetrics) {
				// Was collecting - stop and show report
				CudaInterface::collectMetrics = false;
				Console_Print("[CUDA] Metrics collection stopped");
				auto report = CudaInterface::graphMetrics().report();
				// Print each line separately for console
				std::istringstream iss(report);
				std::string line;
				while (std::getline(iss, line)) {
					Console_Print("%s", line.c_str());
				}
			}
			else {
				// Start collecting
				CudaInterface::resetMetrics();
				CudaInterface::collectMetrics = true;
				Console_Print("[CUDA] Metrics collection started (run 'smp cuda' again to see results)");
			}
			return true;
		}
		if (_strnicmp(buffer, "gputiming", MAX_PATH) == 0) {
			// Toggle GPU timing collection or show report
			if (buffer2[0] != '\0' && _strnicmp(buffer2, "reset", MAX_PATH) == 0) {
				CudaInterface::resetMetrics();
				Console_Print("[GPU Timing] Reset");
			}
			else if (CudaInterface::gpuTimingEnabled) {
				// Was collecting - stop and show report
				CudaInterface::gpuTimingEnabled = false;
				Console_Print("[GPU Timing] Stopped");
				auto report = CudaInterface::gpuTiming().report();
				std::istringstream iss(report);
				std::string line;
				while (std::getline(iss, line)) {
					Console_Print("%s", line.c_str());
				}
			}
			else {
				// Start collecting
				CudaInterface::resetMetrics();
				CudaInterface::gpuTimingEnabled = true;
				Console_Print("[GPU Timing] Started - run 'smp gputiming' again to see results");
				Console_Print("  Measures actual GPU execution time via CUDA events");
			}
			return true;
		}
#endif
		if (_strnicmp(buffer, "timing", MAX_PATH) == 0) {
			int frames = 200;
			if (buffer2[0] != '\0') {
				frames = atoi(buffer2);
				if (frames < 10)
					frames = 10;
				if (frames > 1000)
					frames = 1000;
			}
			FrameTimer::instance()->reset(frames);
			Console_Print("Started frame timing for %d frames", frames);
			return true;
		}
		if (_strnicmp(buffer, "metrics", MAX_PATH) == 0) {
			auto world = SkyrimPhysicsWorld::get();
			world->m_forceMetrics = !world->m_forceMetrics;
			if (world->m_forceMetrics) {
				Console_Print("[HDT-SMP] Metrics logging enabled (check hdtSMP64.log)");
			}
			else {
				Console_Print("[HDT-SMP] Metrics logging disabled");
				Console_Print("[HDT-SMP] Avg main loop: %.2f ms, Avg 2nd step: %.2f ms",
							  world->m_averageSMPProcessingTimeInMainLoop, world->m_2ndStepAverageProcessingTime);
			}
			return true;
		}
		if (_strnicmp(buffer, "stats", MAX_PATH) == 0) {
			auto world = SkyrimPhysicsWorld::get();
			Console_Print("[HDT-SMP] Performance:");
			Console_Print("  Main: %.3f ms | 2nd: %.3f ms | Total: %.3f ms",
						  world->m_averageSMPProcessingTimeInMainLoop, world->m_2ndStepAverageProcessingTime,
						  world->m_averageSMPProcessingTimeInMainLoop + world->m_2ndStepAverageProcessingTime);
			float fps = 1000.0f /
						(world->m_averageSMPProcessingTimeInMainLoop + world->m_2ndStepAverageProcessingTime + 0.001f);
			Console_Print("  Max SMP FPS: %.1f | Metrics: %s", fps > 1000 ? 1000.0f : fps,
						  world->m_forceMetrics ? "ON" : "OFF");

			Console_Print("[HDT-SMP] Solver:");
			Console_Print("  Iterations: %d | GroupIter: %d | MLCP: %s | ERP: %.3f",
						  world->getSolverInfo().m_numIterations, ConstraintGroup::MaxIterations,
						  ConstraintGroup::EnableMLCP ? "ON" : "OFF", world->getSolverInfo().m_erp);
			Console_Print("  MinFPS: %d (%.4fs) | MaxSubSteps: %d", world->min_fps, world->m_timeTick,
						  world->m_maxSubSteps);

			auto skeletons = ActorManager::instance()->getSkeletons();
			size_t activeSkeletons = 0, armors = 0, activeArmors = 0, activeCollisionMeshes = 0;
			for (auto skeleton : skeletons) {
				if (skeleton.state > ActorManager::SkeletonState::e_SkeletonActive)
					activeSkeletons++;
				for (const auto armor : skeleton.getArmors()) {
					armors++;
					if (armor.state() == ActorManager::ItemState::e_Active) {
						activeArmors++;
						activeCollisionMeshes += armor.meshes().size();
					}
				}
			}
			Console_Print("[HDT-SMP] Actors:");
			Console_Print("  Skeletons: %d/%d | Armors: %d/%d | Meshes: %d", activeSkeletons, skeletons.size(),
						  activeArmors, armors, activeCollisionMeshes);
			return true;
		}
		if (_strnicmp(buffer, "dumptree", MAX_PATH) == 0) {
			if (thisObj) {
				Console_Print("dumping targeted reference's node tree");
				DumpNodeChildren(thisObj->GetNiRootNode(0));
			}
			else {
				Console_Print("error: you must target a reference to dump their node tree");
			}

			return true;
		}
		if (_strnicmp(buffer, "detail", MAX_PATH) == 0) {
			SMPDebug_PrintDetailed(true);
			return true;
		}
		if (_strnicmp(buffer, "list", MAX_PATH) == 0) {
			SMPDebug_PrintDetailed(false);
			return true;
		}
		if (_strnicmp(buffer, "on", MAX_PATH) == 0) {
			SkyrimPhysicsWorld::get()->disabled = false;
			{
				Console_Print("HDT-SMP enabled");
			}
			return true;
		}
		if (_strnicmp(buffer, "off", MAX_PATH) == 0) {
			SkyrimPhysicsWorld::get()->disabled = true;
			{
				Console_Print("HDT-SMP disabled");
			}
			return true;
		}

		if (_strnicmp(buffer, "QueryOverride", MAX_PATH) == 0) {
			Console_Print("%s", hdt::Override::OverrideManager::GetSingleton()->queryOverrideData().c_str());
			return true;
		}

		// Unknown command - show help hint
		Console_Print("[HDT-SMP] Unknown command: %s", buffer);
		Console_Print("[HDT-SMP] Type 'smp' for help");
		return true;
	}

	int filterException(int code, PEXCEPTION_POINTERS ex)
	{
		_FATALERROR("SEH exception caught while loading FSMP plugin into SKSE.");
		if (code == -529697949) {
			_FATALERROR(
				"This exception occurs when a system process, application, or file fails to open, or your system lacks some necessary redistributable packages like Visual C++ extensions.\
						It can be caused by Damaged or Corrupt system files, Missing files in the registry, Improper configuration of system files, Conflict with third-party programs.\
						Please see https://www.elevenforum.com/t/0xe06d7363-error-which-fix-to-use.8382/post-201372. SEH exception code: %x",
				code);
			return EXCEPTION_EXECUTE_HANDLER;
		}
		else {
			_FATALERROR("Contact DaydreamingDay on the FSMP discord server, and provide him with this SEH exception "
						"code: %x. The discord invite is on the Nexus FSMP description page.",
						code);
			return EXCEPTION_EXECUTE_HANDLER;
		}
	}

	// Global crash handler - logs crash info before game's handler takes over
	static LONG WINAPI hdtCrashHandler(PEXCEPTION_POINTERS ex)
	{
		// Only log actual crashes, not C++ exceptions or breakpoints
		DWORD code = ex->ExceptionRecord->ExceptionCode;
		if (code == EXCEPTION_ACCESS_VIOLATION || code == EXCEPTION_ARRAY_BOUNDS_EXCEEDED ||
			code == EXCEPTION_STACK_OVERFLOW || code == EXCEPTION_ILLEGAL_INSTRUCTION ||
			code == EXCEPTION_IN_PAGE_ERROR || code == EXCEPTION_INT_DIVIDE_BY_ZERO ||
			code == EXCEPTION_PRIV_INSTRUCTION)
		{
			// Build entire crash report atomically to avoid interleaved output from multiple threads
			char crashReport[8192];
			char* p = crashReport;
			char* end = crashReport + sizeof(crashReport) - 1;

			auto append = [&p, end](const char* fmt, ...) {
				if (p >= end)
					return;
				va_list args;
				va_start(args, fmt);
				int written = vsnprintf(p, end - p, fmt, args);
				va_end(args);
				if (written > 0)
					p += written;
			};

			append("=== HDT-SMP CRASH [Thread %lu] ===\n", GetCurrentThreadId());
			append("Exception Code: 0x%08X\n", code);
			append("Exception Addr: 0x%p\n", ex->ExceptionRecord->ExceptionAddress);

			if (code == EXCEPTION_ACCESS_VIOLATION && ex->ExceptionRecord->NumberParameters >= 2) {
				const char* op = ex->ExceptionRecord->ExceptionInformation[0] == 0 ? "reading" : "writing";
				append("Access violation %s address: 0x%p\n", op, (void*)ex->ExceptionRecord->ExceptionInformation[1]);
			}

			// Log registers
			CONTEXT* ctx = ex->ContextRecord;
			append("RIP=0x%p RSP=0x%p RBP=0x%p\n", (void*)ctx->Rip, (void*)ctx->Rsp, (void*)ctx->Rbp);
			append("RAX=0x%p RBX=0x%p RCX=0x%p RDX=0x%p\n", (void*)ctx->Rax, (void*)ctx->Rbx, (void*)ctx->Rcx,
				   (void*)ctx->Rdx);

			// Capture stack trace with symbol names
			HANDLE process = GetCurrentProcess();
			SymInitialize(process, NULL, TRUE);

			append("Stack trace:\n");
			void* stack[32];
			USHORT frames = CaptureStackBackTrace(0, 32, stack, NULL);

			char symbolBuffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)];
			PSYMBOL_INFO symbol = (PSYMBOL_INFO)symbolBuffer;
			symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
			symbol->MaxNameLen = MAX_SYM_NAME;

			HMODULE hMod = GetModuleHandleA("hdtSMP64.dll");
			ULONGLONG dllBase = hMod ? (ULONGLONG)hMod : 0;

			for (USHORT i = 0; i < frames; i++) {
				DWORD64 address = (DWORD64)stack[i];
				DWORD64 displacement = 0;

				if (SymFromAddr(process, address, &displacement, symbol)) {
					append("  [%02d] %s+0x%llX (0x%p)\n", i, symbol->Name, displacement, stack[i]);
				}
				else if (dllBase && address >= dllBase && address < dllBase + 0x1000000) {
					append("  [%02d] hdtSMP64.dll+0x%llX (0x%p)\n", i, address - dllBase, stack[i]);
				}
				else {
					append("  [%02d] 0x%p\n", i, stack[i]);
				}
			}

			if (hMod) {
				append("hdtSMP64.dll base: 0x%p (crash offset: 0x%llX)\n", hMod,
					   (ULONGLONG)ex->ExceptionRecord->ExceptionAddress - dllBase);
			}

			SymCleanup(process);
			append("=== END CRASH INFO ===");

			// Single atomic log call
			_FATALERROR("%s", crashReport);
		}

		// Continue search - let game's handler deal with it
		return EXCEPTION_CONTINUE_SEARCH;
	}

	static PVOID g_vehHandle = nullptr;

	static void installCrashHandler()
	{
		g_vehHandle = AddVectoredExceptionHandler(1, hdtCrashHandler);
		if (g_vehHandle) {
			_MESSAGE("HDT-SMP crash handler installed");
		}
	}

	/* This function is the most prone to SEH exceptions. */
	static bool enclosedLoadConfig(const SKSEInterface* skse)
	{
		__try
		{
			hdt::loadConfig();
		}
		__except (hdt::filterException(GetExceptionCode(), GetExceptionInformation()))
		{
			_FATALERROR("A fatal exception has occurred thrown while reading FSMP's configs.xml");
			return false;
		}
		return true;
	}

	static bool hdtSKSEPlugin_Load(const SKSEInterface* skse)
	{
#ifdef ANNIVERSARY_EDITION
		hdt::gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim Special Edition\\SKSE\\hdtSMP64.log");
		hdt::gLog.SetAutoFlush(true); // Ensure logs are written immediately
		// Set initial log level (will be overridden by config)
		hdt::gLog.SetLogLevel(IDebugLog::LogLevel::kLevel_Message);
		hdt::logging::configuredLogLevel.store(IDebugLog::kLevel_Message, std::memory_order_relaxed);
		installCrashHandler(); // Install VEH crash handler early
		_MESSAGE("hdtSMP64 v%lu", hdt::hdtSMP64Version);
		_MESSAGE("  Build: %s %s", hdt::BuildInfo::GetBuildDate(), hdt::BuildInfo::GetBuildTime());
		_MESSAGE("  Target: %s | %s | %s", hdt::BuildInfo::GetGameVersionString(), hdt::BuildInfo::GetCudaStatus(),
				 hdt::BuildInfo::GetAVXLevel());
		_MESSAGE("  Compiler: %s (%s)", hdt::BuildInfo::GetCompilerInfo(), hdt::BuildInfo::GetBuildType());
		_MESSAGE("  Logging: async, level=%d (%s)", static_cast<int>(IDebugLog::kLevel_Message),
				 hdt::logging::GetLevelName(IDebugLog::kLevel_Message));
#ifdef HDT_TRACY_ENABLE
		_MESSAGE("  Profiling: Tracy ENABLED - connect profiler to capture data");
#endif

		if (!g_branchTrampoline.Create(1024 * 1)) {
			_FATALERROR("Couldn't create branch trampoline. This is fatal. Skipping remainder of init process.");
			return false;
		}

		if (!g_localTrampoline.Create(1024 * 1, nullptr)) {
			_FATALERROR("Couldn't create codegen buffer. This is fatal. Skipping remainder of init process.");
			return false;
		}

		hdt::g_PluginHandle = skse->GetPluginHandle();
#endif // ANNIVERSARY_EDITION

		hdt::g_frameEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_frameEventDispatcher.addListener(hdt::SkyrimPhysicsWorld::get());
		hdt::g_frameSyncEventDispatcher.addListener(hdt::SkyrimPhysicsWorld::get());
		hdt::g_shutdownEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_shutdownEventDispatcher.addListener(hdt::SkyrimPhysicsWorld::get());
		hdt::g_armorAttachEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_armorDetachEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_skinSingleHeadGeometryEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_skinAllHeadGeometryEventDispatcher.addListener(hdt::ActorManager::instance());

		hdt::hookAll();

		hdt::g_pluginInterface.init(skse);

		const auto messageInterface =
			reinterpret_cast<SKSEMessagingInterface*>(skse->QueryInterface(kInterface_Messaging));
		if (messageInterface) {
			const auto cameraDispatcher = static_cast<EventDispatcher<SKSECameraEvent>*>(
				messageInterface->GetEventDispatcher(SKSEMessagingInterface::kDispatcher_CameraEvent));

			if (cameraDispatcher)
				cameraDispatcher->AddEventSink(hdt::SkyrimPhysicsWorld::get());

			messageInterface->RegisterListener(hdt::g_PluginHandle, "SKSE", [](SKSEMessagingInterface::Message* msg) {
				if (msg && msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
					MenuManager* mm = MenuManager::GetSingleton();
					if (mm)
						mm->MenuOpenCloseEventDispatcher()->AddEventSink(&hdt::g_freezeEventHandler);
					hdt::checkOldPlugins();

					// I think we only have _DEBUG now...
#ifdef DEBUG
					hdt::g_armorAttachEventDispatcher.addListener(&hdt::g_eventDebugLogger);
					GetEventDispatcherList()->unk1B8.AddEventSink(&hdt::g_eventDebugLogger);
					GetEventDispatcherList()->unk840.AddEventSink(&hdt::g_eventDebugLogger);
#endif
				}

				// If we receive a SaveGame message, we serialize our data and save it in our dedicated save files.
				if (msg && msg->type == SKSEMessagingInterface::kMessage_SaveGame) {
					auto data = hdt::Override::OverrideManager::GetSingleton()->Serialize();
					if (!data.str().empty()) {
						std::string save_name = reinterpret_cast<char*>(msg->data);
						// SEC-001: Validate save name to prevent path traversal (CWE-22)
						if (!hdt::security::isValidSaveName(save_name)) {
							_WARNING("HDT-SMP: Invalid save name rejected (potential path traversal): %s",
									 save_name.c_str());
							return;
						}
						std::ofstream ofs(OVERRIDE_SAVE_PATH + save_name + ".dhdt", std::ios::out);
						if (ofs && ofs.is_open())
							ofs << data.str();
					}
				}

				// If we receive a PreLoadGame message, suspend physics IMMEDIATELY before the game
				// starts destroying actors/objects. This is earlier than the Loading Menu event.
				if (msg && msg->type == SKSEMessagingInterface::kMessage_PreLoadGame) {
					_VMESSAGE("PreLoadGame: suspending physics before game destroys world objects");
					SkyrimPhysicsWorld::get()->suspend(true);

					std::string save_name = reinterpret_cast<char*>(msg->data);
					save_name = save_name.substr(0, save_name.find_last_of("."));

					// SEC-001: Validate save name to prevent path traversal (CWE-22)
					if (!hdt::security::isValidSaveName(save_name)) {
						_WARNING("HDT-SMP: Invalid save name rejected (potential path traversal): %s",
								 save_name.c_str());
						return;
					}

					std::ifstream ifs(OVERRIDE_SAVE_PATH + save_name + ".dhdt", std::ios::in);
					if (ifs && ifs.is_open()) {
						std::stringstream data;
						data << ifs.rdbuf();
						hdt::Override::OverrideManager::GetSingleton()->Deserialize(data);
					}
				}

				// Send our public interface to registered plugins
				if (msg && msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
					hdt::g_pluginInterface.onPostPostLoad();
				}
			});
		}

		ObScriptCommand* hijackedCommand = nullptr;
		for (ObScriptCommand* iter = g_firstConsoleCommand;
			 iter->opcode < kObScript_NumConsoleCommands + kObScript_ConsoleOpBase; ++iter)
		{
			if (!strcmp(iter->longName, "ShowRenderPasses")) {
				hijackedCommand = iter;
				break;
			}
		}
		if (hijackedCommand) {
			static ObScriptParam params[2];
			params[0].typeID = ObScriptParam::kType_String;
			params[0].typeStr = "Command (optional)";
			params[0].isOptional = 1;
			params[1].typeID = ObScriptParam::kType_String;
			params[1].typeStr = "Argument (optional)";
			params[1].isOptional = 1;

			ObScriptCommand cmd = *hijackedCommand;

			cmd.longName = "SMPDebug";
			cmd.shortName = "smp";
			cmd.helpText = "smp [command] - Type 'smp' for command list";
			cmd.needsParent = 0;
			cmd.numParams = 2;
			cmd.params = params;
			cmd.execute = hdt::SMPDebug_Execute;
			cmd.flags = 0;
			SafeWriteBuf(reinterpret_cast<uintptr_t>(hijackedCommand), &cmd, sizeof(cmd));
		}

		hdt::papyrus::RegisterAllFunctions(
			reinterpret_cast<SKSEPapyrusInterface*>(skse->QueryInterface(kInterface_Papyrus)));

		if (!enclosedLoadConfig(skse))
			return false;

		if (hdt::SkyrimPhysicsWorld::get()->m_enableWind) {
			_MESSAGE("Wind enabled");
			std::thread t(hdt::WeatherCheck);
			t.detach();
		}
		return true;
	}
} // namespace hdt

extern "C"
{
#ifdef ANNIVERSARY_EDITION
	__declspec(dllexport) SKSEPluginVersionData SKSEPlugin_Version = {
		SKSEPluginVersionData::kVersion,
		hdt::hdtSMP64Version,
		"hdtSMP64",
		"hydrogensaysHDT",
		"",
		0, // not version independent
#ifndef ANNIVERSARY_EDITION_353MINUS
		SKSEPluginVersionData::kVersionIndependent_StructsPost629,
#endif // !ANNIVERSARY_EDITION_353MINUS
		{CURRENT_RELEASE_RUNTIME, 0},
		0, // works with any version of the script extender. you probably do not need to put anything here
	};
#else
	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		// populate info structure
		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "hdtSMP64";
		info->version = hdt::hdtSMP64Version;

		hdt::gLog.OpenRelative(CSIDL_MYDOCUMENTS,
#ifndef SKYRIMVR
							   "\\My Games\\Skyrim Special Edition\\SKSE\\hdtSMP64.log"

#else
							   "\\My Games\\Skyrim VR\\SKSE\\hdtSMP64.log"
#endif
		);
		hdt::gLog.SetAutoFlush(true); // Ensure logs are written immediately
		// Set initial log level (will be overridden by config)
		hdt::gLog.SetLogLevel(IDebugLog::LogLevel::kLevel_Message);
		hdt::logging::configuredLogLevel.store(IDebugLog::kLevel_Message, std::memory_order_relaxed);
		installCrashHandler(); // Install VEH crash handler early

		_MESSAGE("hdtSMP64 v%lu", hdt::hdtSMP64Version);
		_MESSAGE("  Build: %s %s", hdt::BuildInfo::GetBuildDate(), hdt::BuildInfo::GetBuildTime());
		_MESSAGE("  Target: %s | %s | %s", hdt::BuildInfo::GetGameVersionString(), hdt::BuildInfo::GetCudaStatus(),
				 hdt::BuildInfo::GetAVXLevel());
		_MESSAGE("  Compiler: %s (%s)", hdt::BuildInfo::GetCompilerInfo(), hdt::BuildInfo::GetBuildType());
		_MESSAGE("  Logging: async, level=%d (%s)", static_cast<int>(IDebugLog::kLevel_Message),
				 hdt::logging::GetLevelName(IDebugLog::kLevel_Message));
#ifdef HDT_TRACY_ENABLE
		_MESSAGE("  Profiling: Tracy ENABLED - connect profiler to capture data");
#endif

		if (skse->isEditor) {
			return false;
		}

		if (skse->runtimeVersion != CURRENT_RELEASE_RUNTIME) {
			_FATALERROR("attempted to load plugin into unsupported game version, exiting");
			return false;
		}

		if (!g_branchTrampoline.Create(1024 * 1)) {
			_FATALERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
			return false;
		}

		if (!g_localTrampoline.Create(1024 * 1, nullptr)) {
			_FATALERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
			return false;
		}

		hdt::g_PluginHandle = skse->GetPluginHandle();

		return true;
	}
#endif

	bool SKSEPlugin_Load(const SKSEInterface* skse)
	{
		// SKSE for AE now __try'es/__except's the plugins load,
		// but doesn't provide the exception code.
		// So, we __try/__except our code to better log and understand what happens in case of bug.
		int nCode;
		__try
		{
			return hdt::hdtSKSEPlugin_Load(skse);
		}
		__except (nCode = hdt::filterException(GetExceptionCode(), GetExceptionInformation()))
		{
			return nCode == EXCEPTION_CONTINUE_EXECUTION;
		}
	}
}
