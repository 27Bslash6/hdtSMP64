#pragma once
#include "hdtSkyrimPhysicsWorld.h"

#include "Offsets.h"

#include <skse64/GameExtraData.h>
#include <skse64/GameForms.h>
#include <skse64/GameRTTI.h>
#include <skse64/NiNodes.h>

#include <algorithm>
#include <random>

// All weather code borrowed from Shizof. https://www.nexusmods.com/skyrimspecialedition/mods/24486
// 0x2C8

namespace hdt
{


	class Sky
	{
	public:
		virtual ~Sky();

		// static Sky* GetSingleton(); //unused

		// members
		// vtbl 0x0
		void* unkBSMultiBoundNode;	 // 0x8   BSMultiBoundNode
		NiNode* unkNode;			 // 0x10  NiNode
		UInt64 unk18[5];			 // 0x18
		TESClimate* climate;		 // 0x40
		TESWeather* currentWeather;	 // 0x48  current weather active or being transitioned to
		TESWeather* outgoingWeather; // 0x50  previous weather during transition
		TESWeather* nextWeather;	 // 0x58  next weather in queue, transition hasn't begun yet
		UInt64 unk60;				 // 0x60
		TESRegion* unk68;			 // 0x68
		// following are SkyObjects
		void* atmosphere;	 // 0x70  Atmosphere
		void* stars;		 // 0x78  Stars
		void* sun;			 // 0x80  Sun
		void* clouds;		 // 0x88  Clouds
		void* masser;		 // 0x90  Moon
		void* secunda;		 // 0x98  Moon
		void* precipitation; // 0xA0  Precipitation
		// a whole bunch of unknown floats, some may be UInt32 too
		float unkA8[(0x18c - 0xA8) >> 2];	// 0xA8
		float windSpeed;					// 0x18c
		float windDirection;				// 0x190
		float unk1b0[(0x1B0 - 0x190) >> 2]; // 0x1b0
		float timeOfDay;					// 0x1B0 time of day as float 0.0~24.0
		float unk1B4;						// 0x1B4
		float transition;					// 0x1B8 weather transition amount 0.0 -> 1.0
		UInt32 skyMode;						// 0x1BC
		void* unk1C0;						// 0x1C0
		float unk1BC[(0x278 - 0x1C8) >> 2]; // 0x1C8
		void* skyEffectController;			// 0x278 SkyEffectController : ReferenceEffectController
		UInt64 unk280[3];					// 0x280
		// BSTArray<NiSourceTexture> textures;                        // 0x298 //says same type as tarray in
		// BSTextureSet.h
		tArray<NiSourceTexture> textures; // 0x298 //
		UnkArray unkArray2B0;			  // 0x2B0
	};

	float magnitude(NiPoint3 p);
	size_t randomGenerator(size_t min, size_t max);
	void WeatherCheck();

	NiPoint3* getWindDirection();

} // namespace hdt
