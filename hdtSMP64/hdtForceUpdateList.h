#pragma once

#include "..\hdtSSEUtils\FrameworkUtils.h"

#include <string>
#include <unordered_set>

namespace hdt
{
	class ForceUpdateList
	{
		typedef struct
		{
			std::unordered_set<std::string> nodes;
			std::unordered_set<std::string> nodes_mov;
		} nodeList_t;

	public:
		static ForceUpdateList* GetSingleton();
		int isAmong(std::string node_name);
		int isAmong(hdt::IDStr node_name);

	private:
		ForceUpdateList();

		nodeList_t m_list;
	};
} // namespace hdt
