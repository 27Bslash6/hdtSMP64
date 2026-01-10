#include "PluginInterfaceImpl.h"

hdt::PluginInterfaceImpl hdt::g_pluginInterface;

void hdt::PluginInterfaceImpl::addListener(IPreStepListener* l)
{
	if (l) {
		m_preStepDispatcher.addListener(l);
	}
}

void hdt::PluginInterfaceImpl::removeListener(IPreStepListener* l)
{
	if (l) {
		m_preStepDispatcher.removeListener(l);
	}
}

void hdt::PluginInterfaceImpl::addListener(IPostStepListener* l)
{
	if (l) {
		m_postStepDispatcher.addListener(l);
	}
}

void hdt::PluginInterfaceImpl::removeListener(IPostStepListener* l)
{
	if (l) {
		m_postStepDispatcher.removeListener(l);
	}
}

void hdt::PluginInterfaceImpl::onPostPostLoad()
{
	// Send ourselves to any plugin that registered during the PostLoad event
	if (m_skseMessagingInterface) {
		m_skseMessagingInterface->Dispatch(m_sksePluginHandle, PluginInterface::MSG_STARTUP,
										   static_cast<PluginInterface*>(this), 0, nullptr);
	}
}

void hdt::PluginInterfaceImpl::init(const SKSEInterface* skse)
{
	// We need to have our SKSE plugin handle and the messaging interface in order to reach our plugins later
	if (skse) {
		m_sksePluginHandle = skse->GetPluginHandle();
		m_skseMessagingInterface =
			reinterpret_cast<SKSEMessagingInterface*>(skse->QueryInterface(kInterface_Messaging));
	}
	if (!m_skseMessagingInterface) {
		_WARNING("Failed to get a messaging interface. Plugins will not work.");
	}
}
