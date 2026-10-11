// Package entry point for deki-lora.
#include "DekiLoRaPackage.h"
#include <deki/interop/Plugin.h>
#include "DekiLoRa.h"
#include "SX1262LoRaComponent.h"

extern void DekiLoRaRegisterComponents();
extern int DekiLoRaGetAutoComponentCount();
extern const Deki::ComponentMeta* DekiLoRaGetAutoComponentMeta(int index);

namespace DekiLora
{

#ifdef DEKI_EDITOR

static bool s_LoRaRegistered = false;

// The exports below are C symbols at global scope; the package's own
// registration helpers and statics live in its namespace.
using namespace DekiLora;

extern "C"
{
    DEKI_LORA_API int DekiLoRaEnsureRegistered(void)
    {
        if (s_LoRaRegistered)
        {
            return ::DekiLoRaGetAutoComponentCount();
        }
        s_LoRaRegistered = true;
        ::DekiLoRaRegisterComponents();
        return ::DekiLoRaGetAutoComponentCount();
    }

    DEKI_PLUGIN_API const char* DekiPluginGetName(void)
    {
        return "Deki LoRa Package";
    }
    DEKI_PLUGIN_API const char* DekiPluginGetVersion(void)
    {
#ifdef DEKI_PACKAGE_VERSION
        return DEKI_PACKAGE_VERSION;
#else
        return "0.0.0-dev";
#endif
    }
    DEKI_PLUGIN_API int DekiPluginInit(void)
    {
        return 0;
    }
    DEKI_PLUGIN_API void DekiPluginShutdown(void)
    {
        s_LoRaRegistered = false;
        // The engine must not call into this DLL's code once it unloads.
        SX1262LoRaComponent::StopUpdates();
        DekiLoRa::SetCurrent(nullptr);
    }
    DEKI_PLUGIN_API int DekiPluginGetComponentCount(void)
    {
        return ::DekiLoRaGetAutoComponentCount();
    }
    DEKI_PLUGIN_API const Deki::ComponentMeta* DekiPluginGetComponentMeta(int index)
    {
        return ::DekiLoRaGetAutoComponentMeta(index);
    }
    DEKI_PLUGIN_API void DekiPluginRegisterComponents(void)
    {
        DekiLoRaEnsureRegistered();
    }

}  // extern "C"

#endif  // DEKI_EDITOR
}  // namespace DekiLora
