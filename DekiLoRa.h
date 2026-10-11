#pragma once

#include "ILoRaRadio.h"

namespace DekiLora
{

/// Holds the active LoRa radio.
///
/// A SetupComponent in the boot scene (SX1262LoRaComponent) registers it
/// during Setup(). Game code reaches it through GetCurrent(), which is nullptr
/// on a board without one.
class DekiLoRa
{
public:
    static void SetCurrent(ILoRaRadio* radio);
    static ILoRaRadio* GetCurrent();

private:
    static ILoRaRadio* s_Current;
};

}  // namespace DekiLora
