#include "DekiLoRa.h"

namespace DekiLora
{

ILoRaRadio* DekiLoRa::s_Current = nullptr;

void DekiLoRa::SetCurrent(ILoRaRadio* radio)
{
    s_Current = radio;
}

ILoRaRadio* DekiLoRa::GetCurrent()
{
    return s_Current;
}

}  // namespace DekiLora
