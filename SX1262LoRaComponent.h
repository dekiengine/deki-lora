#pragma once

#include <cstdint>
#include <deki/SetupComponent.h>
#include <deki/reflection/Property.h>

namespace DekiLora
{

/// Boot-scene component for a Semtech SX1262 LoRa radio on SPI. Sets it up and
/// registers it as DekiLoRa::GetCurrent(); without it, or when the radio does
/// not answer, there is none.
///
/// The radio may share its SPI bus with the display and an SD card (the
/// LilyGO T-Deck's does): it joins a bus that is already up.
DEKI_CATEGORY("Radio")
DEKI_DISPLAY_NAME("SX1262 LoRa")
DEKI_DESCRIPTION("Sets up a Semtech SX1262 LoRa radio on SPI.")
class SX1262LoRaComponent : public Deki::SetupComponent
{
public:
    DEKI_GROUP("Wiring")
    DEKI_EXPORT
    DEKI_TOOLTIP("SPI controller: 1 is SPI2, 2 is SPI3. Use the display's when they share pins.")
    DEKI_RANGE(1, 2)
    int32_t spiHost = 1;

    DEKI_EXPORT
    DEKI_RANGE(-1, 48)
    int32_t sckPin = 40;

    DEKI_EXPORT
    DEKI_RANGE(-1, 48)
    int32_t mosiPin = 41;

    DEKI_EXPORT
    DEKI_RANGE(-1, 48)
    int32_t misoPin = 38;

    DEKI_EXPORT
    DEKI_TOOLTIP("The radio's chip select (NSS)")
    DEKI_RANGE(0, 48)
    int32_t csPin = 9;

    DEKI_EXPORT
    DEKI_TOOLTIP("The radio's BUSY pin")
    DEKI_RANGE(0, 48)
    int32_t busyPin = 13;

    DEKI_EXPORT
    DEKI_TOOLTIP("The radio's NRESET pin")
    DEKI_RANGE(0, 48)
    int32_t resetPin = 17;

    DEKI_EXPORT
    DEKI_TOOLTIP("SPI clock. The SX1262 takes up to 16 MHz.")
    DEKI_RANGE(100000, 16000000)
    int32_t spiHz = 8000000;

    DEKI_EXPORT
    DEKI_TOOLTIP("Volts DIO3 gives the radio's TCXO; 0 when the board has a plain crystal instead. LilyGO boards: 1.8")
    DEKI_RANGE(0, 3.3)
    float tcxoVoltage = 1.8f;

    DEKI_EXPORT
    DEKI_TOOLTIP("DIO2 switches the antenna between sending and receiving, as on most SX1262 modules")
    bool dio2AsRfSwitch = true;

    DEKI_GROUP("Radio")
    DEKI_EXPORT
    DEKI_TOOLTIP("Carrier frequency in Hz. It must be one your region allows: 868100000 in Europe, 915000000 in "
                 "the Americas.")
    DEKI_RANGE(150000000, 960000000)
    int32_t frequencyHz = 868100000;

    DEKI_EXPORT
    DEKI_TOOLTIP("7 is fastest; each step up doubles the time on air and adds range")
    DEKI_RANGE(7, 12)
    int32_t spreadingFactor = 7;

    DEKI_EXPORT
    DEKI_TOOLTIP("Bandwidth in kHz: 125, 250 or 500 (or 62.5 and below, written as 62, 41, 31, 20, 15, 10, 7)")
    DEKI_RANGE(7, 500)
    int32_t bandwidthKHz = 125;

    DEKI_EXPORT
    DEKI_TOOLTIP("Coding rate 4/5 to 4/8, as 5 to 8: more is sturdier and slower")
    DEKI_RANGE(5, 8)
    int32_t codingRate = 5;

    DEKI_EXPORT
    DEKI_TOOLTIP("Transmit power in dBm. Europe's 868.0-868.6 MHz allows 14.")
    DEKI_RANGE(-9, 22)
    int32_t txPowerDbm = 14;

    DEKI_EXPORT
    DEKI_TOOLTIP("Sync word: radios only hear others with the same one. 0x12 (18) private, 0x34 (52) LoRaWAN.")
    DEKI_RANGE(0, 255)
    int32_t syncWord = 0x12;

    DEKI_EXPORT
    DEKI_TOOLTIP("Most of the time the radio may be sending, in percent: after a packet it waits until that holds. "
                 "Europe's 868.0-868.6 MHz allows 1. 100 turns the limit off, where the law allows it.")
    DEKI_RANGE(0.1, 100)
    float dutyCyclePercent = 1.0f;

    void Setup(SetupCallback onComplete) override;
    const char* GetSetupName() const override { return "SX1262 LoRa"; }

    /// Stops updating the radio every frame. Called before the package's DLL
    /// unloads, so the engine does not call into unloaded code.
    static void StopUpdates();
};

}  // namespace DekiLora
