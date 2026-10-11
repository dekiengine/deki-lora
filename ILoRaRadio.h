#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace DekiLora
{

/// A packet the radio received.
struct LoRaPacket
{
    std::vector<uint8_t> data;
    float rssi = 0.0f;  // dBm
    float snr = 0.0f;   // dB
};

/// The settings a radio runs with, as it reports them.
struct LoRaSettings
{
    uint32_t frequencyHz = 0;
    int spreadingFactor = 0;   // 7 to 12
    uint32_t bandwidthHz = 0;  // 125000, 250000, 500000, ...
    int codingRate = 0;        // 5 to 8, for 4/5 to 4/8
    int txPowerDbm = 0;
    float dutyCyclePercent = 100.0f;  // 100: no limit
};

/// A LoRa radio. Listens all the time it is not sending; received packets
/// queue until read with Receive().
///
/// Sending is limited by the duty cycle the radio was set up with: after a
/// packet that took T on air, the next may go after T * (100 / duty - 1). Most
/// of Europe's 868 MHz band allows 1%, so a 50 ms packet means waiting about
/// 5 s. Send() refuses earlier, and MillisecondsUntilCanSend() says how long
/// is left.
class ILoRaRadio
{
public:
    virtual ~ILoRaRadio() = default;

    /// Starts sending `size` bytes (at most 255). False when the radio is
    /// already sending, the duty cycle does not allow it yet, or the packet
    /// is too long. Returns at once; IsSending() is true until it is done.
    virtual bool Send(const uint8_t* data, size_t size) = 0;
    virtual bool IsSending() const = 0;

    /// 0 when a packet may be sent now.
    virtual uint32_t MillisecondsUntilCanSend() const = 0;

    /// How long `size` bytes take on air with the current settings.
    virtual uint32_t TimeOnAirMs(size_t size) const = 0;

    /// Takes the oldest received packet. False when there is none.
    virtual bool Receive(LoRaPacket& packet) = 0;

    virtual LoRaSettings Settings() const = 0;

    /// Packets sent and received, and received ones dropped for a bad CRC,
    /// since start.
    virtual uint32_t PacketsSent() const = 0;
    virtual uint32_t PacketsReceived() const = 0;
    virtual uint32_t PacketsCorrupt() const = 0;
};

}  // namespace DekiLora
