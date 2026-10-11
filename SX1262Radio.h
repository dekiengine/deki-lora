#pragma once

#include "ILoRaRadio.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace DekiLora
{

/// What the SX1262 driver needs from the board: its SPI device and two pins.
/// Kept apart so the driver itself has no platform code.
class ISX126xBus
{
public:
    virtual ~ISX126xBus() = default;

    /// One SPI transaction, chip select held for all `length` bytes. `rx` may
    /// be nullptr.
    virtual bool Transfer(const uint8_t* tx, uint8_t* rx, size_t length) = 0;

    /// Drives the NRESET pin.
    virtual void SetReset(bool high) = 0;

    /// The BUSY pin: high while the chip cannot take a command.
    virtual bool IsBusy() const = 0;

    virtual void DelayMs(uint32_t ms) = 0;
    virtual uint32_t NowMs() const = 0;

    /// The DIO1 pin, which the driver routes every interrupt it uses to:
    /// when it is wired, a low DIO1 means nothing happened and the chip is not
    /// asked over SPI, which may be busy (sending the display a frame).
    virtual bool HasDio1() const { return false; }
    virtual bool Dio1() const { return true; }
};

struct SX1262Config
{
    uint32_t frequencyHz = 868100000;
    int spreadingFactor = 7;
    uint32_t bandwidthHz = 125000;
    int codingRate = 5;  // 4/5
    int txPowerDbm = 14;
    int preambleLength = 8;
    uint8_t syncWord = 0x12;  // private networks; 0x34 is LoRaWAN's
    float tcxoVoltage = 1.8f;  // 0: the board has a crystal, not a TCXO on DIO3
    bool dio2AsRfSwitch = true;
    float dutyCyclePercent = 1.0f;
};

/// A Semtech SX1262 in LoRa mode: explicit header, CRC on, standard IQ.
/// Listens continuously; Update() (once a frame) collects what arrived and
/// goes back to listening after a send. Polls the chip's interrupt status
/// over SPI, so no interrupt pin is needed.
class SX1262Radio : public ILoRaRadio
{
public:
    explicit SX1262Radio(ISX126xBus& bus) : m_Bus(bus) {}

    /// Resets and sets up the chip. False, with why, when it does not answer
    /// or refuses a setting.
    bool Begin(const SX1262Config& config, std::string& error);

    /// Collects received packets and finished sends. Call once a frame.
    void Update();

    bool Send(const uint8_t* data, size_t size) override;
    bool IsSending() const override { return m_Sending; }
    uint32_t MillisecondsUntilCanSend() const override;
    uint32_t TimeOnAirMs(size_t size) const override;
    bool Receive(LoRaPacket& packet) override;
    LoRaSettings Settings() const override;
    uint32_t PacketsSent() const override { return m_Sent; }
    uint32_t PacketsReceived() const override { return m_Received; }
    uint32_t PacketsCorrupt() const override { return m_Corrupt; }

private:
    static constexpr size_t kQueueLimit = 8;

    ISX126xBus& m_Bus;
    SX1262Config m_Config;
    bool m_LowDataRateOptimize = false;

    bool m_Sending = false;
    uint32_t m_SendStartedMs = 0;
    uint32_t m_SendAirtimeMs = 0;
    uint32_t m_NextSendAllowedMs = 0;
    bool m_HaveSent = false;

    uint32_t m_Sent = 0;
    uint32_t m_Received = 0;
    uint32_t m_Corrupt = 0;
    std::deque<LoRaPacket> m_Queue;

    bool WaitWhileBusy(uint32_t timeoutMs);
    bool Command(const uint8_t* tx, size_t length, uint8_t* rx = nullptr);
    bool WriteRegister(uint16_t address, uint8_t value);
    bool ReadRegister(uint16_t address, uint8_t& value);
    bool SetPacketLength(uint8_t length);
    bool StartReceive();
    void ReadPacket();
};

}  // namespace DekiLora
