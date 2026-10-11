#include "SX1262Radio.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace DekiLora
{

namespace
{
// Commands (SX1261/2 datasheet, chapter 13).
constexpr uint8_t kSetStandby = 0x80;
constexpr uint8_t kSetRx = 0x82;
constexpr uint8_t kSetTx = 0x83;
constexpr uint8_t kSetRfFrequency = 0x86;
constexpr uint8_t kSetPacketType = 0x8A;
constexpr uint8_t kSetModulationParams = 0x8B;
constexpr uint8_t kSetPacketParams = 0x8C;
constexpr uint8_t kSetTxParams = 0x8E;
constexpr uint8_t kSetBufferBaseAddress = 0x8F;
constexpr uint8_t kSetPaConfig = 0x95;
constexpr uint8_t kSetRegulatorMode = 0x96;
constexpr uint8_t kSetDio3AsTcxoCtrl = 0x97;
constexpr uint8_t kCalibrateImage = 0x98;
constexpr uint8_t kSetDio2AsRfSwitchCtrl = 0x9D;
constexpr uint8_t kCalibrate = 0x89;
constexpr uint8_t kSetDioIrqParams = 0x08;
constexpr uint8_t kClearIrqStatus = 0x02;
constexpr uint8_t kGetIrqStatus = 0x12;
constexpr uint8_t kGetRxBufferStatus = 0x13;
constexpr uint8_t kGetPacketStatus = 0x14;
constexpr uint8_t kGetStatus = 0xC0;
constexpr uint8_t kWriteRegister = 0x0D;
constexpr uint8_t kReadRegister = 0x1D;
constexpr uint8_t kWriteBuffer = 0x0E;
constexpr uint8_t kReadBuffer = 0x1E;

constexpr uint16_t kRegSyncWord = 0x0740;
constexpr uint16_t kRegIqPolarity = 0x0736;

constexpr uint16_t kIrqTxDone = 0x0001;
constexpr uint16_t kIrqRxDone = 0x0002;
constexpr uint16_t kIrqHeaderError = 0x0020;
constexpr uint16_t kIrqCrcError = 0x0040;
constexpr uint16_t kIrqTimeout = 0x0200;
constexpr uint16_t kIrqUsed = kIrqTxDone | kIrqRxDone | kIrqHeaderError | kIrqCrcError | kIrqTimeout;

// The LoRa bandwidth codes, by bandwidth in Hz.
bool BandwidthCode(uint32_t hz, uint8_t& code)
{
    struct Entry
    {
        uint32_t hz;
        uint8_t code;
    };
    static const Entry kTable[] = { { 7810, 0x00 },   { 10420, 0x08 },  { 15630, 0x01 },
                                    { 20830, 0x09 },  { 31250, 0x02 },  { 41670, 0x0A },
                                    { 62500, 0x03 },  { 125000, 0x04 }, { 250000, 0x05 },
                                    { 500000, 0x06 } };
    for (const Entry& e : kTable)
    {
        // The odd ones are written as their kHz with a decimal (7.8, 10.4).
        if (hz == e.hz || (hz > e.hz - 100 && hz < e.hz + 100))
        {
            code = e.code;
            return true;
        }
    }
    return false;
}

// DIO3's TCXO supply codes, by voltage.
uint8_t TcxoCode(float volts)
{
    static const float kVolts[] = { 1.6f, 1.7f, 1.8f, 2.2f, 2.4f, 2.7f, 3.0f, 3.3f };
    uint8_t best = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (std::fabs(kVolts[i] - volts) < std::fabs(kVolts[best] - volts))
        {
            best = i;
        }
    }
    return best;
}
}  // namespace

bool SX1262Radio::WaitWhileBusy(uint32_t timeoutMs)
{
    const uint32_t start = m_Bus.NowMs();
    while (m_Bus.IsBusy())
    {
        if (m_Bus.NowMs() - start > timeoutMs)
        {
            return false;
        }
    }
    return true;
}

bool SX1262Radio::Command(const uint8_t* tx, size_t length, uint8_t* rx)
{
    if (!WaitWhileBusy(100))
    {
        return false;
    }
    return m_Bus.Transfer(tx, rx, length);
}

bool SX1262Radio::WriteRegister(uint16_t address, uint8_t value)
{
    const uint8_t tx[] = { kWriteRegister, static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address), value };
    return Command(tx, sizeof(tx));
}

bool SX1262Radio::ReadRegister(uint16_t address, uint8_t& value)
{
    const uint8_t tx[] = { kReadRegister, static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address), 0, 0 };
    uint8_t rx[sizeof(tx)] = {};
    if (!Command(tx, sizeof(tx), rx))
    {
        return false;
    }
    value = rx[4];
    return true;
}

bool SX1262Radio::SetPacketLength(uint8_t length)
{
    const uint8_t tx[] = { kSetPacketParams,
                           static_cast<uint8_t>(m_Config.preambleLength >> 8),
                           static_cast<uint8_t>(m_Config.preambleLength),
                           0x00,  // explicit header
                           length,
                           0x01,  // CRC on
                           0x00 };  // standard IQ
    return Command(tx, sizeof(tx));
}

bool SX1262Radio::StartReceive()
{
    if (!SetPacketLength(255))
    {
        return false;
    }
    const uint8_t rx[] = { kSetRx, 0xFF, 0xFF, 0xFF };  // continuous
    return Command(rx, sizeof(rx));
}

bool SX1262Radio::Begin(const SX1262Config& config, std::string& error)
{
    m_Config = config;
    uint8_t bandwidth = 0;
    if (!BandwidthCode(config.bandwidthHz, bandwidth))
    {
        error = "unsupported LoRa bandwidth " + std::to_string(config.bandwidthHz) + " Hz";
        return false;
    }
    if (config.spreadingFactor < 5 || config.spreadingFactor > 12 || config.codingRate < 5 || config.codingRate > 8)
    {
        error = "spreading factor must be 5-12 and coding rate 5-8 (4/5-4/8)";
        return false;
    }

    m_Bus.SetReset(false);
    m_Bus.DelayMs(2);
    m_Bus.SetReset(true);
    m_Bus.DelayMs(10);
    if (!WaitWhileBusy(200))
    {
        error = "the SX1262 does not answer: BUSY stays high after reset (wrong pins, or no radio)";
        return false;
    }

    const uint8_t standby[] = { kSetStandby, 0x00 };  // STDBY_RC
    Command(standby, sizeof(standby));

    // GetStatus: chip mode in bits 6:4, STDBY_RC is 2. A missing chip reads
    // as 0x00 or 0xFF.
    const uint8_t getStatus[] = { kGetStatus, 0x00 };
    uint8_t status[2] = {};
    if (!Command(getStatus, sizeof(getStatus), status) || ((status[1] >> 4) & 0x07) != 0x02)
    {
        error = "no SX1262 answers on SPI (status " + std::to_string(status[1]) + ")";
        return false;
    }

    if (config.tcxoVoltage > 0.0f)
    {
        // 5 ms for the TCXO to settle, in 15.625 us steps.
        const uint8_t tcxo[] = { kSetDio3AsTcxoCtrl, TcxoCode(config.tcxoVoltage), 0x00, 0x01, 0x40 };
        Command(tcxo, sizeof(tcxo));
    }
    const uint8_t calibrate[] = { kCalibrate, 0x7F };
    Command(calibrate, sizeof(calibrate));
    m_Bus.DelayMs(5);
    if (!WaitWhileBusy(100))
    {
        error = "the SX1262 did not finish calibrating";
        return false;
    }

    if (config.dio2AsRfSwitch)
    {
        const uint8_t rfSwitch[] = { kSetDio2AsRfSwitchCtrl, 0x01 };
        Command(rfSwitch, sizeof(rfSwitch));
    }
    const uint8_t regulator[] = { kSetRegulatorMode, 0x01 };  // DC-DC
    Command(regulator, sizeof(regulator));
    const uint8_t loraPackets[] = { kSetPacketType, 0x01 };
    Command(loraPackets, sizeof(loraPackets));

    const uint32_t frf = static_cast<uint32_t>((static_cast<double>(config.frequencyHz) * (1u << 25)) / 32000000.0);
    const uint8_t frequency[] = { kSetRfFrequency, static_cast<uint8_t>(frf >> 24), static_cast<uint8_t>(frf >> 16),
                                  static_cast<uint8_t>(frf >> 8), static_cast<uint8_t>(frf) };

    // Image calibration for the band the frequency is in.
    const uint32_t mhz = config.frequencyHz / 1000000;
    uint8_t image[2] = { 0xD7, 0xDB };  // 863-870
    if (mhz >= 902)
    {
        image[0] = 0xE1;
        image[1] = 0xE9;
    }
    else if (mhz >= 779 && mhz < 863)
    {
        image[0] = 0xC1;
        image[1] = 0xC5;
    }
    else if (mhz >= 470 && mhz < 779)
    {
        image[0] = 0x75;
        image[1] = 0x81;
    }
    else if (mhz < 470)
    {
        image[0] = 0x6B;
        image[1] = 0x6F;
    }
    const uint8_t calibrateImage[] = { kCalibrateImage, image[0], image[1] };
    Command(calibrateImage, sizeof(calibrateImage));
    Command(frequency, sizeof(frequency));

    // PA settings from the datasheet's table of optimal ones: the table's row
    // at or above the power asked for, with the TX power lowered by the
    // difference. Its rows all set TX power 22.
    const int dbm = std::clamp(config.txPowerDbm, -9, 22);
    struct PaRow
    {
        int dbm;
        uint8_t dutyCycle;
        uint8_t hpMax;
    };
    static const PaRow kPa[] = { { 14, 0x02, 0x02 }, { 17, 0x02, 0x03 }, { 20, 0x03, 0x05 }, { 22, 0x04, 0x07 } };
    const PaRow* row = &kPa[3];
    for (const PaRow& r : kPa)
    {
        if (r.dbm >= dbm)
        {
            row = &r;
            break;
        }
    }
    const uint8_t pa[] = { kSetPaConfig, row->dutyCycle, row->hpMax, 0x00, 0x01 };
    Command(pa, sizeof(pa));
    const int power = 22 - (row->dbm - dbm);
    const uint8_t txParams[] = { kSetTxParams, static_cast<uint8_t>(static_cast<int8_t>(power)), 0x04 };  // 200 us ramp
    Command(txParams, sizeof(txParams));

    const uint8_t bufferBase[] = { kSetBufferBaseAddress, 0x00, 0x00 };
    Command(bufferBase, sizeof(bufferBase));

    // Low data rate optimisation is required once a symbol lasts 16 ms.
    const double symbolMs = std::ldexp(1.0, config.spreadingFactor) * 1000.0 / config.bandwidthHz;
    m_LowDataRateOptimize = symbolMs >= 16.0;
    const uint8_t modulation[] = { kSetModulationParams, static_cast<uint8_t>(config.spreadingFactor), bandwidth,
                                   static_cast<uint8_t>(config.codingRate - 4),
                                   static_cast<uint8_t>(m_LowDataRateOptimize ? 1 : 0) };
    Command(modulation, sizeof(modulation));

    // The one-byte LoRa sync word, as the chip stores it in two registers.
    WriteRegister(kRegSyncWord, static_cast<uint8_t>((config.syncWord & 0xF0) | 0x04));
    WriteRegister(kRegSyncWord + 1, static_cast<uint8_t>(((config.syncWord & 0x0F) << 4) | 0x04));

    // Errata 15.4: with standard IQ, bit 2 of this register must be set, or
    // packets are lost.
    uint8_t iq = 0;
    if (ReadRegister(kRegIqPolarity, iq))
    {
        WriteRegister(kRegIqPolarity, static_cast<uint8_t>(iq | 0x04));
    }

    const uint8_t irq[] = { kSetDioIrqParams,
                            static_cast<uint8_t>(kIrqUsed >> 8),
                            static_cast<uint8_t>(kIrqUsed),
                            static_cast<uint8_t>(kIrqUsed >> 8),
                            static_cast<uint8_t>(kIrqUsed),
                            0,
                            0,
                            0,
                            0 };
    Command(irq, sizeof(irq));

    if (!StartReceive())
    {
        error = "the SX1262 would not start listening";
        return false;
    }
    return true;
}

uint32_t SX1262Radio::TimeOnAirMs(size_t size) const
{
    // Semtech's formula (SX1262 datasheet 6.1.4): explicit header, CRC on.
    const int sf = m_Config.spreadingFactor;
    const double symbolMs = std::ldexp(1.0, sf) * 1000.0 / m_Config.bandwidthHz;
    const int de = m_LowDataRateOptimize ? 1 : 0;
    const double bits = 8.0 * static_cast<double>(size) - 4.0 * sf + 28.0 + 16.0;
    const double perBlock = 4.0 * (sf - 2 * de);
    const double payloadSymbols = 8.0 + std::max(std::ceil(bits / perBlock) * m_Config.codingRate, 0.0);
    const double preambleSymbols = m_Config.preambleLength + 4.25;
    return static_cast<uint32_t>(std::ceil((preambleSymbols + payloadSymbols) * symbolMs));
}

uint32_t SX1262Radio::MillisecondsUntilCanSend() const
{
    if (m_Sending)
    {
        const uint32_t sinceStart = m_Bus.NowMs() - m_SendStartedMs;
        return sinceStart < m_SendAirtimeMs ? m_SendAirtimeMs - sinceStart : 1;
    }
    if (!m_HaveSent)
    {
        return 0;
    }
    const int32_t left = static_cast<int32_t>(m_NextSendAllowedMs - m_Bus.NowMs());
    return left > 0 ? static_cast<uint32_t>(left) : 0;
}

bool SX1262Radio::Send(const uint8_t* data, size_t size)
{
    if (!data || size == 0 || size > 255 || MillisecondsUntilCanSend() > 0)
    {
        return false;
    }

    const uint8_t standby[] = { kSetStandby, 0x00 };
    if (!Command(standby, sizeof(standby)) || !SetPacketLength(static_cast<uint8_t>(size)))
    {
        return false;
    }
    uint8_t write[2 + 255];
    write[0] = kWriteBuffer;
    write[1] = 0x00;
    std::memcpy(write + 2, data, size);
    if (!Command(write, 2 + size))
    {
        return false;
    }
    const uint8_t tx[] = { kSetTx, 0x00, 0x00, 0x00 };  // no timeout
    if (!Command(tx, sizeof(tx)))
    {
        return false;
    }

    // The duty cycle counts from the start: a packet of T on air at d% is
    // followed by T * (100 / d - 1) of silence, T * 100 / d in all.
    const uint32_t now = m_Bus.NowMs();
    m_SendAirtimeMs = TimeOnAirMs(size);
    const float duty = std::clamp(m_Config.dutyCyclePercent, 0.01f, 100.0f);
    m_NextSendAllowedMs = now + static_cast<uint32_t>(std::ceil(m_SendAirtimeMs * 100.0f / duty));
    m_SendStartedMs = now;
    m_Sending = true;
    m_HaveSent = true;
    return true;
}

void SX1262Radio::ReadPacket()
{
    const uint8_t bufferStatus[] = { kGetRxBufferStatus, 0, 0, 0 };
    uint8_t status[sizeof(bufferStatus)] = {};
    if (!Command(bufferStatus, sizeof(bufferStatus), status))
    {
        return;
    }
    const uint8_t length = status[2];
    const uint8_t start = status[3];

    uint8_t read[3 + 255] = { kReadBuffer, start, 0 };
    uint8_t data[3 + 255] = {};
    if (!Command(read, 3u + length, data))
    {
        return;
    }

    const uint8_t packetStatus[] = { kGetPacketStatus, 0, 0, 0, 0 };
    uint8_t quality[sizeof(packetStatus)] = {};
    Command(packetStatus, sizeof(packetStatus), quality);

    LoRaPacket packet;
    packet.data.assign(data + 3, data + 3 + length);
    packet.rssi = -static_cast<float>(quality[2]) / 2.0f;
    packet.snr = static_cast<float>(static_cast<int8_t>(quality[3])) / 4.0f;
    if (m_Queue.size() >= kQueueLimit)
    {
        m_Queue.pop_front();
    }
    m_Queue.push_back(std::move(packet));
    ++m_Received;
}

void SX1262Radio::Update()
{
    // DIO1 low: no interrupt, so nothing to read, and the bus is left alone.
    // Only a send that never finished needs the chip then.
    if (m_Bus.HasDio1() && !m_Bus.Dio1() &&
        !(m_Sending && m_Bus.NowMs() - m_SendStartedMs > m_SendAirtimeMs * 2 + 500))
    {
        return;
    }

    const uint8_t getIrq[] = { kGetIrqStatus, 0, 0, 0 };
    uint8_t rx[sizeof(getIrq)] = {};
    if (!Command(getIrq, sizeof(getIrq), rx))
    {
        return;
    }
    const uint16_t irq = static_cast<uint16_t>((rx[2] << 8) | rx[3]) & kIrqUsed;

    if (irq)
    {
        const uint8_t clear[] = { kClearIrqStatus, static_cast<uint8_t>(irq >> 8), static_cast<uint8_t>(irq) };
        Command(clear, sizeof(clear));
    }

    if (irq & kIrqRxDone)
    {
        if (irq & (kIrqCrcError | kIrqHeaderError))
        {
            ++m_Corrupt;
        }
        else
        {
            ReadPacket();
        }
    }

    if (irq & kIrqTxDone)
    {
        m_Sending = false;
        ++m_Sent;
        StartReceive();
    }
    else if (m_Sending && m_Bus.NowMs() - m_SendStartedMs > m_SendAirtimeMs * 2 + 500)
    {
        // TxDone never came: back to listening rather than stuck sending.
        m_Sending = false;
        StartReceive();
    }
    else if (irq & kIrqTimeout)
    {
        StartReceive();
    }
}

bool SX1262Radio::Receive(LoRaPacket& packet)
{
    if (m_Queue.empty())
    {
        return false;
    }
    packet = std::move(m_Queue.front());
    m_Queue.pop_front();
    return true;
}

LoRaSettings SX1262Radio::Settings() const
{
    LoRaSettings s;
    s.frequencyHz = m_Config.frequencyHz;
    s.spreadingFactor = m_Config.spreadingFactor;
    s.bandwidthHz = m_Config.bandwidthHz;
    s.codingRate = m_Config.codingRate;
    s.txPowerDbm = std::clamp(m_Config.txPowerDbm, -9, 22);
    s.dutyCyclePercent = m_Config.dutyCyclePercent;
    return s;
}

}  // namespace DekiLora
