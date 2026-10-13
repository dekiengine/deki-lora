// SX1262Radio against a fake chip that answers the datasheet's SPI commands:
// what setup sends (frequency, sync word, power, the IQ errata), time on air
// against Semtech's figures, the duty-cycle limit, sending, receiving with
// signal quality, corrupt packets, the receive queue's limit, a send that
// never finishes, and a missing or stuck chip.
//
// The driver takes its SPI, pins and clock through ISX126xBus, so it runs
// here unchanged, on a clock the test moves.

#include <gtest/gtest.h>

#include "SX1262Radio.h"

#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

using namespace DekiLora;

namespace
{

constexpr uint16_t kIrqTxDone = 0x0001;
constexpr uint16_t kIrqRxDone = 0x0002;
constexpr uint16_t kIrqCrcError = 0x0040;

// An SX1262 as far as the driver talks to it.
class FakeChip : public ISX126xBus
{
public:
    std::vector<std::vector<uint8_t>> commands;  // every transaction's bytes
    std::map<uint16_t, uint8_t> registers;
    uint16_t irq = 0;
    std::vector<uint8_t> rxBuffer;  // what the next received packet holds
    uint8_t rssiRaw = 0;            // -RSSI * 2
    int8_t snrRaw = 0;              // SNR * 4
    std::vector<uint8_t> txBuffer;  // what the driver wrote to send
    uint8_t status = 0x20;          // chip mode 2: STDBY_RC
    bool busy = false;
    bool hasDio1 = false;
    bool dio1 = true;
    uint32_t now = 1000;

    bool Transfer(const uint8_t* tx, uint8_t* rx, size_t length) override
    {
        commands.emplace_back(tx, tx + length);
        std::vector<uint8_t> out(length, 0);
        switch (tx[0])
        {
            case 0xC0: out[1] = status; break;  // GetStatus
            case 0x0D: registers[uint16_t(tx[1] << 8 | tx[2])] = tx[3]; break;
            case 0x1D: out[4] = registers[uint16_t(tx[1] << 8 | tx[2])]; break;
            case 0x12:  // GetIrqStatus
                out[2] = uint8_t(irq >> 8);
                out[3] = uint8_t(irq);
                break;
            case 0x02: irq &= ~uint16_t(tx[1] << 8 | tx[2]); break;  // ClearIrqStatus
            case 0x13:  // GetRxBufferStatus
                out[2] = uint8_t(rxBuffer.size());
                out[3] = 0;
                break;
            case 0x1E:  // ReadBuffer
                for (size_t i = 3; i < length && i - 3 < rxBuffer.size(); ++i)
                {
                    out[i] = rxBuffer[i - 3];
                }
                break;
            case 0x14:  // GetPacketStatus
                out[2] = rssiRaw;
                out[3] = uint8_t(snrRaw);
                break;
            case 0x0E: txBuffer.assign(tx + 2, tx + length); break;  // WriteBuffer
            default: break;
        }
        if (rx)
        {
            std::memcpy(rx, out.data(), length);
        }
        return true;
    }
    void SetReset(bool) override {}
    bool IsBusy() const override
    {
        // The driver spins on BUSY reading the clock; a stuck chip has to let
        // time pass or the wait never ends.
        if (busy)
        {
            const_cast<FakeChip*>(this)->now += 10;
        }
        return busy;
    }
    void DelayMs(uint32_t ms) override { now += ms; }
    uint32_t NowMs() const override { return now; }
    bool HasDio1() const override { return hasDio1; }
    bool Dio1() const override { return dio1; }

    // The last command starting with `opcode`, or empty.
    std::vector<uint8_t> Last(uint8_t opcode) const
    {
        for (auto it = commands.rbegin(); it != commands.rend(); ++it)
        {
            if (!it->empty() && (*it)[0] == opcode)
            {
                return *it;
            }
        }
        return {};
    }
    size_t Count(uint8_t opcode) const
    {
        size_t n = 0;
        for (const auto& c : commands)
        {
            n += (!c.empty() && c[0] == opcode) ? 1 : 0;
        }
        return n;
    }
};

class SX1262Test : public ::testing::Test
{
protected:
    void Start(SX1262Config config = {})
    {
        std::string error;
        ASSERT_TRUE(radio.Begin(config, error)) << error;
    }

    // A packet arrives: the chip raises RxDone and the driver reads it.
    void Arrive(const std::vector<uint8_t>& bytes, uint16_t extraIrq = 0)
    {
        chip.rxBuffer = bytes;
        chip.irq |= kIrqRxDone | extraIrq;
        radio.Update();
    }

    FakeChip chip;
    SX1262Radio radio{ chip };
};

using Bytes = std::vector<uint8_t>;

}  // namespace

// --- Setup ---

TEST_F(SX1262Test, SetupProgramsTheFrequency)
{
    // frf = f * 2^25 / 32 MHz, most significant byte first.
    Start();
    EXPECT_EQ(chip.Last(0x86), (Bytes{ 0x86, 0x36, 0x41, 0x99, 0x99 }));  // 868.1 MHz
    EXPECT_EQ(chip.Last(0x98), (Bytes{ 0x98, 0xD7, 0xDB }));              // 863-870 MHz image

    FakeChip us;
    SX1262Radio usRadio(us);
    SX1262Config config;
    config.frequencyHz = 915000000;
    std::string error;
    ASSERT_TRUE(usRadio.Begin(config, error)) << error;
    EXPECT_EQ(us.Last(0x86), (Bytes{ 0x86, 0x39, 0x30, 0x00, 0x00 }));
    EXPECT_EQ(us.Last(0x98), (Bytes{ 0x98, 0xE1, 0xE9 }));  // 902-928 MHz image
}

TEST_F(SX1262Test, TheSyncWordIsSplitAcrossTwoRegisters)
{
    // 0x12 (private networks) is stored as 0x14 0x24, LoRaWAN's 0x34 as
    // 0x34 0x44: a radio with the wrong pair hears nobody.
    Start();
    EXPECT_EQ(chip.registers[0x0740], 0x14);
    EXPECT_EQ(chip.registers[0x0741], 0x24);

    FakeChip lorawan;
    SX1262Radio r(lorawan);
    SX1262Config config;
    config.syncWord = 0x34;
    std::string error;
    ASSERT_TRUE(r.Begin(config, error));
    EXPECT_EQ(lorawan.registers[0x0740], 0x34);
    EXPECT_EQ(lorawan.registers[0x0741], 0x44);
}

TEST_F(SX1262Test, TheIqErrataBitIsSet)
{
    chip.registers[0x0736] = 0x0B;
    Start();
    EXPECT_EQ(chip.registers[0x0736], 0x0F);  // bit 2 set, the rest kept
}

TEST_F(SX1262Test, PowerUsesTheDatasheetsPaTable)
{
    // 14 dBm: the table's 14 dBm row at power 22. 10 dBm: the same row, 4 dB
    // lower. 22 dBm: the top row.
    struct Case
    {
        int dbm;
        Bytes pa;
        uint8_t power;
    };
    for (const Case& c : { Case{ 14, { 0x95, 0x02, 0x02, 0x00, 0x01 }, 22 },
                           Case{ 10, { 0x95, 0x02, 0x02, 0x00, 0x01 }, 18 },
                           Case{ 22, { 0x95, 0x04, 0x07, 0x00, 0x01 }, 22 },
                           Case{ 30, { 0x95, 0x04, 0x07, 0x00, 0x01 }, 22 } })
    {
        FakeChip c2;
        SX1262Radio r(c2);
        SX1262Config config;
        config.txPowerDbm = c.dbm;
        std::string error;
        ASSERT_TRUE(r.Begin(config, error));
        EXPECT_EQ(c2.Last(0x95), c.pa) << c.dbm << " dBm";
        EXPECT_EQ(c2.Last(0x8E)[1], c.power) << c.dbm << " dBm";
    }
}

TEST_F(SX1262Test, SetupEndsListening)
{
    Start();
    EXPECT_EQ(chip.commands.back(), (Bytes{ 0x82, 0xFF, 0xFF, 0xFF }));  // continuous receive
}

TEST_F(SX1262Test, ImpossibleSettingsAreRefusedBeforeTheChipIsTouched)
{
    SX1262Config bw;
    bw.bandwidthHz = 100000;
    std::string error;
    EXPECT_FALSE(radio.Begin(bw, error));
    EXPECT_NE(error, "");

    SX1262Config sf;
    sf.spreadingFactor = 13;
    EXPECT_FALSE(radio.Begin(sf, error));
    SX1262Config cr;
    cr.codingRate = 4;
    EXPECT_FALSE(radio.Begin(cr, error));
    EXPECT_TRUE(chip.commands.empty());
}

TEST_F(SX1262Test, AMissingChipIsReported)
{
    chip.status = 0x00;  // nothing drives MISO
    SX1262Config config;
    std::string error;
    EXPECT_FALSE(radio.Begin(config, error));
    EXPECT_NE(error.find("no SX1262"), std::string::npos) << error;
}

TEST_F(SX1262Test, AChipStuckBusyIsReported)
{
    chip.busy = true;
    SX1262Config config;
    std::string error;
    EXPECT_FALSE(radio.Begin(config, error));
    EXPECT_NE(error.find("BUSY"), std::string::npos) << error;
}

// --- Time on air ---

TEST_F(SX1262Test, TimeOnAirMatchesSemtechsFormula)
{
    // Semtech's calculator: SF7, 125 kHz, 4/5, 8 symbols of preamble,
    // explicit header and CRC: 10 bytes take 41.22 ms, 255 bytes 399.62 ms.
    Start();
    EXPECT_EQ(radio.TimeOnAirMs(10), 42u);
    EXPECT_EQ(radio.TimeOnAirMs(255), 400u);
}

TEST_F(SX1262Test, SlowSettingsTurnOnLowDataRateOptimisation)
{
    // SF12 at 125 kHz: a symbol takes 32.8 ms, past the 16 ms that requires
    // it, and 10 bytes take 991.23 ms.
    SX1262Config config;
    config.spreadingFactor = 12;
    Start(config);
    EXPECT_EQ(chip.Last(0x8B), (Bytes{ 0x8B, 12, 0x04, 1, 1 }));
    EXPECT_EQ(radio.TimeOnAirMs(10), 992u);
}

// --- Sending and the duty cycle ---

TEST_F(SX1262Test, SendWritesThePacketAndTransmits)
{
    Start();
    const uint8_t data[] = { 'h', 'i', 0, 255 };
    ASSERT_TRUE(radio.Send(data, sizeof(data)));
    EXPECT_TRUE(radio.IsSending());
    EXPECT_EQ(chip.txBuffer, (Bytes{ 'h', 'i', 0, 255 }));
    EXPECT_EQ(chip.Last(0x8C)[4], 4);  // packet length
    EXPECT_EQ(chip.commands.back(), (Bytes{ 0x83, 0x00, 0x00, 0x00 }));

    chip.irq = kIrqTxDone;
    radio.Update();
    EXPECT_FALSE(radio.IsSending());
    EXPECT_EQ(radio.PacketsSent(), 1u);
    EXPECT_EQ(chip.commands.back(), (Bytes{ 0x82, 0xFF, 0xFF, 0xFF }));  // listening again
}

TEST_F(SX1262Test, TheDutyCycleHoldsTheNextSend)
{
    // 1%: 42 ms on air, then 4158 ms of silence; 4200 ms from the start.
    Start();
    EXPECT_EQ(radio.MillisecondsUntilCanSend(), 0u);
    const uint8_t data[10] = {};
    ASSERT_TRUE(radio.Send(data, 10));
    EXPECT_FALSE(radio.Send(data, 10));  // still on air

    chip.now += 42;
    chip.irq = kIrqTxDone;
    radio.Update();
    EXPECT_EQ(radio.MillisecondsUntilCanSend(), 4158u);
    EXPECT_FALSE(radio.Send(data, 10));

    chip.now += 4157;
    EXPECT_EQ(radio.MillisecondsUntilCanSend(), 1u);
    chip.now += 1;
    EXPECT_EQ(radio.MillisecondsUntilCanSend(), 0u);
    EXPECT_TRUE(radio.Send(data, 10));
}

TEST_F(SX1262Test, NoDutyCycleLimitMeansOnlyTheAirtime)
{
    SX1262Config config;
    config.dutyCyclePercent = 100.0f;
    Start(config);
    const uint8_t data[10] = {};
    ASSERT_TRUE(radio.Send(data, 10));
    chip.now += 42;
    chip.irq = kIrqTxDone;
    radio.Update();
    EXPECT_EQ(radio.MillisecondsUntilCanSend(), 0u);
}

TEST_F(SX1262Test, PacketsOutsideOneTo255BytesAreRefused)
{
    Start();
    uint8_t big[256] = {};
    EXPECT_FALSE(radio.Send(big, 256));
    EXPECT_FALSE(radio.Send(big, 0));
    EXPECT_FALSE(radio.Send(nullptr, 4));
    EXPECT_TRUE(radio.Send(big, 255));
}

TEST_F(SX1262Test, ASendThatNeverFinishesGivesUp)
{
    // No TxDone: after twice the airtime and half a second the driver goes
    // back to listening instead of sending forever.
    Start();
    const uint8_t data[10] = {};
    ASSERT_TRUE(radio.Send(data, 10));
    chip.now += 42 * 2 + 400;
    radio.Update();
    EXPECT_TRUE(radio.IsSending());
    chip.now += 200;
    radio.Update();
    EXPECT_FALSE(radio.IsSending());
    EXPECT_EQ(radio.PacketsSent(), 0u);
    EXPECT_EQ(chip.commands.back(), (Bytes{ 0x82, 0xFF, 0xFF, 0xFF }));
}

// --- Receiving ---

TEST_F(SX1262Test, AReceivedPacketCarriesItsSignalQuality)
{
    Start();
    chip.rssiRaw = 170;  // -85 dBm
    chip.snrRaw = -22;   // -5.5 dB
    Arrive({ 1, 2, 3 });
    LoRaPacket p;
    ASSERT_TRUE(radio.Receive(p));
    EXPECT_EQ(p.data, (Bytes{ 1, 2, 3 }));
    EXPECT_FLOAT_EQ(p.rssi, -85.0f);
    EXPECT_FLOAT_EQ(p.snr, -5.5f);
    EXPECT_EQ(radio.PacketsReceived(), 1u);
    EXPECT_EQ(chip.irq, 0);  // the interrupt was cleared
    EXPECT_FALSE(radio.Receive(p));
}

TEST_F(SX1262Test, ACorruptPacketIsCountedNotQueued)
{
    Start();
    Arrive({ 9, 9 }, kIrqCrcError);
    LoRaPacket p;
    EXPECT_FALSE(radio.Receive(p));
    EXPECT_EQ(radio.PacketsCorrupt(), 1u);
    EXPECT_EQ(radio.PacketsReceived(), 0u);
}

TEST_F(SX1262Test, TheQueueKeepsTheNewestEight)
{
    Start();
    for (uint8_t i = 0; i < 10; ++i)
    {
        Arrive({ i });
    }
    EXPECT_EQ(radio.PacketsReceived(), 10u);
    LoRaPacket p;
    for (uint8_t want = 2; want < 10; ++want)
    {
        ASSERT_TRUE(radio.Receive(p));
        EXPECT_EQ(p.data, Bytes{ want });
    }
    EXPECT_FALSE(radio.Receive(p));
}

TEST_F(SX1262Test, WithDio1LowTheBusIsLeftAlone)
{
    // DIO1 wired and low: no interrupt, so no SPI, which may be busy with
    // the display.
    chip.hasDio1 = true;
    Start();
    chip.dio1 = false;
    const size_t before = chip.commands.size();
    radio.Update();
    EXPECT_EQ(chip.commands.size(), before);

    chip.dio1 = true;
    Arrive({ 7 });
    EXPECT_EQ(radio.PacketsReceived(), 1u);
}

TEST_F(SX1262Test, SettingsReportWhatTheRadioRuns)
{
    SX1262Config config;
    config.txPowerDbm = 40;
    config.dutyCyclePercent = 10.0f;
    Start(config);
    const LoRaSettings s = radio.Settings();
    EXPECT_EQ(s.frequencyHz, 868100000u);
    EXPECT_EQ(s.spreadingFactor, 7);
    EXPECT_EQ(s.bandwidthHz, 125000u);
    EXPECT_EQ(s.codingRate, 5);
    EXPECT_EQ(s.txPowerDbm, 22);  // clamped to what the chip can do
    EXPECT_FLOAT_EQ(s.dutyCyclePercent, 10.0f);
}
