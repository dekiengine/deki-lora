#include "SX1262LoRaComponent.h"
#include "DekiLoRa.h"
#include "SX1262Radio.h"

#include <deki/Engine.h>
#include <deki/LogSystem.h>

#include <cstddef>
#include <string>

#if defined(ESP32)
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace DekiLora
{

namespace
{

constexpr size_t kNoUpdate = static_cast<size_t>(-1);
size_t s_UpdateId = kNoUpdate;

#if defined(ESP32)

// The SX1262 on an ESP-IDF SPI bus. Joins the bus when the display (or an SD
// card) has already started it: ESP-IDF allows several devices on one bus,
// and LovyanGFX takes the bus for each of its transfers.
class EspSx126xBus : public ISX126xBus
{
public:
    bool Begin(const SX1262LoRaComponent& c, std::string& error)
    {
        const auto host = static_cast<spi_host_device_t>(c.spiHost);
        spi_bus_config_t bus = {};
        bus.mosi_io_num = c.mosiPin;
        bus.miso_io_num = c.misoPin;
        bus.sclk_io_num = c.sckPin;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = 300;
        const esp_err_t init = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
        if (init != ESP_OK && init != ESP_ERR_INVALID_STATE)  // INVALID_STATE: already up
        {
            error = std::string("spi_bus_initialize: ") + esp_err_to_name(init);
            return false;
        }

        spi_device_interface_config_t device = {};
        device.clock_speed_hz = c.spiHz;
        device.mode = 0;
        device.spics_io_num = c.csPin;
        device.queue_size = 1;
        const esp_err_t add = spi_bus_add_device(host, &device, &m_Device);
        if (add != ESP_OK)
        {
            error = std::string("spi_bus_add_device: ") + esp_err_to_name(add);
            return false;
        }

        if (c.dio1Pin >= 0)
        {
            m_Dio1Pin = static_cast<gpio_num_t>(c.dio1Pin);
            gpio_config_t dio1 = {};
            dio1.pin_bit_mask = 1ULL << c.dio1Pin;
            dio1.mode = GPIO_MODE_INPUT;
            gpio_config(&dio1);
        }
        m_BusyPin = static_cast<gpio_num_t>(c.busyPin);
        m_ResetPin = static_cast<gpio_num_t>(c.resetPin);
        gpio_config_t busy = {};
        busy.pin_bit_mask = 1ULL << c.busyPin;
        busy.mode = GPIO_MODE_INPUT;
        gpio_config(&busy);
        gpio_config_t reset = {};
        reset.pin_bit_mask = 1ULL << c.resetPin;
        reset.mode = GPIO_MODE_OUTPUT;
        gpio_config(&reset);
        gpio_set_level(m_ResetPin, 1);
        return true;
    }

    bool Transfer(const uint8_t* tx, uint8_t* rx, size_t length) override
    {
        spi_transaction_t t = {};
        t.length = length * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        return spi_device_polling_transmit(m_Device, &t) == ESP_OK;
    }

    void SetReset(bool high) override { gpio_set_level(m_ResetPin, high ? 1 : 0); }
    bool IsBusy() const override { return gpio_get_level(m_BusyPin) != 0; }
    bool HasDio1() const override { return m_Dio1Pin != GPIO_NUM_NC; }
    bool Dio1() const override { return gpio_get_level(m_Dio1Pin) != 0; }
    void DelayMs(uint32_t ms) override { vTaskDelay(pdMS_TO_TICKS(ms > 0 ? ms : 1)); }
    uint32_t NowMs() const override { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

private:
    spi_device_handle_t m_Device = nullptr;
    gpio_num_t m_BusyPin = GPIO_NUM_NC;
    gpio_num_t m_Dio1Pin = GPIO_NUM_NC;
    gpio_num_t m_ResetPin = GPIO_NUM_NC;
};

// Made once and never freed, like the other drivers the boot scene starts.
EspSx126xBus* s_Bus = nullptr;
SX1262Radio* s_Radio = nullptr;

#endif

}  // namespace

void SX1262LoRaComponent::Setup(SetupCallback onComplete)
{
#if defined(ESP32)
    if (!s_Radio)
    {
        std::string error;
        auto* bus = new EspSx126xBus();
        if (!bus->Begin(*this, error))
        {
            DEKI_LOG_ERROR("SX1262LoRaComponent: SPI for the radio: %s", error.c_str());
            delete bus;
            onComplete(false);
            return;
        }

        SX1262Config config;
        config.frequencyHz = static_cast<uint32_t>(frequencyHz);
        config.spreadingFactor = spreadingFactor;
        // The fractional bandwidths, written as whole kHz.
        switch (bandwidthKHz)
        {
            case 7: config.bandwidthHz = 7810; break;
            case 10: config.bandwidthHz = 10420; break;
            case 15: config.bandwidthHz = 15630; break;
            case 20: config.bandwidthHz = 20830; break;
            case 31: config.bandwidthHz = 31250; break;
            case 41: config.bandwidthHz = 41670; break;
            case 62: config.bandwidthHz = 62500; break;
            default: config.bandwidthHz = static_cast<uint32_t>(bandwidthKHz) * 1000; break;
        }
        config.codingRate = codingRate;
        config.txPowerDbm = txPowerDbm;
        config.syncWord = static_cast<uint8_t>(syncWord);
        config.tcxoVoltage = tcxoVoltage;
        config.dio2AsRfSwitch = dio2AsRfSwitch;
        config.dutyCyclePercent = dutyCyclePercent;

        auto* radio = new SX1262Radio(*bus);
        if (!radio->Begin(config, error))
        {
            DEKI_LOG_ERROR("SX1262LoRaComponent: %s", error.c_str());
            delete radio;
            delete bus;
            onComplete(false);
            return;
        }
        s_Bus = bus;
        s_Radio = radio;
        DEKI_LOG_INFO("SX1262LoRaComponent: listening on %.3f MHz, SF%d, %d kHz, %d dBm, %.1f%% duty cycle",
                      frequencyHz / 1e6, (int)spreadingFactor, (int)bandwidthKHz, (int)txPowerDbm,
                      (double)dutyCyclePercent);
    }

    DekiLoRa::SetCurrent(s_Radio);
    if (s_UpdateId == kNoUpdate)
    {
        s_UpdateId = Deki::Engine::GetInstance().RegisterUpdate(
            [](uint32_t)
            {
                if (s_Radio)
                {
                    s_Radio->Update();
                }
            });
    }
    onComplete(true);
#else
    // No radio driver on this platform: the game sees none.
    DEKI_LOG_INFO("SX1262LoRaComponent: no SX1262 driver on this platform; no radio");
    onComplete(true);
#endif
}

void SX1262LoRaComponent::StopUpdates()
{
    if (s_UpdateId != kNoUpdate)
    {
        Deki::Engine::GetInstance().UnregisterUpdate(s_UpdateId);
        s_UpdateId = kNoUpdate;
    }
}

}  // namespace DekiLora
