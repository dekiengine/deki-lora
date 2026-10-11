# Deki LoRa

LoRa radio for the Deki Engine: a driver for the Semtech SX1262 that sends
and receives packets, with a duty-cycle limit built in.

Part of [Deki Engine](https://github.com/dekiengine/deki-engine).

## Use

Add **SX1262 LoRa** (`DekiLora::SX1262LoRaComponent`) to the board's boot
scene with its wiring and radio settings. Game code then reaches the radio
through `DekiLora::DekiLoRa::GetCurrent()`, which is `nullptr` on a board
without one or when it did not answer:

```cpp
#include "DekiLoRa.h"  // from deki-lora

if (auto* radio = DekiLora::DekiLoRa::GetCurrent())
{
    const char text[] = "hello";
    if (radio->MillisecondsUntilCanSend() == 0)
    {
        radio->Send(reinterpret_cast<const uint8_t*>(text), sizeof(text) - 1);
    }

    DekiLora::LoRaPacket packet;
    while (radio->Receive(packet))
    {
        // packet.data, packet.rssi (dBm), packet.snr (dB)
    }
}
```

The radio listens whenever it is not sending; received packets queue (the
last 8) until read.

## Frequency, power and duty cycle

Radio use is regulated. Set the frequency, power and duty cycle your region
allows. The defaults are Europe's 868.0-868.6 MHz sub-band: **868.1 MHz,
14 dBm, 1% duty cycle**. In the Americas use 902-928 MHz (915 MHz).

The duty cycle is enforced: after a packet that took T on air, `Send()`
refuses for T × (100 / duty − 1). At SF7 and 125 kHz a 16-byte packet takes
about 50 ms, so at 1% the next may go about 5 s later.
`MillisecondsUntilCanSend()` says how long is left.

## Boards

On the LilyGO T-Deck and T-Deck Plus the SX1262 shares the display's SPI bus:
SCK 40, MOSI 41, MISO 38, CS 9, BUSY 13, RESET 17, a 1.8 V TCXO on DIO3, and
DIO2 switching the antenna. Those are the component's defaults.

Only ESP-IDF builds have a driver. Elsewhere the component starts no radio,
and `GetCurrent()` stays `nullptr`.

## Namespace

Types live in `DekiLora`.
