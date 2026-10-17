# Changelog

Notable changes to `deki-lora`. Engine and editor changes are in the
[engine changelog](https://github.com/dekiengine/deki-engine/blob/master/CHANGELOG.md).

A package's `minEngine` names the engine version it needs. Before 1.0 a
breaking change bumps the minor across the editor, the engine and every
package together, so a package with no changes of its own is still released
alongside one that has them.

## 0.19.0

### Changed
- `minEngine` 0.19.0: built against engine 0.19.0, whose component metadata records the package each class is in. A build of this package from before does not load and is rebuilt.

### Added
- Japanese, Simplified Chinese and Korean: `editor/lang/ja.json`, `zh.json`
  and `ko.json` translate the names, tooltips and enum values this
  package's components show in the Inspector.

## 0.18.2

### Added
- The package: a Semtech SX1262 LoRa driver (`SX1262LoRaComponent` for the
  boot scene, `DekiLoRa::GetCurrent()` for game code). Sends and receives
  packets with RSSI and SNR, listens whenever it is not sending, and enforces
  a duty cycle (1% by default, Europe's 868 MHz rule). Joins an SPI bus the
  display already uses, as on the LilyGO T-Deck. ESP-IDF only.
- `dio1Pin` (45, the T-Deck's): with the radio's interrupt line wired, the
  radio is asked over SPI only when DIO1 is raised, rather than every frame
  on a bus the display is sending frames on.
