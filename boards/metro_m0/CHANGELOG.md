# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.19.2](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.19.1...metro_m0-0.19.2) - 2025-06-26

### Other

- updated the following local packages: atsamd-hal

## [0.19.1](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.19.0...metro_m0-0.19.1) - 2025-06-22

### Other

- updated the following local packages: atsamd-hal

## [0.19.0](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.18.4...metro_m0-0.19.0) - 2025-05-08

### Other

- updated the following local packages: atsamd-hal

## [0.18.4](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.18.3...metro_m0-0.18.4) - 2025-05-07

### Other

- updated the following local packages: atsamd-hal

## [0.18.3](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.18.2...metro_m0-0.18.3) - 2025-04-22

### Other

- updated the following local packages: atsamd-hal

## [0.18.2](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.18.1...metro_m0-0.18.2) - 2025-04-12

### Other

- updated the following local packages: atsamd-hal

## [0.18.1](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.18.0...metro_m0-0.18.1) - 2025-01-31

### Examples

- Updates all Tier 1 BSP examples to use RTIC v2 and the RTC monotonic where applicable (#817)

## [0.18.0](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.17.1...metro_m0-0.18.0) - 2025-01-13

### Added

- [**breaking**] Add support for RTIC v2.0 monotonic drivers using RTC peripheral (#804)

## [0.17.1](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.17.0...metro_m0-0.17.1) - 2024-12-11

### Other

- updated the following local packages: atsamd-hal

## [0.17.0](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.16.0...metro_m0-0.17.0) - 2024-11-28

### Added

- [**breaking**] Add async support for many peripherals ([#635](https://github.com/atsamd-rs/atsamd/pull/635))

## [0.16.0](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.15.1...metro_m0-0.16.0) - 2024-11-17

### Examples

- Add DMA examples for SPI, I2C, and UART to Tier 1 BSPs ([#772](https://github.com/atsamd-rs/atsamd/pull/772))

### Dependencies

- Update HAL dependency to `0.19`

## [0.15.1](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.15.0...metro_m0-0.15.1) - 2024-10-25

### Other

- updated the following local packages: atsamd-hal

## [0.15.0](https://github.com/atsamd-rs/atsamd/compare/metro_m0-0.14.0...metro_m0-0.15.0) - 2024-10-17

### Refactored

- update path of Cargo config ([#749](https://github.com/atsamd-rs/atsamd/pull/749))
- Remove build profiles from Cargo.toml ([#762](https://github.com/atsamd-rs/atsamd/pull/762))

### Dependencies

- **[breaking]** Upgrade PAC generated code to latest SVD and `svd2rust-0.34.1` [#756](https://github.com/atsamd-rs/atsamd/pull/756):
- Update HAL dependency to `0.18`

## v0.14.0

- Implement `embedded-hal` `1.0` for GPIO, SPI, I2C, UART and fix examples
- Update the PACs to svd2rust 0.30.2.

## v0.13.0

- Replace homebrew time library with `fugit` (#672)

## v0.12.1

- Update to `atsamd-hal` version `0.15.1`

## v0.12.0

- Update `lib.rs` and examples to reflect removal of `v1` APIs and promotion of `v2` APIs
- Update `i2c_master` convenience function to use the new `sercom::v2::i2c` API
- Add an `i2c` example
- Updated to 2021 edition, updated dependencies, removed unused dependencies (#562)

## v0.11.0

- remove extraneous `embedded-hal` dependencies from BSPs
- cleanup `cortex_m` dependency
* move `usbd-x` crates used only in examples to `[dev-dependencies]`
* removed unnecessary dependency on `nb` and `panic_rtt` (#510)
- Update to use refactored SPI module (#467)

## v0.10.1

* Bump dependencies `rtic-monotonic` to `0.1.0-rc.1` and `cortex-m-rtic` to `0.6.0-rc.2`.

---

Changelog tracking started at v0.10.0
