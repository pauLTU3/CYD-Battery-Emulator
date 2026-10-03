# Changelog

All notable changes to each published firmware release are documented here.

## [2.3.0] - 2026-10-03

### Changed

- Set the CYD Wi-Fi access point password to `123456789` by default.
- Keep web administrator authentication optional and disabled by default.
- Add a **Web Admin Protection** setting for protecting configuration changes and OTA uploads with a user-selected password.
- Turn the setup access point off after the CYD joins the configured Wi-Fi network and restore it if that connection is lost.
- Stop returning the saved home Wi-Fi password in the configuration page.
- Give each access point a chip-specific `BatteryEmulator-CYD-XXXX` name.
- Document Firefox, Chrome, and Edge Web Serial support in the installer.

### Firmware files

- `CYD-OTA-firmware.bin` updates an existing installation through the CYD OTA page.
- `CYD-factory-firmware.bin` is the complete image for a first USB installation.
