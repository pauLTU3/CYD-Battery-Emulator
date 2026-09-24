# Editing the CYD UI with SquareLine Studio

The repository contains the complete editable UI project. You do not need to
recreate the screens from the generated C files.

## Project files

- [`CYD.spj`](../CYD.spj) — main SquareLine Studio project; open this file.
- [`CYD.sll`](../CYD.sll) — SquareLine layout and editor metadata.
- [`CYD.slp`](../CYD.slp) — local export settings.
- [`Themes.slt`](../Themes.slt) — SquareLine theme data.
- [`src/ui/`](../src/ui/) — generated LVGL source used by the firmware.
- [`SquareLine_CYD/`](../SquareLine_CYD/) — optional Windows LVGL simulator.

The project was last saved with SquareLine Studio `1.6.1` and targets LVGL 8.

## Editing and exporting

1. Clone or download this repository.
2. Open `CYD.spj` in SquareLine Studio.
3. Make the UI changes.
4. Set both SquareLine export folders to the local repository's `src/ui`
   directory. The path stored in `CYD.slp` may point to the original author's
   checkout and should be changed on another computer.
5. Export the UI files.
6. Build the firmware with `pio run -e cyd` before submitting changes.

## Important compatibility notes

Firmware code refers to a number of generated SquareLine objects by name.
Renaming or deleting these objects requires updating their references in
`src/espnow_receiver.cpp` and `src/screen_network.cpp`:

- main single- and dual-battery screen values;
- battery status and balancing labels;
- cell charts and min/max/deviation labels;
- Wi-Fi, IP address, and firmware-version labels;
- navigation buttons and the faults screen.

Do not commit SquareLine backup directories, editor caches, `.pio`, or local
`.vscode` settings. Changes to the project source and the regenerated `src/ui`
files should be committed together so other contributors can continue editing
the same UI visually.
