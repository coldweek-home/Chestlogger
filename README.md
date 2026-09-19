# ChestLogger in VS Code

This is an AVR-GCC project for an **ATtiny85** running at `1 MHz`.

## Required tools

- Microchip Studio 7 AVR-GCC toolchain
- VS Code with the Microsoft C/C++ extension
- `avrdude` and a USBasp programmer for flashing

The workspace files use the default Microchip Studio installation paths. If the tools are installed elsewhere, override the variables when invoking `make`:

```text
make ATMEL_TOOLCHAIN=C:/path/to/avr8-gnu-toolchain DFP_DIR=C:/path/to/ATtiny_DFP
```

## Build and flash

Use `Ctrl+Shift+B` for the default Debug build. Other commands are available through `Terminal > Run Build Task`:

- `Build Debug`
- `Build Release`
- `Flash Debug with USBasp`
- `Flash Release with USBasp`
- `Clean`

The same operations can be run from a terminal with the Microchip Studio `make.exe`:

```text
make CONFIG=Debug
make CONFIG=Release
make CONFIG=Debug flash
```

Build results are written to `build/Debug` or `build/Release`.