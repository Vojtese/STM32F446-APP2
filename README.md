# STM32F446 Application Slot 2 (APP2)

This repository contains the fallback application firmware for the STM32F446 signal acquisition unit. It ensures system reliability in case the primary application fails CRC or is missing.

## 🚀 Features

- Identical structure to APP1
- Sensor acquisition and RS485 communication
- Compatible with bootloader fallback logic
- CMSIS LL drivers for low-level control

## 📁 Project Structure

- `Core/`: Application logic
- `Drivers/`: STM32 LL drivers
- `.ioc`: STM32CubeMX configuration
- `STM32F446RETX_FLASH.ld`: Linker script for App2 memory region

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)
- [STM32F446-APP1](https://github.com/Vojtese/STM32F446-APP1)

## 📜 License

This project is licensed under the GNU General Public License v3.0.
