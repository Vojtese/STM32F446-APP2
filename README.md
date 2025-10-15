# STM32F446 Application Slot 2 (APP2)

This repository contains the fallback application firmware for the STM32F446 signal acquisition unit. It is used when the primary application (APP1) fails CRC validation or is missing, ensuring system reliability in field deployments.

## 🚀 Features

- Identical sensor acquisition and communication logic as APP1
- Fully compatible with bootloader fallback mechanism
- Memory-mapped to secondary application region
- CRC validation and watchdog support
- Modular structure for easy feature extension

## 🧠 CMSIS LL Driver Usage

Same as APP1, including:
- ADC, UART, DMA, GPIO
- Flash read access for CRC verification

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
