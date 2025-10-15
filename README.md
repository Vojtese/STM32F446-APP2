# STM32F446 Application Slot 2 (APP2)

This repository contains the fallback application firmware for the STM32F446 signal acquisition unit. It is used when the primary application (APP1) fails CRC validation or is missing, ensuring system reliability in field deployments.

This version does not support in-application programming. It focuses purely on signal acquisition and communication. Firmware updates are handled by the bootloader.

---

## 🚀 Features

- Acquisition of analog and digital sensor data:
  - Pressure, flow, temperature, voltage, current
- Signal conditioning and filtering
- RS-485 communication using UART + DMA
- Bootloader jump compatibility
- Minimal footprint for reliability

---

## 🧠 CMSIS LL Driver Usage

- `LL_ADC_REG_StartConversion()`, `LL_ADC_REG_ReadConversionData32()` – ADC sampling
- `LL_USART_TransmitData8()`, `LL_USART_IsActiveFlag_TXE()` – UART transmission
- `LL_DMA_EnableChannel()` – DMA for UART
- `LL_GPIO_IsInputPinSet()` – digital sensor polling

---

## 📁 Project Structure

- `Core/` – Application logic and sensor routines
- `Drivers/` – STM32 LL drivers
- `.ioc` – STM32CubeMX configuration
- `STM32F446RETX_FLASH.ld` – Linker script for App2 memory region
- `docs/` – Diagrams and hardware images

---

## 📊 Application Architecture

### ⚙️ Application Flow without IAP

This version is a simplified signal acquisition application. It does not support in-application programming. Firmware updates are handled exclusively by the bootloader.

![APP Standard Flow](docs/SWdesignv2.svg)

---

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)
- [STM32F446-APP1](https://github.com/Vojtese/STM32F446-APP1)

---

## 🛠️ TODO & Improvements

- [ ] Add basic diagnostics output via UART
- [ ] Add watchdog integration for fault recovery
- [ ] Add bootloader-triggered reset logic
- [ ] Add optional IAP support for future expansion

---

## 📜 License

This project is licensed under the GNU General Public License v3.0.
