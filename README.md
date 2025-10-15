# STM32F446 Application Slot 2 (APP2)

This repository contains the fallback application firmware for the STM32F446 signal acquisition unit. It is used when the primary application (APP1) fails CRC validation or is missing, ensuring system reliability in field deployments.

Like APP1, this firmware also supports in-application programming (IAP), allowing firmware updates to be received and flashed while the application is running.

---

## 🚀 Features

- Acquisition of analog and digital sensor data:
  - Pressure, flow, temperature, voltage, current
- Signal conditioning and filtering
- RS-485 communication using UART + DMA
- In-application programming (IAP) via UART or RS485
- Bootloader jump compatibility
- Minimal footprint for reliability

---

## 🧠 CMSIS LL Driver Usage

- `LL_ADC_REG_StartConversion()`, `LL_ADC_REG_ReadConversionData32()` – ADC sampling
- `LL_USART_TransmitData8()`, `LL_USART_IsActiveFlag_TXE()` – UART transmission
- `LL_DMA_EnableChannel()` – DMA for UART
- `LL_GPIO_IsInputPinSet()` – digital sensor polling
- `LL_FLASH_Program()` – Flash writing during IAP
- `LL_CRC_FeedData32()` – CRC validation

---

## 📁 Project Structure

- `Core/` – Application logic and sensor routines
- `Drivers/` – STM32 LL drivers
- `iap.c` – In-application programming logic
- `.ioc` – STM32CubeMX configuration
- `STM32F446RETX_FLASH.ld` – Linker script for App2 memory region
- `docs/` – Diagrams and hardware images

---

## 📊 Application Architecture

### 🧠 Application Flow with IAP

This version supports in-application programming. The application can receive firmware packets via UART or RS485, validate them, and flash a new application image without rebooting into the bootloader.

![APP IAP Flow](docs/img/SWdesignv1.svg)

---

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)
- [STM32F446-APP1](https://github.com/Vojtese/STM32F446-APP1)
- [serial_BIN_file_transfer](https://github.com/Vojtese/serial_BIN_file_transfer)
- [STM32F446-uploadRS485Test](https://github.com/Vojtese/STM32F446-uploadRS485Test)

---

## 🛠️ TODO & Improvements

- [ ] Add bootloader-triggered reset logic
- [ ] Add UART status feedback during IAP
- [ ] Improve Flash write verification and error handling
- [ ] Add watchdog integration for fault recovery

---

## 📜 License

This project is licensed under the GNU General Public License v3.0.
