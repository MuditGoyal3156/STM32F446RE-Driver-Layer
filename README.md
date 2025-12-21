# STM32F446RE – Driver Layer

This repository contains custom bare-metal drivers for the STM32F446RE microcontroller, written using direct register-level programming (CMSIS-based, no HAL / no LL).

The drivers are implemented for commonly used peripherals and validated using small test applications.
____________________________________________________________________________________________________________________________________________________________________
## ✅ **Supported Peripherals**

🔹**GPIO**

Pin mode configuration (Input / Output / Alternate / Analog)

Output type, speed, pull-up / pull-down

Pin read, write, and toggle

External Interrupts (EXTI)

Edge selection (rising / falling)

IRQ configuration

Callback-based interrupt handling

🔹 **SPI**

Master and Slave modes

Full-duplex communication

Software & hardware slave management

Polling and interrupt-based transmission

Multi-byte TX / RX support

🔹 **I2C**

Master mode communication

Start / Stop condition handling

Address and data phase control

ACK / NACK management

Interrupt-based flow

🔹 **UART (USART)**

Baud rate configuration

Transmit and receive support

Polling and interrupt modes

Debug output support over serial terminal
____________________________________________________________________________________________________________________________________________________________________
🛠 **Development Environment**

MCU: STM32F446RE

Toolchain: arm-none-eabi-gcc

IDE: STM32CubeIDE / VS Code

Debugger: ST-Link

Logic Analyzer used for verification
____________________________________________________________________________________________________________________________________________________________________
📌 **Notes**

All drivers are written using direct register access

APIs are designed to closely follow the STM32 reference manual

Code is intended for learning, experimentation, and reuse
