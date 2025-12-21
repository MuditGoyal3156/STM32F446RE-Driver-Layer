STM32F446RE – Driver Layer

This repository contains custom bare-metal drivers for the STM32F446RE microcontroller, written using direct register-level programming (CMSIS-based, no HAL / no LL).

The drivers are implemented for commonly used peripherals and validated using small test applications.

✅ Supported Peripherals
🔹 GPIO

Pin mode configuration (Input / Output / Alternate / Analog)

Output type, speed, pull-up / pull-down

Pin read, write, and toggle

External Interrupts (EXTI)

Edge selection (rising / falling)

IRQ configuration

Callback-based interrupt handling

🔹 SPI

Master and Slave modes

Full-duplex communication

Software & hardware slave management

Polling and interrupt-based transmission

Multi-byte TX / RX support

🔹 I2C

Master mode communication

Start / Stop condition handling

Address and data phase control

ACK / NACK management

Interrupt-based flow (work in progress)

🔹 UART (USART)

Baud rate configuration

Transmit and receive support

Polling and interrupt modes

Debug output support over serial terminal

📁 Project Structure
STM32F446RE-Driver-Layer/
│
├── stm32f446xx_drivers/
│   ├── Inc/
│   │   ├── stm32f446xx.h
│   │   ├── stm32f446xx_gpio_driver.h
│   │   ├── stm32f446xx_spi_driver.h
│   │   ├── stm32f446xx_i2c_driver.h
│   │   └── stm32f446xx_usart_driver.h
│   │
│   └── Src/
│       ├── stm32f446xx_gpio_driver.c
│       ├── stm32f446xx_spi_driver.c
│       ├── stm32f446xx_i2c_driver.c
│       └── stm32f446xx_usart_driver.c
│
├── 001HelloWorld/
├── 001SPISlaveRxString/
├── 002SPISlaveCmdHandling/
├── 003SPISlaveUartReadOverSPI/
├── SPI_COMMUNICATION/
└── README.md


stm32f446xx_drivers/ contains the driver layer

Numbered folders are test applications for validating drivers

🛠 Development Environment

MCU: STM32F446RE

Toolchain: arm-none-eabi-gcc

IDE: STM32CubeIDE / VS Code

Debugger: ST-Link

Logic Analyzer used for SPI verification

📌 Notes

All drivers are written using direct register access

APIs are designed to closely follow the STM32 reference manual

Code is intended for learning, experimentation, and reuse
