WIZnet FreeRTOS Project

This project is a simple example of OpenSSL communication using the W6300-EVB-PICO2 board, which combines the RP2350 microcontroller and the W6300 chip from WIZnet.The implementation is based on Pico-SDK, FreeRTOS, MbedTLS, and io6Library for WIZnet communication.

This example demonstrates how to establish secure communication over Ethernet using OpenSSL and the W6300 chip.By following the instructions below, you can successfully build and run this project on your W6300-EVB-PICO2 board.

📌 Key Features

Microcontroller: RP2350 (ARM Cortex-M55)

Ethernet Chip: W6300 (WIZnet)

RTOS: FreeRTOS

Libraries Used:

Pico-SDK: RP2350 SDK for peripheral and hardware control.

FreeRTOS: Real-Time Operating System for task management.

MbedTLS: For secure communication (SSL/TLS).

io6Library: For WIZnet Ethernet communication.

Example Functionality: Simple OpenSSL communication over Ethernet.

⚠️ Important Notices for Building the Project

To successfully build the project, you need to modify a line in the port.c file of the FreeRTOS Kernel and adjust the CMakeLists.txt file:

File: WIZnet-PICO-v6-FREERTOS-C\libraries\FreeRTOS-Kernel\portable\ThirdParty\GCC\RP2040\port.c

Line: 414

Original Code:

uint32_t irq_num = SIO_IRQ_PROC0 + get_core_num();

Modified Code:

uint32_t irq_num = 15 + get_core_num();

Reason: This modification is required for compatibility with the RP2350 architecture to ensure successful compilation.

⚠️ CMakeLists.txt Configuration

To select the appropriate board, you need to uncomment one of the following lines in your CMakeLists.txt file:

# Set board
# set(BOARD_NAME W6100_EVB_PICO)
# set(BOARD_NAME W6100_EVB_PICO2)
set(BOARD_NAME W6300_EVB_PICO2)

Only one of these lines should be uncommented at a time to ensure that the correct board is selected.

Additionally, ensure that the Ethernet chip setting is correct:

if(${BOARD_NAME} STREQUAL W6100_EVB_PICO)
    set(WIZNET_CHIP W6100)
    add_definitions(-D_WIZCHIP_=W6100)
elseif(${BOARD_NAME} STREQUAL W6100_EVB_PICO2)
    set(PICO_PLATFORM rp2350)
    set(WIZNET_CHIP W6100)
    add_definitions(-D_WIZCHIP_=W6100)
elseif(${BOARD_NAME} STREQUAL W6300_EVB_PICO2)
    set(PICO_PLATFORM rp2350)
    set(WIZNET_CHIP W6300)
    add_definitions(-D_WIZCHIP_=W6300)

Make sure that the settings match your hardware configuration to avoid build errors.

📦 Requirements

Hardware:

W6300-EVB-PICO2 board.

Ethernet connection.

Software:

Pico-SDK

FreeRTOS

MbedTLS

io6Library

🔄 How to Build and Run

Clone the repository:

git clone <repository-url>

Apply the modification mentioned above to port.c.

Build the project using CMake:

mkdir build
cd build
cmake ..
make

Upload the firmware to the W6300-EVB-PICO2 board.

