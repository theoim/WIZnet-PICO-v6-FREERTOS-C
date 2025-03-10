# W6100-EVB-PICO, W6100-EVB-PICO2 and W6300-EVB-PICO2 FreeRTOS Project

This project is a simple example of OpenSSL communication using the **W6100-EVB-PICO**, **W6100-EVB-PICO2** and **W6300-EVB-PICO2** board, which combines the **RP2040** & **RP2350** microcontroller and the **W6100** & **W6300** chip from WIZnet.  
The implementation is based on **Pico-SDK, FreeRTOS, MbedTLS, and io6Library** for WIZnet communication.

This example demonstrates how to establish secure communication over Ethernet using OpenSSL and the W6300 chip.  
By following the instructions below, you can successfully build and run this project on your W6300-EVB-PICO2 board.

---

## 📌 **Key Features**
- **Microcontroller:** RP2040 (ARM Cortex-M0), RP2350 (ARM Cortex-M33)  
- **Ethernet Chip:** W6100, W6300 (WIZnet)  
- **RTOS:** FreeRTOS  
- **Libraries Used:**
  - **Pico-SDK:** RP2350 SDK for peripheral and hardware control.  
  - **FreeRTOS:** Real-Time Operating System for task management.  
  - **MbedTLS:** For secure communication (SSL/TLS).  
  - **io6Library:** For WIZnet Ethernet communication.  
- **Example Functionality:** Simple OpenSSL communication over Ethernet.  

---

## ⚠️ **Important Notice for Building the Project**
To successfully build the project, you need to modify a line in the `port.c` file of the FreeRTOS Kernel:

- **File:** `WIZnet-PICO-v6-FREERTOS-C\libraries\FreeRTOS-Kernel\portable\ThirdParty\GCC\RP2040\port.c`  
- **Line:** **414**  
- **Original Code:**
  ```c
  uint32_t irq_num = SIO_IRQ_PROC0 + get_core_num();
  ```
- **Modified Code:**
  ```c
  uint32_t irq_num = 15 + get_core_num();
  ```
- **Reason:** This modification is required for compatibility with the RP2350 architecture to ensure successful compilation.

---

## 📦 **Requirements**
- **Hardware:**
  - W6300-EVB-PICO2 board.  
  - Ethernet connection.  

- **Software:**
  - Pico-SDK  
  - FreeRTOS  
  - MbedTLS  
  - io6Library  

---

## 🔄 **How to Build and Run**
1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   ```

2. **Apply the modification mentioned above to `port.c`.**

3. **Build the project using CMake:**
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

4. **Upload the firmware to the W6300-EVB-PICO2 board.**

---

## 📧 **Contact**
For any issues or inquiries, please open an issue on GitHub.

---


