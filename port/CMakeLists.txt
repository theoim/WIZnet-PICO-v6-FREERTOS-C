# io6Library
add_library(IOLIBRARY_FILES STATIC)

target_sources(IOLIBRARY_FILES PUBLIC
        ${PORT_DIR}/io6Library/src/w6x00_spi.c
        ${PORT_DIR}/io6Library/src/w6x00_gpio_irq.c
        )
if(${WIZNET_CHIP} STREQUAL W6300)
        pico_generate_pio_header(IOLIBRARY_FILES ${PORT_DIR}/io6Library/src/wizchip_pio_spi.pio)

        target_sources(IOLIBRARY_FILES PUBLIC 
        ${PORT_DIR}/io6Library/src/wizchip_pio_spi.c
        ${PORT_DIR}/io6Library/src/wizchip_timer.c
        )
endif()

target_include_directories(IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${PORT_DIR}/io6Library/inc
        ${PORT_DIR}/FreeRTOS-Kernel/inc
        ${PORT_DIR}
        )

target_link_libraries(IOLIBRARY_FILES PUBLIC
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_clocks
        FreeRTOS-Kernel-Heap4
        )

# timer
add_library(TIMER_FILES STATIC)

target_sources(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer/timer.c
        )

target_include_directories(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer
        )

target_link_libraries(TIMER_FILES PRIVATE
        pico_stdlib      
        )

# port_common
add_library(PORT_COMMON_FILES STATIC)

target_include_directories(PORT_COMMON_FILES PUBLIC
        ${PORT_DIR}
        )

target_link_libraries(PORT_COMMON_FILES PRIVATE
        pico_stdlib
        hardware_clocks
        )