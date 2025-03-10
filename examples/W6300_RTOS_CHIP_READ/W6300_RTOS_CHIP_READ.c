/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "w6x00_gpio_irq.h"
#include "w6x00_spi.h"

#include "mbedtls/x509_crt.h"
#include "mbedtls/error.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/platform.h"

#include "timer.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define TCP_TASK_STACK_SIZE  4096  // 스택 크기 증가
#define TCP_TASK_PRIORITY    1

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Socket */
#define SOCKET_SSL 0

/* Semaphore */
static xSemaphoreHandle recv_sem = NULL;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;



/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 20},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .lla = {0xfe, 0x80, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x02, 0x08, 0xdc, 0xff,
                0xfe, 0x57, 0x57, 0x25},             // Link Local Address
        .gua = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Global Unicast Address
        .sn6 = {0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xff,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // IPv6 Prefix
        .gw6 = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Gateway IPv6 Address
        .dns6 = {0x20, 0x01, 0x48, 0x60,
                0x48, 0x60, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x88, 0x88},             // DNS6 server
        .ipmode = NETINFO_STATIC_ALL
};


/**
 * ----------------------------------------------------------------------------------------------------
 * FreeRTOS Stack Overflow Check
 * ----------------------------------------------------------------------------------------------------
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("\n[ERROR] Stack Overflow in Task: %s\n", pcTaskName);
    while (1);
}

/**
 * ----------------------------------------------------------------------------------------------------
 * FreeRTOS Malloc Failure Hook
 * ----------------------------------------------------------------------------------------------------
 */
void vApplicationMallocFailedHook(void)
{
    printf("\n[ERROR] Heap Memory Allocation Failed!\n");
    while (1);
}

/**
 * ----------------------------------------------------------------------------------------------------
 * TCP Task (Version Check with Debugging)
 * ----------------------------------------------------------------------------------------------------
 */
void tcp_task(void *pvParameters)
{
    uint16_t version = 0;
    while (1)
    {
        printf("\n======================================\n");
        printf("Task Running... Checking W6300 Version\n");

        for (int i = 1; i <= 10; i++)
        {
            version = getVER();
            printf("%d. W6300 VER : 0x%04X\r\n", i, version);

            

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1초 대기
        }
    }
}


/**
 * ----------------------------------------------------------------------------------------------------
 * Clock Configuration
 * ----------------------------------------------------------------------------------------------------
 */
static void set_clock_khz(void)
{
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(
        clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        PLL_SYS_KHZ * 1000,
        PLL_SYS_KHZ * 1000
    );
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;
}

/* GPIO */
static void gpio_callback(void)
{
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(recv_sem, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}



/**
 * ----------------------------------------------------------------------------------------------------
 * Main Function
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    set_clock_khz();
    stdio_init_all();

    sleep_ms(3000);
    printf("\n[INFO] System Initialization...\n");

    // 힙 메모리 사용량 확인
    printf("[INFO] Initial Free Heap Size: %d bytes\n", xPortGetFreeHeapSize());

    printf("[INFO] WIZchip Initialization Start\n");
    wizchip_spi_initialize();
    wizchip_cris_initialize();  
    wizchip_reset();
    wizchip_initialize();

    // W6300 버전 초기 확인 (디버깅용)
    printf("[INFO] Checking Initial W6300 Version...\n");

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);  

    wizchip_1ms_timer_initialize(repeating_timer_callback);
    wizchip_gpio_interrupt_initialize(SOCKET_SSL, gpio_callback);
    

        // GPIO 25번을 출력으로 설정
        const uint LED_PIN = 25;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    
        // GPIO 25번을 HIGH로 설정
        gpio_put(LED_PIN, 1);

    // TCP Task 생성
    if (xTaskCreate(tcp_task, "chip_read_Task", TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, NULL) != pdPASS)
    {
        printf("[ERROR] Task Creation Failed!\n");
        while (1);
    }

    // FreeRTOS 스케줄러 시작
    vTaskStartScheduler();

    // 메인 루프 (실행되지 않음)
    while (1) {}

    return 0;
}
