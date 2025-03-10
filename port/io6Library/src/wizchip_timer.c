/**
 * Copyright (c) 2024 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "wizchip_timer.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
static struct repeating_timer g_timer;
static void (*callback_ptr)(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
void wizchip_1msec_timer_init(void (*callback)(void))
{
  callback_ptr = callback;
  add_repeating_timer_us(-1000, wizchip_1msec_timer_callback, NULL, &g_timer);
}

bool wizchip_1msec_timer_callback(struct repeating_timer *t)
{
  if (callback_ptr != NULL)
  {
    callback_ptr();
  }
}

void wizchip_delay_msec(uint32_t msec)
{
  sleep_ms(msec);
}

void wizchip_delay_sec(uint32_t msec)
{
  sleep_ms(msec * 1000);
}
