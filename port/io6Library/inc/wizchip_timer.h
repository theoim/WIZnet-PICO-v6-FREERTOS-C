/**
 * Copyright (c) 2024 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _WIZCHIP_TIMER_H_
#define _WIZCHIP_TIMER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define RECEIVE_TIMEOUT (1000 * 10) // 10 seconds

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/*! \brief Initialize timer callback function
 *  \ingroup wizchip_timer
 *
 *  Add a repeating timer that is called repeatedly at the specified interval in milliseconds.
 *
 *  \param callback The repeating timer callback function
 */
void wizchip_1msec_timer_init(void (*callback)(void));

/*! \brief Assign timer callback function
 *  \ingroup wizchip_timer
 *
 *  1 millisecond timer callback function.
 *
 *  \param t Information about a repeating timer
 */
bool wizchip_1msec_timer_callback(struct repeating_timer *t);

/*! \brief Wait for the given number of milliseconds before returning
 *  \ingroup wizchip_timer
 *
 *  This method attempts to perform a lower power sleep (using WFE) as much as possible.
 *
 *  \param msec The number of milliseconds to sleep
 */
void wizchip_delay_msec(uint32_t msec);

/*! \brief Wait for the given number of seconds before returning
 *  \ingroup wizchip_timer
 *
 *  This method attempts to perform a lower power sleep (using WFE) as much as possible.
 *
 *  \param msec The number of milliseconds * 1000 to sleep
 */
void wizchip_delay_sec(uint32_t msec);

#ifdef __cplusplus
}
#endif

#endif /* _WIZCHIP_TIMER_H_ */
