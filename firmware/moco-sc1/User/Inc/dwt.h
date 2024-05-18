
#ifndef DWT_H_
#define DWT_H_

#include <stdint.h>
#include "cmsis_compiler.h"
#include "parameters.h"

// Ref:
// - https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/Debug-register-support-in-the-SCS/Debug-Exception-and-Monitor-Control-Register--DEMCR
// - https://developer.arm.com/documentation/ddi0439/b/Data-Watchpoint-and-Trace-Unit/DWT-Programmers-Model
// - https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/Control-register--DWT-CTRL?lang=en

#define DEMCR (*(volatile uint32_t *)0xE000EDFC) // Debug Exception and Monitor Control Register.
#define DEMCR_TRCENA (1U << 24U)                 // The TRCENA bit in DEMCR.

#define DWT_CTRL (*(volatile uint32_t *)0xE0001000) // Control Register.
#define DWT_CTRL_CYCCNTENA (1U)                     // The CYCCNTENA bit in DWT_CTRL.

#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004) // Cycle Count Register.

/**
 * @brief Init DWT (Data Watchpoint and Trace).
 */
__STATIC_INLINE void dwt_init(void)
{
    DEMCR |= DEMCR_TRCENA;          // Enable DWT and ITM by setting the TRCENA bit to 1.
    DWT_CYCCNT = 0U;                // Clear CYCCNT.
    DWT_CTRL |= DWT_CTRL_CYCCNTENA; // Enable CYCCNT counter by setting the CYCCNTENA bit to 1.
}

/**
 * @brief Get current DWT cycle count.
 *
 * @return The DWT cycle count.
 */
__STATIC_INLINE uint32_t dwt_get_cyccnt(void)
{
    return DWT_CYCCNT;
}

/**
 * @brief Calculate the time difference between 2 DWT cycle counts.
 *
 * @param a Starting DWT cycle count.
 * @param b Ending DWT cycle count.
 * @return The time difference in ms.
 */
__STATIC_INLINE float dwt_get_diff_time(uint32_t a, uint32_t b)
{
    uint32_t diff;
    if (b < a)
    {
        diff = UINT32_MAX - a + b + 1U; // +1U for wrap-around.
    }
    else
    {
        diff = b - a;
    }
    return (diff / (float)SYSCLK_MHZ);
}

#endif /* DWT_H_ */
