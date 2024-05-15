
#ifndef DWT_H_
#define DWT_H_

#include <stdint.h>
#include "cmsis_compiler.h"
#include "parameters.h"

// Ref:
// - https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/Debug-register-support-in-the-SCS/Debug-Exception-and-Monitor-Control-Register--DEMCR
// - https://developer.arm.com/documentation/ddi0439/b/Data-Watchpoint-and-Trace-Unit/DWT-Programmers-Model

#define DEMCR (*(volatile uint32_t *)0xE000EDFC) // Debug Exception and Monitor Control Register
#define DEMCR_TRCENA (1U << 24U)

#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CTRL_CYCCNTENA (1U)

#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)

__STATIC_INLINE void dwt_init(void)
{
    DEMCR |= DEMCR_TRCENA;          // Enable DWT
    DWT_CYCCNT = 0U;                // Clear CYCCNT
    DWT_CTRL |= DWT_CTRL_CYCCNTENA; // Enable CYCCNT counter.
}

__STATIC_INLINE uint32_t dwt_get_cyccnt(void)
{
    return DWT_CYCCNT;
}

__STATIC_INLINE float dwt_get_diff_time(uint32_t a, uint32_t b)
{
    float t;
    if (b < a)
    {
        t = (0xFFFFFFFFU - a + b) / (float)SYSCLK_MHZ;
    }
    else
    {
        t = (b - a) / (float)SYSCLK_MHZ;
    }
    return t;
}

#endif /* DWT_H_ */
