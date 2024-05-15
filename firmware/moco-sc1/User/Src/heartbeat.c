
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "parameters.h"

static uint16_t ccr = 0; // Capture/Compare Register value
static int8_t dir = 1;

void heartbeat_init(void)
{
    LL_TIM_EnableAllOutputs(HB_LED_TIM_INST);
    LL_TIM_CC_EnableChannel(HB_LED_TIM_INST, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_UPDATE(HB_LED_TIM_INST);
    LL_TIM_EnableCounter(HB_LED_TIM_INST);
}

void heartbeat_handler(void)
{
    if (ccr >= (HB_LED_TIM_ARR * 0.8F))
    {
        dir = -1;
    }
    else if (ccr == 0)
    {
        dir = 1;
    }

    ccr += dir;
    // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_0);
    LL_TIM_OC_SetCompareCH1(HB_LED_TIM_INST, ccr);
}
