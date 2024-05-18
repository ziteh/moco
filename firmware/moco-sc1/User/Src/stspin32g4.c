
#include "stspin32g4.h"

// #include "main.h"
#include "cmsis_compiler.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "parameters.h"
#include "motion.h"
#include "dwt.h"
#include <math.h>

#define GET_PWM_CCR(dc) ((GD_PWM_TIM_ARR + 1) * (float)(dc)) // dc = 0 ~ 1.0

void spg4_init(void)
{
    dwt_init();

    spg4_set_pwm(50.0, 50.0, 50.0);
    LL_TIM_OC_SetCompareCH4(GD_PWM_TIM_INST, 3);

    // LL_TIM_EnableAllOutputs(TIM8);
    // LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    // LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
    // LL_TIM_EnableCounter(TIM8);

    LL_TIM_EnableIT_UPDATE(ML_TIM_INST);
    LL_TIM_EnableCounter(ML_TIM_INST);

    LL_TIM_EnableAllOutputs(GD_PWM_TIM_INST);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH3N);
    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(GD_PWM_TIM_INST);

    spg4_adc_init();
}

uint32_t a = 0;

void ml_handler(void)
{
    float vm = 12;
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_15);
    open_loop_velocity(0.25F, vm * 0.1F, vm, space_vector_pwm);
}

void spg4_adc_init()
{
    // LL_ADC_Enable(VBUS_ADC_INST);
    // LL_ADC_Disable(VBUS_ADC_INST);
    //  while (LL_ADC_IsEnabled(VBUS_ADC_INST))
    {
        __NOP();
    }
    HAL_Delay(10);

    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1))
    {
        __NOP();
    }

    // HAL_Delay(LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES);
    HAL_Delay(10);
    LL_ADC_EnableIT_JEOC(ADC1);
    LL_ADC_Enable(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);
    HAL_Delay(10);

    {
        __NOP();
    }
    HAL_Delay(10);

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2))
    {
        __NOP();
    }

    HAL_Delay(10);
    LL_ADC_EnableIT_JEOC(ADC2);
    LL_ADC_Enable(ADC2);
    LL_ADC_INJ_StartConversion(ADC2);
    HAL_Delay(10);
}

void spg4_adc_start(void)
{
    LL_ADC_REG_StartConversion(VBUS_ADC_INST);
}

float vbus_adc = 0;
volatile float act = 0;

const float ADC_33_4095 = 8.058608E-4F; // 3.3V / (2^12-1)

void adc_isr(void)
{
    ADC_TypeDef *adc;
    if (LL_ADC_IsActiveFlag_JEOC(ADC1))
    {
        adc = ADC1;
    }
    else if (LL_ADC_IsActiveFlag_JEOC(ADC2))
    {
        adc = ADC2;
    }
    else
    {
        return; // error
    }

    uint32_t raw = LL_ADC_INJ_ReadConversionData32(adc, LL_ADC_INJ_RANK_1);
    act = (raw & 0x0FFF) * ADC_33_4095; // 0x0FFF: 12-bits mask

    LL_ADC_ClearFlag_JEOC(adc);
    LL_ADC_INJ_StartConversion(adc); // start next conversion
}

void spg4_set_pwm(float dc_a, float dc_b, float dc_c)
{
    LL_TIM_OC_SetCompareCH1(GD_PWM_TIM_INST, GET_PWM_CCR(dc_a));
    LL_TIM_OC_SetCompareCH2(GD_PWM_TIM_INST, GET_PWM_CCR(dc_b));
    LL_TIM_OC_SetCompareCH3(GD_PWM_TIM_INST, GET_PWM_CCR(dc_c));
}

float get_vbus(void)
{
    LL_ADC_REG_StartConversion(VBUS_ADC_INST);
    uint16_t raw = LL_ADC_REG_ReadConversionData32(VBUS_ADC_INST);
    return (raw * 3.3 / 4096) * 11.0; // 12bit
}
