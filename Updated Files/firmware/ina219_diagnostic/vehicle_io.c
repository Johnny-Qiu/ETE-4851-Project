#include "vehicle_io.h"
#include "current_ina219.h"
#include "stm32f7xx.h"

/* Owns TIM3, TIM8, TIM4 and PB4/5, PC6/7, PD14/15, PF12/13.
 * TIM1 and the existing USART/IMU configuration are left to Simulink.
 * Called by the base-rate 1 ms task. No motor is enabled at initialization.
 */
static void alternate(GPIO_TypeDef *gpio, unsigned pin, unsigned af)
{
    unsigned shift = pin * 2U;
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) | (2UL << shift);
    gpio->PUPDR &= ~(3UL << shift); /* No pull-up on 5 V encoder inputs. */
    gpio->OTYPER &= ~(1UL << pin);
    gpio->AFR[pin / 8U] = (gpio->AFR[pin / 8U] & ~(15UL << ((pin % 8U)*4U)))
                          | (af << ((pin % 8U)*4U));
}
static void encoder(TIM_TypeDef *timer)
{
    timer->CR1 = 0;
    timer->PSC = 0;
    timer->ARR = 65535;
    /* Both inputs direct, digital filter fDTS/8 N=8; quadrature x4. */
    timer->CCMR1 = 0x9191;
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->SMCR = 3;
    timer->EGR = 1;
    timer->CNT = 0;
    timer->CR1 = 1;
}
/* Dedicated TIM2: PA0=CH1, PB3=CH2, AF1. No pulses until requested.
 * The current model has a 96 MHz APB1 timer clock. 1 us ticks, 20 ms frame.
 * PB3 is reserved for steering; use SWD, not SWO/full JTAG tracing.
 */
void vehicle_servo_io(double front_us, double rear_us)
{
    static uint8_t initialized = 0;
    uint32_t channels = 0;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        (void)RCC->APB1ENR;
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
        TIM2->PSC = 95;
        TIM2->ARR = 19999;
        TIM2->CCR1 = 0; TIM2->CCR2 = 0;
        TIM2->CCMR1 = (6UL<<4)|(1UL<<3)|(6UL<<12)|(1UL<<11);
        TIM2->CCER = 0;
        TIM2->EGR = 1;
        alternate(GPIOA,0,1); alternate(GPIOB,3,1);
        /* Hold the lines low while their timer channels are disabled. */
        GPIOA->PUPDR = (GPIOA->PUPDR & ~3UL) | 2UL;
        GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL<<6)) | (2UL<<6);
        TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
        initialized = 1;
    }
    if (front_us >= 1000 && front_us <= 2000) {
        if (TIM2->CCR1 != (uint32_t)front_us) { TIM2->CCR1 = (uint32_t)front_us; }
        channels |= TIM_CCER_CC1E;
    } else { if (TIM2->CCR1 != 0) { TIM2->CCR1 = 0; } }
    if (rear_us >= 1000 && rear_us <= 2000) {
        if (TIM2->CCR2 != (uint32_t)rear_us) { TIM2->CCR2 = (uint32_t)rear_us; }
        channels |= TIM_CCER_CC2E;
    } else { if (TIM2->CCR2 != 0) { TIM2->CCR2 = 0; } }
    if (TIM2->CCER != channels) { TIM2->CCER = channels; }
}
void vehicle_servo_status(uint16_t *front_us, uint16_t *rear_us, uint16_t *period_us)
{
    *front_us = (TIM2->CCER & TIM_CCER_CC1E) ? (uint16_t)TIM2->CCR1 : 0;
    *rear_us = (TIM2->CCER & TIM_CCER_CC2E) ? (uint16_t)TIM2->CCR2 : 0;
    *period_us = (uint16_t)(TIM2->ARR + 1U);
}
void vehicle_io(double duty, uint8_t enable, uint16_t *e1, uint16_t *e2)
{
    static uint8_t initialized = 0;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOFEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
        (void)RCC->APB2ENR;
        GPIOF->BSRR = (1UL << (12+16)) | (1UL << (13+16));
        GPIOF->MODER = (GPIOF->MODER & ~((3UL<<24)|(3UL<<26))) | (1UL<<24)|(1UL<<26);
        GPIOF->OTYPER &= ~((1UL<<12)|(1UL<<13));
        GPIOF->PUPDR = (GPIOF->PUPDR & ~((3UL<<24)|(3UL<<26))) | (2UL<<24)|(2UL<<26);
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM4RST;
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM4RST);
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
        alternate(GPIOB,4,2); alternate(GPIOB,5,2);
        alternate(GPIOC,6,3); alternate(GPIOC,7,3);
        encoder(TIM3); encoder(TIM8);
        /* This project's clock setup gives TIM1 and TIM4 96 MHz each:
         * HCLK=96 MHz, APB1=/2 with timer x2, APB2=/1, TIMPRE=0.
         * Reserve the top compare count so CCR=ARR+1 for true 100% fits
         * the 16-bit registers. Both timers then run at ~1464.87 Hz.
         * Recheck both timer clocks if the CubeMX clock setup changes.
         */
        if (TIM1->ARR == 65535UL) { TIM1->ARR = 65534UL; }
        TIM4->PSC = TIM1->PSC;
        TIM4->ARR = TIM1->ARR;
        TIM4->CCR3 = 0; TIM4->CCR4 = 0;
        TIM4->CCMR2 = (6UL<<4)|(1UL<<3)|(6UL<<12)|(1UL<<11);
        TIM4->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
        TIM4->EGR = 1;
        alternate(GPIOD,14,2); alternate(GPIOD,15,2);
        TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
        initialized = 1;
    }
    /* Signed duty: positive=RPWM, negative=LPWM. At most one is active.
     * The command layer enforces zero output for 500 ms before reversal.
     */
    if (!enable || !(duty >= -100.0 && duty <= 100.0)) {
        GPIOF->BSRR = (1UL<<28)|(1UL<<29);
        TIM4->CCR4 = 0; TIM4->CCR3 = 0;
    } else {
        if (duty >= 0.0) {
            TIM4->CCR3 = 0;
            TIM4->CCR4 = (uint32_t)((TIM4->ARR+1U)*duty/100.0);
        } else {
            TIM4->CCR4 = 0;
            TIM4->CCR3 = (uint32_t)((TIM4->ARR+1U)*(-duty)/100.0);
        }
        GPIOF->BSRR = (1UL<<12)|(1UL<<13);
    }
    current_ina219_tick();
    *e1 = (uint16_t)TIM3->CNT;
    *e2 = (uint16_t)TIM8->CNT;
}
