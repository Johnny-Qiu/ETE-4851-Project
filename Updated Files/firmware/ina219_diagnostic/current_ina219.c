#include "current_ina219.h"
#include "stm32f7xx.h"

/* Dedicated I2C1: PB8=SCL/PB9=SDA, AF4, open drain, external 3.3 V
 * pullups supplied by breakouts. Addresses 0x40 and 0x41. Other I2C buses
 * and the UART IMU are untouched. Fixed existing APB1 clock: 48 MHz.
 * One state transition per tick: a missing/stuck sensor never blocks control.
 * Reconfigure each sample so a brownout/reset is recovered automatically.
 * Raw shunt readings avoid dependence on the INA219 calibration register.
 */
static uint32_t ticks, deadline, next_sample, stamp[2];
static uint8_t initialized, state, sensor, stage, pos, count, receiving;
static uint8_t tx[3], rx[2], status[2];
static int16_t shunt[2], pending_shunt;
static uint16_t bus_mv[2];
static uint8_t error_code[2], recovery_step, recovery_pulses;
static uint32_t recovery_deadline;

static void pin_mode(uint32_t mode)
{
    GPIOB->OTYPER |= (1UL<<8)|(1UL<<9);
    GPIOB->PUPDR &= ~((3UL<<16)|(3UL<<18));
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFUL)|0x44UL;
    GPIOB->MODER = (GPIOB->MODER & ~((3UL<<16)|(3UL<<18)))|
                      (mode<<16)|(mode<<18);
}

static void reset_bus(void)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    /* PRESC=4: 104.17 ns ticks. SCL low=5.21 us, high=5.21 us;
     * SCLDEL=1.25 us, SDADEL=0.208 us. Conservative <100 kHz with
     * analog filtering and <=1 us rise time. Recompute if clocks change. */
    I2C1->TIMINGR = (4UL<<28)|(11UL<<20)|(2UL<<16)|(49UL<<8)|49UL;
    I2C1->CR1 = I2C_CR1_PE;
}
static void finish_sensor(void)
{
    sensor ^= 1U;
    stage = 0;
    state = 0;
    next_sample = ticks + (sensor == 0 ? 50U : 2U);
}
static void failed(void)
{
    status[sensor] = 0;
    reset_bus();
    finish_sensor();
}
/* Open-drain bus clear: release SDA, send at most nine SCL pulses, then
 * generate STOP. One edge per 1 ms call; never waits in the motor task.
 * GPIO BSRR set means release, never drive a line high against a device.
 * An absent address (NACK) does not trigger this sequence. */
static void recover_bus(void)
{
    I2C1->CR1 = 0;
    GPIOB->BSRR = (1UL<<8)|(1UL<<9);
    pin_mode(1);
    recovery_step = 1;
    recovery_pulses = 0;
    recovery_deadline = ticks + 50U;
    state = 2;
}
static void recovery_tick(void)
{
    if ((int32_t)(ticks-recovery_deadline)>=0) {
        error_code[sensor] = 9;
        GPIOB->BSRR = (1UL<<8)|(1UL<<9);
        pin_mode(2);
        failed();
        return;
    }
    switch (recovery_step) {
    case 1:
        if (!(GPIOB->IDR & (1UL<<8))) return;
        if ((GPIOB->IDR & (1UL<<9)) || recovery_pulses>=9) {
            GPIOB->BSRR = 1UL<<(8+16);
            recovery_step = 4;
        } else {
            GPIOB->BSRR = 1UL<<(8+16);
            recovery_step = 2;
        }
        break;
    case 2:
        GPIOB->BSRR = 1UL<<8;
        recovery_step = 3;
        break;
    case 3:
        if (!(GPIOB->IDR & (1UL<<8))) return;
        ++recovery_pulses;
        recovery_step = 1;
        break;
    case 4:
        GPIOB->BSRR = 1UL<<(9+16);
        recovery_step = 5;
        break;
    case 5:
        GPIOB->BSRR = 1UL<<8;
        recovery_step = 6;
        break;
    case 6:
        if (!(GPIOB->IDR & (1UL<<8))) return;
        GPIOB->BSRR = 1UL<<9;
        recovery_step = 7;
        break;
    default:
        if ((GPIOB->IDR & ((1UL<<8)|(1UL<<9))) != ((1UL<<8)|(1UL<<9)))
            error_code[sensor] = 9;
        pin_mode(2);
        failed();
        break;
    }
}
static void begin(void)
{
    receiving = 0;
    pos = 0;
    tx[0] = stage == 0 ? 0 : stage; /* configuration, shunt, bus */
    tx[1] = 0x39; tx[2] = 0x9F; /* 32 V range, +/-320 mV, 12 bit, continuous */
    /* The chip's 32 V ADC setting does NOT extend its 26 V bus rating. */
    count = stage == 0 ? 3 : 1;
    I2C1->ICR = I2C_ICR_NACKCF|I2C_ICR_STOPCF|I2C_ICR_BERRCF|I2C_ICR_ARLOCF|I2C_ICR_OVRCF;
    I2C1->CR2 = ((0x40UL+sensor)<<1)|((uint32_t)count<<16)|
        (stage == 0 ? I2C_CR2_AUTOEND : 0)|I2C_CR2_START;
    deadline = ticks + 20U;
    state = 1;
}
void current_ina219_tick(void)
{
    uint32_t flags;
    ++ticks;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        (void)RCC->APB1ENR;
        RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_I2C1SEL; /* APB1 */
        pin_mode(2);
        GPIOB->OSPEEDR |= (3UL<<16)|(3UL<<18);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFUL)|0x44UL;
        reset_bus();
        initialized = 1;
        next_sample = ticks + 100U;
        return;
    }
    if (state == 2) { recovery_tick(); return; }
    if (state == 0) {
        if ((int32_t)(ticks-next_sample) >= 0) {
            if ((I2C1->ISR & I2C_ISR_BUSY) ||
                (GPIOB->IDR & ((1UL<<8)|(1UL<<9))) != ((1UL<<8)|(1UL<<9))) {
                error_code[sensor] = 3;
                status[sensor] = 0;
                recover_bus();
            }
            else begin();
        }
        return;
    }
    flags = I2C1->ISR;
    if (flags & (I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO|I2C_ISR_OVR)) {
        error_code[sensor] = (flags & I2C_ISR_NACKF) ? 1 :
            (flags & I2C_ISR_BERR) ? 4 : (flags & I2C_ISR_ARLO) ? 5 : 6;
        failed(); return;
    }
    if ((int32_t)(ticks-deadline) >= 0) {
        error_code[sensor] = 2;
        status[sensor] = 0;
        recover_bus(); return;
    }
    if (!receiving && (flags & I2C_ISR_TXIS) && pos < count) {
        I2C1->TXDR = tx[pos++];
        return;
    }
    if (stage != 0 && !receiving && pos == count && (flags & I2C_ISR_TC)) {
        receiving = 1; pos = 0;
        I2C1->CR2 = ((0x40UL+sensor)<<1)|(2UL<<16)|I2C_CR2_RD_WRN|
            I2C_CR2_AUTOEND|I2C_CR2_START;
        return;
    }
    if (receiving && (flags & I2C_ISR_RXNE) && pos < 2) {
        rx[pos++] = (uint8_t)I2C1->RXDR;
        return;
    }
    if (flags & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPCF;
        if ((pos != count && !receiving) || (receiving && pos != 2)) {
            error_code[sensor] = 8; failed(); return;
        }
        if (stage == 1) pending_shunt = (int16_t)(((uint16_t)rx[0]<<8)|rx[1]);
        if (stage == 2) {
            uint16_t raw = ((uint16_t)rx[0]<<8)|rx[1];
            if (!(raw & 2U)) { error_code[sensor] = 7; failed(); return; }
            error_code[sensor] = 0;
            shunt[sensor] = pending_shunt;
            bus_mv[sensor] = (raw>>3)*4U;
            status[sensor] = pending_shunt >= 32000 || pending_shunt <= -32000 ? 2 : 1;
            stamp[sensor] = ticks;
            finish_sensor();
        } else {
            ++stage; state = 0;
            next_sample = ticks + (stage == 1 ? 3U : 1U);
        }
    }
}
void current_ina219_diagnostics(uint8_t *config, uint8_t *lines,
                               uint8_t *err1, uint8_t *err2)
{
    *config = ((RCC->APB1ENR & RCC_APB1ENR_I2C1EN) ? 1U : 0U) |
              ((I2C1->CR1 & I2C_CR1_PE) ? 2U : 0U) |
              (((GPIOB->MODER & (15UL<<16)) == (10UL<<16) &&
                (GPIOB->AFR[1] & 0xFFUL) == 0x44UL &&
                (GPIOB->OTYPER & (3UL<<8)) == (3UL<<8)) ? 4U : 0U);
    *lines = ((GPIOB->IDR & (1UL<<9)) ? 1U : 0U) |
             ((GPIOB->IDR & (1UL<<8)) ? 2U : 0U);
    *err1 = error_code[0]; *err2 = error_code[1];
}
void current_ina219_status(int16_t *sh1, uint16_t *bv1, uint8_t *st1,
                           int16_t *sh2, uint16_t *bv2, uint8_t *st2)
{
    *sh1 = shunt[0]; *bv1 = bus_mv[0];
    *sh2 = shunt[1]; *bv2 = bus_mv[1];
    *st1 = ticks-stamp[0] <= 250U ? status[0] : 0;
    *st2 = ticks-stamp[1] <= 250U ? status[1] : 0;
}
