/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Damjan Marion <damjan.marion@gmail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define LGREENF GPIO13
#define LGREENF_PORT GPIOG
#define LREDF GPIO14
#define LREDF_PORT GPIOG

#define LGREENB GPIO13
#define LGREENB_PORT GPIOB
#define LREDB GPIO5
#define LREDB_PORT GPIOC

#define LCC LREDF_PORT, LREDF
#define LUP LREDB_PORT, LREDB

/*
  Timer 1 clk frequency:
  If TIMPRE == 0: (default)
    if PPRE = DIV1:
      TIM1CLK = PCLK
    else:
      TIM1CLK = 2*PCLK
  else:
    if PPRE = DIV1|DIV2|DIV4:
      TIM1CLK = HCLK
    else:
      TIM1CLK = 4*PCLK
 */


/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    // AHB Prescaler = 1 (NODIV) libopencm3/lib/stm32/f4/rcc.c
    // APB1: DIV4
    // APB2: DIV2
    // SYSCLK: 168MHz
    // AHBCLK (HCLK or FCLK): SYSCLK/AHBPre = 168MHz/1
    // APB1: 168/4 = 42MHz
    // APB2: 168/2 = 84MHz
    // rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency
    // are set here (libopencm3/lib/stm32/f4/rcc.c)
    // TIM1 is on APB2
    // TIMPRE: arriba. Default 0
    // TIM1CLK = 2*PCLK = 2*84MHz
    // TIM1CNT = 168MHz/2^16   (TIM1CLK/(PRESCALER+1))
    // TIM1CNT = 2563.48Hz
    // If TIMPRE == 0 and PCLK > DIV1:
    // FCNT = 2*PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 0 and PCLK == DIV1:
    // FCNT = PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 1 and PCLK > DIV4:
    // FCNT = 4*PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 1 and PCLK <= DIV4:
    // FCNT = HCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)

	/* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM1);
}

static void gpio_setup(void)
{
	/* Set GPIO13-14 (in GPIO port G) to 'output push-pull'. */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO13 | GPIO14);

	/* Set GPIO5 (in GPIO port C) to 'output push-pull'. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO5);

    /* Set GPIOB13 as AF1 */
    gpio_set_af(LGREENB_PORT, GPIO_AF1, LGREENB);

	/* Set GPIO13 (in GPIO port B) to 'alternate function push-pull'. */
	gpio_mode_setup(LGREENB_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, LGREENB);


}

// ----------- Variable global -----------
static uint32_t g_arr = 0;

// Convierte porcentaje a valor de pulso en ticks
static uint32_t calcular_pulse(float porcentaje_duty, uint32_t arr) {
    return (uint32_t)((porcentaje_duty / 100.0f) * (arr + 1));
}

// Calcula el valor de ARR para un tiempo dado en segundos
static uint32_t calcular_arr(float tiempo_segundos, uint32_t clk_hz, uint32_t prescaler) {
    float f_timer = clk_hz / (float)(prescaler + 1);
    return (uint32_t)(f_timer * tiempo_segundos - 1);
}

static void tim_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM1);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);

    rcc_periph_reset_pulse(RST_TIM1);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM1, 0x00FF);  // 255
    float tiempo_original = 0.020f;     // 20 ms
    
    // Guardamos el ARR global para usarlo luego
    g_arr = calcular_arr(tiempo_original, 168000000, 0x00FF);
    timer_set_period(TIM1, g_arr);

    timer_disable_preload(TIM1);
    timer_continuous_mode(TIM1);
    
    timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(5.0f, g_arr));

    timer_enable_oc_output(TIM1, TIM_OC1); // no OC1N
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);

    timer_enable_break_main_output(TIM1);
    timer_disable_break(TIM1);

    timer_enable_counter(TIM1);

    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
    timer_enable_irq(TIM1, TIM_DIER_UIE);
}

void tim1_cc_isr(void)
{
    timer_clear_flag(TIM1, TIM_SR_CC1IF);
    gpio_toggle(LCC);
}

void tim1_up_tim10_isr(void)
{
    timer_clear_flag(TIM1, TIM_SR_UIF);
    gpio_toggle(LUP);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    tim_setup();

    while (1) {
        // 5% duty cycle (~1 ms) → 2 segundos
        timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(5.0f, g_arr));
        for (volatile int i = 0; i < 16000000; ++i) { __asm__("nop"); }

        // 10% duty cycle (~2 ms) → 1 segundo
        timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(10.0f, g_arr));
        for (volatile int i = 0; i < 8000000; ++i) { __asm__("nop"); }

        // 100% duty cycle (20 ms) → 3 segundos
        timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(100.0f, g_arr));
        for (volatile int i = 0; i < 24000000; ++i) { __asm__("nop"); }

        // 50% duty cycle (10 ms) → 1 segundo
        timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(50.0f, g_arr));
        for (volatile int i = 0; i < 8000000; ++i) { __asm__("nop"); }

        // 10% duty cycle (2 ms) → 5 segundos
        timer_set_oc_value(TIM1, TIM_OC1, calcular_pulse(10.0f, g_arr));
        for (volatile int i = 0; i < 40000000; ++i) { __asm__("nop"); }
    }

    return 0;
}
