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

/* ===== Pines del board ===== */
#define LGREENF GPIO13
#define LGREENF_PORT GPIOG
#define LREDF GPIO14
#define LREDF_PORT GPIOG
#define LREDB GPIO5
#define LREDB_PORT GPIOC
#define LGREENB GPIO13
#define LGREENB_PORT GPIOB

#define LCC LREDF_PORT, LREDF
#define LUP LREDB_PORT, LREDB

/* ===== Variables globales ===== */
volatile uint32_t tim1_period_count = 0;

/* ===== Configuración de reloj ===== */
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOG);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_TIM1);
}

/* ===== Configuración GPIO ===== */
static void gpio_setup(void)
{
    // LEDs
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    // PB13 como salida PWM (AF1 = TIM1_CH1N)
    gpio_mode_setup(LGREENB_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LGREENB);
    gpio_set_af(LGREENB_PORT, GPIO_AF1, LGREENB);
}

/* ===== Configuración del Timer 1 ===== */
static void tim_setup(void)
{
    nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);

    rcc_periph_reset_pulse(RST_TIM1);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 0x00FF);  // Prescaler = 255 → f_timer = 168 MHz / 256 = 656.25 kHz
    timer_set_period(TIM1, 13124);      // Periodo ≈ 20 ms (50 Hz PWM)

    // Configuración PWM canal 1
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_preload(TIM1);

    // Valor inicial: 5% duty
    timer_set_oc_value(TIM1, TIM_OC1, 656);

    timer_enable_break_main_output(TIM1);
    timer_enable_irq(TIM1, TIM_DIER_UIE);

    // Importante: generar evento de actualización inicial
    timer_generate_event(TIM1, TIM_EGR_UG);

    timer_enable_counter(TIM1);
}

/* ===== ISR: interrupción de update ===== */
void tim1_up_tim10_isr(void)
{
    timer_clear_flag(TIM1, TIM_SR_UIF);
    tim1_period_count++;
    gpio_toggle(LUP); // LED visual cada 20 ms
}

/* ===== Delay no bloqueante basado en periodos del timer ===== */
static void wait_periods(uint32_t periods)
{
    uint32_t start = tim1_period_count;
    while ((tim1_period_count - start) < periods)
        __asm__("nop");
}

/* ===== MAIN ===== */
int main(void)
{
    clock_setup();
    gpio_setup();
    tim_setup();

    gpio_set(LGREENF_PORT, LGREENF); // LED referencia encendido

    while (1)
    {
        // 0% (2 s) -> duty ~5%
        timer_set_oc_value(TIM1, TIM_OC1, 656);
        wait_periods(100); // 2 s

        // 10% (1 s)
        timer_set_oc_value(TIM1, TIM_OC1, 721);
        wait_periods(50); // 1 s

        // 100% (3 s)
        timer_set_oc_value(TIM1, TIM_OC1, 13124);
        wait_periods(150); // 3 s

        // 50% (1 s)
        timer_set_oc_value(TIM1, TIM_OC1, 984);
        wait_periods(50); // 1 s

        // 10% (5 s)
        timer_set_oc_value(TIM1, TIM_OC1, 721);
        wait_periods(250); // 5 s
    }

    return 0;
}
