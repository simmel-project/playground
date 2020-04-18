/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "boards.h"
#include "nrf_pwm.h"
#include "app_scheduler.h"
#include "app_timer.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

//------------- IMPLEMENTATION -------------//

void board_init(void)
{
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  // use PMW0 for LED RED
  led_pwm_init(LED_PRIMARY, LED_PRIMARY_PIN);
  #if LEDS_NUMBER > 1
  led_pwm_init(LED_SECONDARY, LED_SECONDARY_PIN);
  #endif

  // Init scheduler
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // Init app timer (use RTC1)
  app_timer_init();

  // Configure Systick for led blinky
  NVIC_SetPriority(SysTick_IRQn, 7);
  SysTick_Config(SystemCoreClock/1000);
}

void board_teardown(void)
{
  // Disable systick, turn off LEDs
  SysTick->CTRL = 0;

  // Disable and reset PWM for LEDs
  led_pwm_teardown();

  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Stop LF clock
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // make sure all pins are back in reset state
  // NUMBER_OF_PINS is defined in nrf_gpio.h
  for (int i = 0; i < NUMBER_OF_PINS; ++i)
  {
    nrf_gpio_cfg_default(i);
  }
}

static uint32_t _systick_count = 0;
void SysTick_Handler(void)
{
  _systick_count++;

  led_tick();
}


uint32_t tusb_hal_millis(void)
{
  return ( ( ((uint64_t)app_timer_cnt_get())*1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)) / APP_TIMER_CLOCK_FREQ );
}

void pwm_teardown(NRF_PWM_Type* pwm )
{
  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE            = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;
  pwm->PSEL.OUT[1] = 0xFFFFFFFF;
  pwm->PSEL.OUT[2] = 0xFFFFFFFF;
  pwm->PSEL.OUT[3] = 0xFFFFFFFF;

  pwm->MODE        = 0;
  pwm->COUNTERTOP  = 0x3FF;
  pwm->PRESCALER   = 0;
  pwm->DECODER     = 0;
  pwm->LOOP        = 0;
  pwm->SEQ[0].PTR  = 0;
  pwm->SEQ[0].CNT  = 0;
}

static uint16_t led_duty_cycles[PWM0_CH_NUM] = { 0 };

#if LEDS_NUMBER > PWM0_CH_NUM
#error "Only " PWM0_CH_NUM " concurrent status LEDs are supported."
#endif

void led_pwm_init(uint32_t led_index, uint32_t led_pin)
{
  NRF_PWM_Type* pwm    = NRF_PWM0;

  pwm->ENABLE = 0;

  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[led_index] = led_pin;

  pwm->MODE            = PWM_MODE_UPDOWN_Up;
  pwm->COUNTERTOP      = 0xff;
  pwm->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_16;
  pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  pwm->LOOP            = 0;

  pwm->SEQ[0].PTR      = (uint32_t) (led_duty_cycles);
  pwm->SEQ[0].CNT      = 4; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH  = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->ENABLE = 1;

  pwm->EVENTS_SEQEND[0] = 0;
//  pwm->TASKS_SEQSTART[0] = 1;
}

void led_pwm_teardown(void)
{
  pwm_teardown(NRF_PWM0);
}

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle)
{
  led_duty_cycles[led_index] = duty_cycle;
  nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
}

static uint32_t primary_cycle_length;
void led_tick() {
    uint32_t millis = _systick_count;

    uint32_t cycle = millis % primary_cycle_length;
    uint32_t half_cycle = primary_cycle_length / 2;
    if (cycle > half_cycle) {
        cycle = primary_cycle_length - cycle;
    }
    uint16_t duty_cycle = 0x4f * cycle / half_cycle;
    #if LED_STATE_ON == 1
    duty_cycle = 0xff - duty_cycle;
    #endif
    led_pwm_duty_cycle(LED_PRIMARY, duty_cycle);
}

void led_state(uint32_t state)
{
    switch (state) {
        case STATE_USB_MOUNTED:
          primary_cycle_length = 3000;
          break;

        case STATE_BOOTLOADER_STARTED:
        case STATE_USB_UNMOUNTED:
          primary_cycle_length = 300;
          break;

        case STATE_WRITING_STARTED:
          primary_cycle_length = 100;
          break;

        case STATE_WRITING_FINISHED:
          // Empty means to unset any temp colors.
          primary_cycle_length = 3000;
          break;

        case STATE_BLE_CONNECTED:
          primary_cycle_length = 3000;
          break;

        case STATE_BLE_DISCONNECTED:
          primary_cycle_length = 300;
          break;

        default:
        break;
    }
}
