/* MCPWM basic config example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/rmt.h"
#include "led_strip.h"

#include "ble_midi.h"

#define CAP_SIG_NUM 1   //Three capture signals

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

#define GPIO_CAP0_IN   15   //Set GPIO 23 as  CAP0


typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

uint32_t *current_cap_value = NULL;
uint32_t *previous_cap_value = NULL;

led_strip_t *strip = NULL;

xQueueHandle cap_queue;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");

    mcpwm_pin_config_t pin_config = {
        .mcpwm_cap0_in_num   = GPIO_CAP0_IN,
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
    gpio_pullup_en(GPIO_CAP0_IN);    //Enable pull down on CAP0   signal
    //gpio_pullup_en(GPIO_CAP1_IN);    //Enable pull down on CAP1   signal

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((1ULL << 13) | (1ULL << 15));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

void leds_init()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_13, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(4, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    ESP_ERROR_CHECK(strip->clear(strip, 100));

    strip->set_pixel(strip, 0, 10, 10, 10); //LED逐个闪烁
    strip->refresh(strip, 100);
    vTaskDelay(20);
    strip->clear(strip, 100);

    strip->set_pixel(strip, 1, 10, 10, 10);
    strip->refresh(strip, 100);
    vTaskDelay(20);
    strip->clear(strip, 100);

    strip->set_pixel(strip, 2, 10, 10, 10);
    strip->refresh(strip, 100);
    vTaskDelay(20);
    strip->clear(strip, 100);

    strip->set_pixel(strip, 3, 10, 10, 10);
    strip->refresh(strip, 100);
    vTaskDelay(20);
    strip->clear(strip, 100);


}
static uint8_t midi_vol[] = {0x88, 0x88, 0xB0 , 0x07, 0};

/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
static void disp_captured_signal(void *arg)
{
    capture evt;
    BaseType_t errcode = 0;

    uint32_t data_queue[30] = { 0 };
    uint32_t data_sum = 0;
    int data_c = 0;
    TickType_t last_time = 0;

    uint vol = 1;

    vTaskDelay(50);
    leds_init();

    uint8_t leds_value[12] = { 0 };

    while (1) 
    {
        errcode = xQueueReceive(cap_queue, &evt, 25/portTICK_PERIOD_MS);
        if(errcode == pdTRUE)
        { 
            data_queue[data_c] = evt.capture_signal;
            data_c ++;
            if((data_c == 10) || (xTaskGetTickCount()-last_time) > 10)
            if((data_c == 10))
            {
                data_sum = 0;
                for(int i  =0; i<data_c; i++)
                {
                    data_sum+=data_queue[i];
                }
                data_sum = data_sum / data_c;

                data_c = 0;
                last_time = xTaskGetTickCount();

                printf("CAP0 : %d us\n", data_sum);
            
                data_c = 0;

                data_sum = (100*3000)/data_sum ;
                if(data_sum > 125) data_sum = 125;
                printf("VOL : %d us\n", data_sum);
                midi_vol[4] = data_sum;

                if(midi_vol[4] != vol) 
                {
                    MiDi_Send(midi_vol, 5);
                }
                vol = data_sum;

                esp_fill_random(leds_value, 12);
                if(midi_vol[4]  == 0) midi_vol[4] = 255;
                for(int i = 0;i<12;i++) leds_value[i] %= midi_vol[4] ;

                ESP_ERROR_CHECK(strip->set_pixel(strip, 0, leds_value[0], leds_value[1], leds_value[2]));
                ESP_ERROR_CHECK(strip->set_pixel(strip, 1, leds_value[3], leds_value[4], leds_value[5]));
                ESP_ERROR_CHECK(strip->set_pixel(strip, 2, leds_value[6], leds_value[7], leds_value[8]));
                ESP_ERROR_CHECK(strip->set_pixel(strip, 3, leds_value[9], leds_value[10], leds_value[11]));

                ESP_ERROR_CHECK(strip->refresh(strip, 100));
                
            }
        }
        else
        {
            if(vol != 0)
            {
                midi_vol[4] = 0;
                MiDi_Send(midi_vol, 5);
                vol=0;
                ESP_ERROR_CHECK(strip->clear(strip, 100));
           }
        }
    }
}

/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void)
{
    uint32_t mcpwm_intr_status;
    capture evt;

    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    if (mcpwm_intr_status & CAP0_INT_EN) //Check for interrupt on rising edge on CAP0 signal
    { 
        current_cap_value[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.capture_signal = (current_cap_value[0] - previous_cap_value[0]) / (rtc_clk_apb_freq_get() / 1000000);
        previous_cap_value[0] = current_cap_value[0];
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;

        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}

/**
 * @brief Configure whole MCPWM module
 */
static void mcpwm_example_config(void *arg)
{
    mcpwm_example_gpio_initialize();

    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second

    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    vTaskDelete(NULL);
}

void pwm_c_init(void)
{
    printf("Testing MCPWM...\n");
    cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module
    current_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t)); //comment if you don't want to use capture module
    previous_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t));  //comment if you don't want to use capture module
    xTaskCreate(disp_captured_signal, "mcpwm_config", 8192, NULL, 6, NULL);  //comment if you don't want to use capture module
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
}
 