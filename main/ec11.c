/* PCNT example -- Rotary Encoder

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/rmt.h"
#include "led_strip.h"

#include "ble_midi.h"
#include "key.h"


static const char *TAG = "example";
rotary_encoder_t *encoder = NULL;
led_strip_t *strip = NULL;

int ro = 0;

static uint8_t midi_vol[] = {0x88, 0x88, 0xB0 , 0x07, 0};

void ec11_task()
{
    while (1) 
    {
        ro = encoder->get_counter_value(encoder);
        // if(ro < 0) ro = 0 - ro;
        // ro = ro * 3;

        // if(ro > 0)
        // {
        //     midi_vol[4] = ro*2;
        //     MiDi_Send(midi_vol, 5);
            
        //     ESP_ERROR_CHECK(strip->set_pixel(strip, 0, ro, ro, ro));
        //     ESP_ERROR_CHECK(strip->set_pixel(strip, 1, ro, ro, ro));
        //     ESP_ERROR_CHECK(strip->set_pixel(strip, 2, ro, ro, ro));
        //     ESP_ERROR_CHECK(strip->set_pixel(strip, 3, ro, ro, ro));
        //     ESP_ERROR_CHECK(strip->refresh(strip, 100));
        //     printf("\rEncoder value: %d            \r\n", ro);
        // }
        // else ESP_ERROR_CHECK(strip->clear(strip, 100));    
        vTaskDelay(pdMS_TO_TICKS(100));

        if(midi_vol[4] >30) midi_vol[4] = 0;

        midi_vol[4] ++;
        MiDi_Send(midi_vol, 5);
    }
}

void ec11_init(void)
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t ec11_config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 13, 15);

    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&ec11_config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));


    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_14, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(4, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    ESP_ERROR_CHECK(strip->clear(strip, 100));

    xTaskCreate(ec11_task, "ec11_task", 2048, NULL, 10, NULL);

    // Report counter value
    
}
