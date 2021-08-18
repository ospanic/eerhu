

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "ble_midi.h"

uint8_t midi_on[] = {0x80, 0x80, 0x90 , 0x3B, 0x63};
uint8_t midi_off[] = {0x80, 0x80, 0x80 , 0x3B, 0x63};

uint8_t tune_start = 0x32;
uint8_t tune_q[] = {0,2,4,5,7,9,10,12,9,10,12,14,11,17,19,21};
uint8_t key_q[] = { 19,21,22,23,4,17,5,18,26,25,33,32,2,12,14,27};
uint8_t key_status[16] = { 0 };

#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
} 

void Key_task()
{
    uint32_t io_num;
    for(;;) 
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
        {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(key_q[io_num]));

            if(io_num == 0xFF) //转调按键
            {
                if(tune_start == 0x32) tune_start +=12;
                else tune_start = 0x32;
                continue;
            }
            if(gpio_get_level(key_q[io_num]) == 0) //按键按下
            {
                if(key_status[io_num] == 0)
                {
                    midi_on[3] = tune_start + tune_q[io_num];
                    MiDi_Send(midi_on,5);
                    key_status[io_num] = 1;
                    ESP_LOGE("KEY", "Down Down");
                }
            }
            else 
            {
                if(key_status[io_num] == 1)
                {
                    midi_off[3] = tune_start + tune_q[io_num];
                    MiDi_Send(midi_off,5);
                    key_status[io_num] = 0;  
                }
            }
        }
    }
}

void key_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = ((1ULL << key_q[0]) | (1ULL << key_q[1] ) |
                            (1ULL << key_q[2]) | (1ULL << key_q[3] ) |
                            (1ULL << key_q[4]) | (1ULL << key_q[5] ) |
                            (1ULL << key_q[6]) | (1ULL << key_q[7] ) |
                            (1ULL << key_q[8]) | (1ULL << key_q[9] ) |
                            (1ULL << key_q[10]) | (1ULL << key_q[11] ) |
                            (1ULL << key_q[12]) | (1ULL << key_q[13] ) |
                            (1ULL << key_q[14]) | (1ULL << key_q[15] ) );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE; //转调按键 IO0
    io_conf.pin_bit_mask = (1ULL); //IO0
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(32, sizeof(uint32_t));

    
    xTaskCreate(Key_task, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    for(int i =0;i<16;i++)
    {
        gpio_isr_handler_add(key_q[i], gpio_isr_handler, (void*) i);
    }

    int i=0xFF; //转调按键
    gpio_isr_handler_add(0, gpio_isr_handler, (void*)i);
}