

#ifndef _TRIAC_H_
#define _TRIAC_H_

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/timer.h"


#define TRIAC_LOW_LIMIT  2000
#define TRIAC_HIGH_LIMIT 9600
#define TRIAC_HOLD_TIME  100


class triac {
public:
    triac(gpio_num_t pin, timer_group_t grp = TIMER_GROUP_0, timer_idx_t idx = TIMER_0);

    static void configACDETPIN(gpio_num_t acdet_pin = GPIO_NUM_1);
    void init(void);
    void SetTimeOverFlow(uint16_t timeOverFlow = 9600);
    void TurnOnTriac();
    void TurnOffTriac();

    static gpio_num_t acdet;
    gpio_num_t pin = GPIO_NUM_1;

    timer_group_t grp = TIMER_GROUP_0;
    timer_idx_t   idx = TIMER_0;

    bool dis_timer = false;
    uint16_t timeOverFlow = TRIAC_HIGH_LIMIT;
    bool RunStatus = false;

    TaskHandle_t TimerTimoutTaskHandle;
    static void TimerTimoutTask(void* ptr);

    void acdet_intr_handler_in_task();
    static void IRAM_ATTR acdet_intr_handler_in_isr(void* arg);
    static bool IRAM_ATTR timer_triac_intr_handler(void* arg);
private:

};

#endif 