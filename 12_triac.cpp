#include "12_triac.h"
#include <Arduino.h>

gpio_num_t triac::acdet = GPIO_NUM_1;
static triac* triac_set[5];
static uint8_t num_triac = 0;

triac::triac(gpio_num_t pin, timer_group_t grp, timer_idx_t idx) {
    this->pin = pin;
    this->grp = grp;
    this->idx = idx;
}


void triac::configACDETPIN(gpio_num_t acdet_pin) {
    triac::acdet = acdet_pin;

    gpio_config_t acdet_pin_conf = {
        .pin_bit_mask = (uint64_t)(1ULL << acdet_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&acdet_pin_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(triac::acdet, acdet_intr_handler_in_isr, NULL);
    gpio_intr_enable(triac::acdet);
}


void triac::init(void) {
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, 0);

    timer_config_t triac1_tim_conf = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
    };
    timer_init(this->grp, this->idx, &triac1_tim_conf);
    timer_set_counter_value(this->grp, this->idx, 0);
    timer_isr_callback_add(this->grp, this->idx, timer_triac_intr_handler, (void*)this, ESP_INTR_FLAG_IRAM);
    timer_enable_intr(this->grp, this->idx);

    triac_set[num_triac++] = this;
    xTaskCreate(TimerTimoutTask, "TimeOutTask", 2048, this, configMAX_PRIORITIES - 1, &TimerTimoutTaskHandle);
}


void triac::SetTimeOverFlow(uint16_t timeOverFlow) {
    if (timeOverFlow > TRIAC_HIGH_LIMIT) timeOverFlow = TRIAC_HIGH_LIMIT;
    if (timeOverFlow < TRIAC_LOW_LIMIT)  timeOverFlow = TRIAC_LOW_LIMIT;
    this->timeOverFlow = timeOverFlow;
}


void triac::TurnOnTriac() {
    this->RunStatus = true;
    digitalWrite(this->pin, 0);
}
void triac::TurnOffTriac() {
    this->RunStatus = false;
    digitalWrite(this->pin, 0);
}

void IRAM_ATTR triac::acdet_intr_handler_in_isr(void* arg) {
    if (digitalRead(triac::acdet)) return;

    for (uint8_t i = 0; i < num_triac; i++) {
        triac* ptriac = triac_set[i];

        ptriac->dis_timer = false;
        digitalWrite(ptriac->pin, 0);
        if (ptriac->RunStatus == true && ptriac->timeOverFlow < TRIAC_HIGH_LIMIT) {
            timer_group_set_alarm_value_in_isr(ptriac->grp, ptriac->idx, ptriac->timeOverFlow);
            timer_start(ptriac->grp, ptriac->idx);
        }
    }
}

void triac::acdet_intr_handler_in_task() {
    if (digitalRead(triac::acdet)) return;

    for (uint8_t i = 0; i < num_triac; i++) {
        triac* ptriac = triac_set[i];

        ptriac->dis_timer = false;
        digitalWrite(ptriac->pin, 0);
        if (ptriac->RunStatus == true && ptriac->timeOverFlow < TRIAC_HIGH_LIMIT) {
            timer_set_alarm_value(ptriac->grp, ptriac->idx, ptriac->timeOverFlow);
            timer_start(ptriac->grp, ptriac->idx);
        }
    }
}

bool IRAM_ATTR triac::timer_triac_intr_handler(void* arg) {
    if (arg == NULL) return false;
    triac* ptriac = (triac*)arg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(ptriac->TimerTimoutTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return false;
}

void triac::TimerTimoutTask(void* ptr) {
    if (ptr == NULL) return;
    triac* pTriac = (triac*)ptr;
    uint32_t notifyNum;
    while (1) {
        xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, portMAX_DELAY);
        timer_pause(pTriac->grp, pTriac->idx);
        gpio_set_level(pTriac->pin, 1);
        delayMicroseconds(100);
        gpio_set_level(pTriac->pin, 0);
    }

}