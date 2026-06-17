#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_log.h"
/**
 * Brief:
 * 
 * GPIO status:
 * GPIO14:  output (LED status: detect 1 ; no detect 0)
 * GPIO2:   output (LED send server)
 * GPIO34:  input, pulled up, falling/rising edge
 * GPIO35:  input, pulled up
 *
 */

#define LED_STATUS        GPIO_NUM_26
#define LED_SEND_SERVER   GPIO_NUM_2  
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<LED_STATUS) | (1ULL<<LED_SEND_SERVER))

#define SENSOR_PIN        GPIO_NUM_35
#define BUTTON_PIN        GPIO_NUM_32
#define GPIO_INPUT_PIN_SEL ((1ULL<<SENSOR_PIN) | (1ULL<<BUTTON_PIN))

#define ESP_INTR_FLAG_DEFAULT 0

#define DELAY_DETECT    3000 // Delay when sensor detect cargo cart - filter human,...

#define SENSOR_ACTIVE_LEVEL     0 // Sensor ON when IO read 0
#define SENSOR_INACTIVE_LEVEL   1

static TimerHandle_t detect_timer = NULL;
static bool sensor_confirmed = false;

static const char *TAG = "GPIO_DETECT";

typedef struct{
    bool detected;
}sensor_event_t;

static QueueHandle_t sensor_event_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // Lấy lại số GPIO đã truyền vào khi đăng ký interrupt
    uint32_t gpio_num = (uint32_t)(uintptr_t)arg;

    // Biến này để FreeRTOS biết có cần chuyển task ngay không
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Gửi số GPIO vừa gây ngắt vào queue
    // Vì đang ở ISR nên phải dùng xQueueSendFromISR, không dùng xQueueSend
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);

    // Nếu có task ưu tiên cao vừa được đánh thức,
    // yêu cầu FreeRTOS chuyển sang task đó ngay sau ISR
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void sensor_send_event(bool detected)
{
    if(sensor_event_queue == NULL)
    {
        return;
    }
    sensor_event_t event;
    event.detected = detected;

    xQueueSend(sensor_event_queue, &event, 0);
}

static void detect_timer_callback(TimerHandle_t xTimer)
{
    int level = gpio_get_level(SENSOR_PIN);

    if(level == SENSOR_ACTIVE_LEVEL)
    {
        if(sensor_confirmed == false)
        {
            sensor_confirmed = true;
            gpio_set_level(LED_STATUS,1);

            sensor_send_event(true);

            ESP_LOGI(TAG, "sensor confirmed ON after %d ms",DELAY_DETECT);
        }
    }
}

static void gpio_task(void *arg)
{
    uint32_t io_num;
    for(;;)
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if(io_num == SENSOR_PIN)
            {
                int level = gpio_get_level(SENSOR_PIN);

                if(level == SENSOR_ACTIVE_LEVEL)
                {
                    ESP_LOGI(TAG, "SENSOR ON detected, start confirm timer");

                    /*
                     * Start/restart timer 3000 ms.
                     * Trong thời gian này chưa bật LED_STATUS.
                     */

                     xTimerReset(detect_timer,0);
                }
                else{
                    ESP_LOGI(TAG, "Sensor OFF detected, cancel confirm timer");

                    /*
                     * Nếu đang đếm 3000 ms mà sensor OFF,
                     * thì hủy xác nhận.
                     */
                    xTimerStop(detect_timer,0);

                    gpio_set_level(LED_STATUS,0);

                    if(sensor_confirmed)
                    {
                        sensor_confirmed = false;

                        sensor_send_event(false);

                        ESP_LOGI(TAG, "Sensor confirmed OFF");
                    }
                    else{
                        ESP_LOGI(TAG, "Sensor OFF before confirmed ON, ignored");
                    }
                }
            }
            else if(io_num == BUTTON_PIN)
            {
                int level = gpio_get_level(BUTTON_PIN);
                ESP_LOGI(TAG, "Button interrupt, level = %d",level);

                /* Handle button */



            }
        }
    }
}

static bool gpio_init(void)
{
    gpio_config_t io_output_config = {
        // disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        // set as output mode
        .mode      = GPIO_MODE_OUTPUT,
        // bit mask of the pins that you want to set, e.g GPIO14/2
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        //disable pull-down mode
        .pull_down_en = 0,
        //disable pull-up mode
        .pull_up_en   = 0,
    };
    // configure GPIO with the given settings.
    ESP_ERROR_CHECK(gpio_config(&io_output_config));

    gpio_config_t io_input_config = {
        // interrupt of rising / falling edge
        .intr_type = GPIO_INTR_ANYEDGE,
        // set as input mode
        .mode      = GPIO_MODE_INPUT,
        // bit mask of the pins that you want to set, e.g 34/35
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        // enable pull-up mode
        .pull_up_en   = 1,
        // disable pull-down mode
        .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_input_config));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10,sizeof(uint32_t));

    sensor_event_queue = xQueueCreate(10, sizeof(sensor_event_t));

    if(sensor_event_queue == NULL)
    {
        printf("Create sensor event queue failed \n");
        return false;
    }

    if(gpio_evt_queue == NULL)
    {
        printf("Create queue failed\n");
        return false;
    }

    detect_timer = xTimerCreate(
        "detect_timer",
        pdMS_TO_TICKS(DELAY_DETECT),
        pdFALSE,
        NULL,
        detect_timer_callback
    );

    if(detect_timer == NULL)
    {
        printf("Create detect timer failed \n");
        return false;
    }
    // install gpio isr service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(SENSOR_PIN, gpio_isr_handler, (void*)(uintptr_t)SENSOR_PIN));
    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*)(uintptr_t)BUTTON_PIN));

    gpio_set_level(LED_SEND_SERVER, 0);

    return true;
}

static void sensor_event_task(void *arg)
{
    sensor_event_t event;

    for(;;)
    {
        if(xQueueReceive(sensor_event_queue, &event, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Sensor event received: detected = %d", event.detected);

            gpio_set_level(LED_SEND_SERVER,1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_SEND_SERVER,0);
        }
    }
}

void app_main(void)
{
    if(!gpio_init())
    {
        return;
    }

    gpio_set_level(LED_STATUS,0);
    gpio_set_level(LED_SEND_SERVER,0);

    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL,10, NULL);

    xTaskCreate(sensor_event_task, "sensor_event_task",2048,NULL,9,NULL);
    /*
     * Nếu lúc vừa khởi động mà sensor đã ON,
     * thì cũng phải chờ DELAY_DETECT rồi mới xác nhận.
     */
    int level = gpio_get_level(SENSOR_PIN);

    if(level == SENSOR_ACTIVE_LEVEL)
    {
        xTimerStart(detect_timer,0);
    }
}
