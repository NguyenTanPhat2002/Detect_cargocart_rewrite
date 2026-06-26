#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "my_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"


#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

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

#define ESP_INTR_FLAG_DEFAULT 0

#define DELAY_DETECT    3000 // Delay when sensor detect cargo cart - filter human,...

#define SENSOR_ACTIVE_LEVEL     0 // Sensor ON when IO read 0
#define SENSOR_INACTIVE_LEVEL   1

#define SENSOR_ID   0x0001
#define DEFAULT_TIMESTAMP   0x00000000

#define WIFI_SSID "PhatBinh"
#define WIFI_PASS "19752002"
#define SERVER_IP "192.168.1.54"
#define SERVER_PORT 5000

#define WIFI_MAX_RETRY  5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT   BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num =0;

static TimerHandle_t detect_timer = NULL;
static bool sensor_confirmed = false;

static const char *TAG = "GPIO_DETECT";

typedef struct{
    bool detected;
}sensor_event_t;

typedef struct
{
    uint8_t data[TX_FRAME_MAX_SIZE];
    size_t len;
    uint8_t msg_type;
}tx_frame_t;

static QueueHandle_t tx_frame_queue = NULL;
static QueueHandle_t sensor_event_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

static void event_handler(void *arg, esp_event_base_t event_base, 
                            int32_t event_id, void *event_data)
{
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if(s_retry_num < WIFI_MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num ++;
            ESP_LOGI(TAG, "retry to connect wifi %s",WIFI_SSID);
        }
        else{
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }

    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num =0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    if(s_wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Create WiFi event group failed");
        return false;
    }
    
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id
    ));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        NULL,
        &instance_got_ip
    ));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Wait until 1 or 2 this bit is setted: 
        1. WIFI_CONNECTED_BIT
        2. WIFI_FAIL_BIT */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, 
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);
    
    if(bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to WIFI");
        return true;
    }
    else if(bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect WIFI");
        return false;
    }
    ESP_LOGE(TAG, "Unexpected WiFi event");
    return false;
}

static void log_hex_frame(const uint8_t *buf, size_t len)
{
    if(buf == NULL || len ==0)
    {
        ESP_LOGW(TAG, "Empty frame");
        return;
    }

    char line[3*256 +1];
    size_t pos = 0;

    line[0] = '\0';

    for(size_t i=0; i < len && pos < sizeof(line) -4; i++)
    {
        int written = snprintf(&line[pos], sizeof(line) -pos, "%02X ", buf[i]);

        if(written < 0 )
        {
            break;
        }

        pos += (size_t) written;
    }

    ESP_LOGI(TAG, "FRAME[%d]: %s", (int)len, line);
}


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

    if(ret != pdTRUE)
    {
        ESP_LOGW(TAG, "sensor_event_queue full, event lost");
    }
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

    gpio_config_t sensor_input_config = {
        // interrupt of rising / falling edge
        .intr_type = GPIO_INTR_ANYEDGE,
        // set as input mode
        .mode      = GPIO_MODE_INPUT,
        // bit mask of the pins that you want to set, e.g 34/35
        .pin_bit_mask = (1ULL << SENSOR_PIN),
        // enable pull-up mode
        .pull_up_en   = 0,
        // disable pull-down mode
        .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&sensor_input_config));

    gpio_config_t button_input_config = {
        // interrupt of rising / falling edge
        .intr_type = GPIO_INTR_ANYEDGE,
        // set as input mode
        .mode      = GPIO_MODE_INPUT,
        // bit mask of the pins that you want to set, e.g 34/35
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        // enable pull-up mode
        .pull_up_en   = 1,
        // disable pull-down mode
        .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&button_input_config));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10,sizeof(uint32_t));

    sensor_event_queue = xQueueCreate(10, sizeof(sensor_event_t));

    tx_frame_queue = xQueueCreate(10,sizeof(tx_frame_t));

    if(sensor_event_queue == NULL)
    {
        ESP_LOGE(TAG, "Create sensor event queue failed");
        return false;
    }

    if(gpio_evt_queue == NULL)
    {
        ESP_LOGE(TAG, "Create gpio event queue failed");
        return false;
    }

    if(tx_frame_queue == NULL)
    {
        ESP_LOGE(TAG, "Create tx frame queue failed");
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
        ESP_LOGE(TAG, "Create detect timer failed \n");
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

static void tx_task(void *arg)
{
    tx_frame_t frame;

    for(;;)
    {
        if(xQueueReceive(tx_frame_queue, &frame, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "TX task received frame, msg_type = 0x%02X, len = %d", 
                            frame.msg_type, (int)frame.len);

            log_hex_frame(frame.data, frame.len);

            gpio_set_level(LED_SEND_SERVER,1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_SEND_SERVER,0);
        }
    }
}

static bool tx_send_frame_to_queue(const uint8_t *frame, size_t frame_len, uint8_t msg_type)
{
    if(tx_frame_queue == NULL || frame == NULL || frame_len ==0)
    {
        return false;
    }

    if(frame_len > TX_FRAME_MAX_SIZE)
    {
        ESP_LOGE(TAG, "Frame too large, len = %d",(int)frame_len);
        return false;
    }

    tx_frame_t tx_frame;

    memcpy(tx_frame.data, frame, frame_len);
    tx_frame.len = frame_len;
    tx_frame.msg_type = msg_type;

    BaseType_t ret = xQueueSend(tx_frame_queue, &tx_frame, 0);

    if(ret != pdTRUE)
    {
        ESP_LOGW(TAG, "tx_frame_queue full, frame lost");
        return false;
    }
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

            uint8_t tx_buf[256];
            size_t tx_len = 0;

            bool ok = my_protocol_build_sensor_data_digital(
                SENSOR_ID,
                event.detected,
                DEFAULT_TIMESTAMP,
                tx_buf,
                sizeof(tx_buf),
                &tx_len
            );

            if(ok)
            {
                ESP_LOGI(TAG, "Build SENSOR_DATA frame OK, tx_len = %d", (int)tx_len);

                tx_send_frame_to_queue(tx_buf, tx_len, MSG_SENSOR_DATA);
            }
            else{
                ESP_LOGE(TAG, "Build SENSOR_DATA frame failed");
            }
        }
    }
}

static void heartbeat_task(void *arg)
{
    for(;;)
    {
        uint8_t tx_buf[256];
        size_t tx_len =0;

        uint32_t uptime_sec = xTaskGetTickCount() / configTICK_RATE_HZ;

        bool ok = my_protocol_build_heartbeat(
            SENSOR_ID,
            uptime_sec, 
            BATTERY_WIRED,
            DEFAULT_TIMESTAMP,
            tx_buf,
            sizeof(tx_buf),
            &tx_len
        );

        if(ok)
        {
            ESP_LOGI(TAG, "Build HEARTBEAT OK, uptime = %lu, len =%d",
                        (unsigned long)uptime_sec,
                        (int)tx_len);

            tx_send_frame_to_queue(tx_buf, tx_len, MSG_HEARTBEAT);
        }
        else{
            ESP_LOGE(TAG, "Build HEARTBEAT failed");
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

static bool nvs_init_app(void)
{
    esp_err_t ret = nvs_flash_init();

    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS init failed");
        return false;
    }

    return true;
}
static int tcp_connect_server(void)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if(sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket, erro = %d", errno);
        return -1;
    }

    ESP_LOGI(TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if(err !=0 )
    {
        ESP_LOGE(TAG, "Socket connect faied, errno = %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP connected to server");

    return sock;
}

void app_main(void)
{
    if(!nvs_init_app())
    {
        return;
    }

    if(!wifi_init_sta())
    {
        ESP_LOGE(TAG, "WIFI failed, stop app");
        return;
    }

    int sock = tcp_connect_server();

    if(sock < 0)
    {
        ESP_LOGE(TAG, "TCP connect faied, stop app");
        return;
    }

    close(sock);
    
    if(!gpio_init())
    {
        return;
    }

    gpio_set_level(LED_STATUS,0);
    gpio_set_level(LED_SEND_SERVER,0);

    xTaskCreate(gpio_task, "gpio_task_example", 4096, NULL,10, NULL);

    xTaskCreate(sensor_event_task, "sensor_event_task",4096,NULL,9,NULL);
    
    xTaskCreate(heartbeat_task, "heartbeat task", 4096, NULL, 8, NULL);

    xTaskCreate(tx_task, "tx_task", 4096, NULL, 8, NULL);

    /*
     * Nếu lúc vừa khởi động mà sensor đã ON,
     * thì cũng phải chờ DELAY_DETECT rồi mới xác nhận.
     */

    uint8_t buff_status[256];
    size_t status_len =0;

    bool status_ok = my_protocol_build_status(SENSOR_ID, SENSOR_STATUS_ON,
                                                DEFAULT_TIMESTAMP, buff_status,
                                                sizeof(buff_status), &status_len);
                                            
    if(status_ok)
    {
        ESP_LOGI(TAG, "Build status online OK, len = %d", (int)status_len);
        tx_send_frame_to_queue(buff_status, status_len, MSG_SENSOR_STATUS);
    }
    else{
        ESP_LOGE(TAG, "Build status online failed");
    }

    int level = gpio_get_level(SENSOR_PIN);

    if(level == SENSOR_ACTIVE_LEVEL)
    {
        xTimerStart(detect_timer,0);
    }
}
