#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "protocol_examples_common.h"

#define INPUT_PORT CONFIG_INPUT_PORT
#define SENSOR_HIGH 1
#define SENSOR_LOW 0
#define GPIO_OUTPUT_PIN_SEL (1ULL << INPUT_PORT)
#define TIMER_DIVIDER 2                              //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERVAL0_SEC (2)                      // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC (0.02)                   // sample test interval for the second timer
#define INCLUDE_vTaskDelete					1
#define OFFSET 4000

static const char *TAG_WIFI = "WIFI";
static const char *TAG = "READER";
static const char *TAG_TIM = "TIMER";
static const char *TAG_SEN = "SENSOR";
// static const char *TAG_ISEN = "SENSOR_INT";
static const char *TAG_DATA = "DATA";
// static int timeCounter = 0;
uint64_t timeCounter[42];
static int pulseCounter = 0;
TaskHandle_t loopTask = NULL;
int readingCount = 0;
uint64_t measure = 0ULL;

struct Measure {
    unsigned char hum_int;
    unsigned char hum_dec;
    unsigned char temp_int;
    unsigned char temp_dec;
    unsigned char cheksum;
};

typedef struct Measure temp_t;

static void SetSensorLevel(void *level);

static void ConfigSensorPort(void *itrType);

static void ConfigTimers(int timer_idx, double timer_interval_sec, void (*handler)());

static void IntrHandler(void *unused);

static void IRAM_ATTR LongTimerHandler(void *unused);

static void IRAM_ATTR ShortTimerHandler(void *unused);

static void InitTimers(void *unused);

static void InitDataArray(void *unused);

static void PrintDataArray(void *unused);

static void InitSensorTimer(void *unused);

static void ConnectToWiFi(void *unused);

int app_main(void)
{
    ESP_LOGI(TAG, "Starting routine!!!\n");
    // xTaskCreate(&ConnectToWiFi, "ConnectToWiFi", 8192, NULL, 3, NULL);
    xTaskCreate(&ConfigSensorPort, "ConfigSensor", 8192, GPIO_INTR_DISABLE, 3, NULL);
    // // Installing ISR Handler Service for GPIO interrupts
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(INPUT_PORT, *IntrHandler, NULL);
    ESP_LOGI(TAG, "Initial level = %d", gpio_get_level(INPUT_PORT));
    xTaskCreate(&InitDataArray, "InitDataArray", 8192, NULL, 3, NULL);
    xTaskCreate(&InitSensorTimer, "InitSensorTimer", 8192, NULL, 3, NULL);
    xTaskCreate(&InitTimers, "InitTimers", 8192, NULL, 3, NULL);
    xTaskCreate(&SetSensorLevel, "SetSensorLevel", 8192, (int*)SENSOR_HIGH, 3, NULL);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    timer_start(TIMER_GROUP_0, TIMER_0);
    // xTaskCreate(&Loop, "Loop", 8192, NULL, 3, loopTask);
    // esp_task_wdt_add(loopTask);
    while (1)
    {
  
    }

    return 0;
}

static void ConnectToWiFi(void *unused)
{
// Initialises partition of Non Volatile Storage(NVS)

    esp_err_t ret = nvs_flash_init();
    /* 
     * Test response of previous step wheter nvs has no free space or contains data 
     * in a different format that cannot be recognized
     * */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // Clear nvs partition and logs error in case erasing could not succed
      ESP_ERROR_CHECK(nvs_flash_erase());
      // Finally initilises NVS partition
      ret = nvs_flash_init();
    }
    // // Logs result of previous step and stop program if it is an error
    // ESP_ERROR_CHECK(ret);
    // // Initialises TCP/IP stack and if there is an error, logs it and ends the program
    // ESP_ERROR_CHECK(esp_netif_init());
    // /* Creates default event loop for system events 
    // * and if there is an error, logs it and ends the program
    // * */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());
    /*
    * Logs information about connection
    */
    // ESP_LOGI(TAG_WIFI, "Connected to AP, begin http example");
    vTaskDelete(NULL);
}

static void PrintDataArray(void * unused)
{
    temp_t comp_meas;
    measure = 0ULL;
    if(readingCount > 0)
    {
        measure |= 1;
        measure = measure << 24;
        for (int i = 2; i < 42; i++)
        {
            // ESP_LOGI(TAG_DATA, "timeCounter.%i = %" PRIu64, i, timeCounter[i]);
            measure = measure<<1;
            if((timeCounter[i] - timeCounter[i - 1]) < OFFSET) {
                measure |= 0;
            }
            else 
            {
                measure |= 1;
            }
        }
        comp_meas.cheksum = (0x00 | measure);
        comp_meas.temp_dec = (0x00 | (measure>>8));
        comp_meas.temp_int = (0x00 | (measure>>16));
        comp_meas.hum_dec = (0x00 | (measure>>24));
        comp_meas.hum_int = (0x00 | (measure>>32));
        if(comp_meas.cheksum == ((int)comp_meas.hum_dec + (int)comp_meas.hum_int + (int)comp_meas.temp_dec + (int)comp_meas.temp_int)){
            ESP_LOGI(TAG_DATA, "Temperature: %i.%i", (int)comp_meas.temp_int, (int)comp_meas.temp_dec);    
            ESP_LOGI(TAG_DATA, "Humidity: %i.%i", (int)comp_meas.hum_int, (int)comp_meas.hum_dec);    
            ESP_LOGI(TAG_DATA, "Checksum: %i", (int)comp_meas.cheksum);    
        }
        else 
        {
            ESP_LOGI(TAG_DATA, "Error: Wrong data.");    
        }
    }
    ESP_LOGI(TAG, "························END OF READING %i························", readingCount);
    readingCount ++;
    vTaskDelete(NULL);
}

static void InitDataArray(void *unused)
{
    for (int i = 0; i < 42; i++)
    {
        timeCounter[i] = 1ULL;
    }
    vTaskDelete(NULL);
}

static void InitTimers(void *unused)
{
    ConfigTimers(TIMER_0, TIMER_INTERVAL0_SEC, LongTimerHandler);
    ConfigTimers(TIMER_1, TIMER_INTERVAL1_SEC, ShortTimerHandler);
    vTaskDelete(NULL);
}

static void IRAM_ATTR LongTimerHandler(void *unused)
{
    ESP_ERROR_CHECK(timer_spinlock_take(TIMER_GROUP_0));
    timer_pause(TIMER_GROUP_0, TIMER_0);
    xTaskCreate(&PrintDataArray, "PrintDataArray", 8192, NULL, 3, NULL);
    gpio_intr_disable(INPUT_PORT);
    xTaskCreate(&SetSensorLevel, "SetSensorLevel", 8192, (int*)SENSOR_LOW, 2, NULL);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_1);
    ESP_ERROR_CHECK(timer_spinlock_give(TIMER_GROUP_0));
}

static void IRAM_ATTR ShortTimerHandler(void *unused)
{
    timer_spinlock_take(TIMER_GROUP_0);
    pulseCounter = 0;
    gpio_intr_enable(INPUT_PORT);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
    
    timer_pause(TIMER_GROUP_0, TIMER_1);
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
    timer_start(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_1, TIMER_0);

    xTaskCreate(&SetSensorLevel, "SetSensorLevel", 8192, (int*)SENSOR_HIGH, 2, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void ConfigTimers(int timer_idx, double timer_interval_sec, void (*handler)())
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL));

    /* Configure the alarm value and the interrupt on alarm. */
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, timer_idx));
    ESP_ERROR_CHECK_WITHOUT_ABORT(timer_isr_register(TIMER_GROUP_0, timer_idx, handler,
                                                     NULL, ESP_INTR_FLAG_IRAM, NULL));

    ESP_LOGI(TAG_TIM, "Timer %i configured succesfully!", timer_idx);
}

static void IntrHandler(void *unused)
{
    timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &timeCounter[pulseCounter]);
    pulseCounter++;
}

static void ConfigSensorPort(void *itrType)
{
    //GPIO port configuration structure
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // io_conf.intr_type = (int)itrType;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    //bit mask of the pins that you want to set,e.g.GPIO2/5
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //set port high as standby value
    gpio_set_level(INPUT_PORT, 1);
    ESP_ERROR_CHECK(gpio_intr_disable(INPUT_PORT));
    ESP_LOGI(TAG_SEN, "Sensor configured!");
    vTaskDelete(NULL);
}

static void SetSensorLevel(void *level)
{
    gpio_set_level(INPUT_PORT, (int)level);
    // ESP_LOGI(TAG_SEN, "Port level = %d.", gpio_get_level(INPUT_PORT));
    vTaskDelete(NULL);
}

static void InitSensorTimer(void *unused)

{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = TIMER_AUTORELOAD_DIS
    }; // default clock source is APB
    timer_init(TIMER_GROUP_1, TIMER_0, &config);
    ESP_LOGI(TAG_TIM, "Sensor timer configured succesfully!");
    vTaskDelete(NULL);
}

// static void Loop(void *unused) 
// {
//     while (1)
//     {
//         esp_task_wdt_reset();
//     }
// }