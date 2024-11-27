// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <esp_system.h>
// #include <esp_log.h>
// #include <driver/gpio.h>
// #include <icm42670.h>
// #include "esp_event.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"
// #include "esp_wifi.h"
// #include "esp_http_server.h"
// #include "esp_system.h"
// #include <time.h>


// static const char *TAG = "icm42670";
// static const char *TAGS = "wifi softAP";


// #define EXAMPLE_ESP_WIFI_SSID "bioSTEP+"
// #define EXAMPLE_ESP_WIFI_PASS "biostim.in"
// #define EXAMPLE_ESP_WIFI_CHANNEL 1
// #define EXAMPLE_MAX_STA_CONN 4


// #define PORT 0

// #if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
// #define I2C_ADDR ICM42670_I2C_ADDR_GND
// #endif
// #if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
// #define I2C_ADDR ICM42670_I2C_ADDR_VCC
// #endif

// #ifndef APP_CPU_NUM
// #define APP_CPU_NUM PRO_CPU_NUM
// #endif
// static int16_t lin_accel_y = 0;
// int thres = -325;

// esp_err_t number_post_handler(httpd_req_t *req)
// {
//     char response[64];
//     snprintf(response, sizeof(response), "{\"sensor\":%d, \"threshold\":%d}", lin_accel_y, thres);
//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
//     ESP_LOGI(TAG, "Sent JSON: %s", response);
//     return ESP_OK;
// }



// // HTTP POST handler to receive messages
// esp_err_t message_post_handler(httpd_req_t *req) {
//     // Check the Content-Type header to ensure it’s text/plain
//     if (httpd_req_get_hdr_value_len(req, "Content-Type") == 9) { // length of "text/plain"
//         char content_type[10];
//         if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) == ESP_OK) {
//             if (strcmp(content_type, "text/plain") != 0) {
//                 ESP_LOGE(TAGS, "Invalid Content-Type: %s", content_type);
//                 httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Type");
//                 return ESP_FAIL;
//             }
//         }
//     }

//     // Buffer to hold received message
//     char content[100];
//     int content_len = req->content_len;

//     // Limit content length to buffer size
//     if (content_len >= sizeof(content)) {
//         content_len = sizeof(content) - 1;
//     }

//     // Read the message content from the HTTP request body
//     int ret = httpd_req_recv(req, content, content_len);
//     if (ret <= 0) {
//         // If receiving failed, respond with an error
//         httpd_resp_send_500(req);
//         return ESP_FAIL;
//     }

//     content[ret] = '\0';  // Null-terminate the received string
//     ESP_LOGI(TAGS, "Received message as string: %s", content);

//     // Convert received message to an integer
//     int received_integer = atoi(content);
//     ESP_LOGI(TAGS, "Received integer: %d", received_integer);
//     thres = received_integer;
//     // Respond with a success message
//     httpd_resp_send(req, "Integer received", HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }

// // Function to start the web server
// httpd_handle_t start_webserver(void) {
//     httpd_handle_t server = NULL;
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();

//     if (httpd_start(&server, &config) == ESP_OK) {
//         // Register the POST handler
//         httpd_uri_t message_uri = {
//             .uri       = "/message",
//             .method    = HTTP_POST,
//             .handler   = message_post_handler,
//             .user_ctx  = NULL
//         };
//         httpd_register_uri_handler(server, &message_uri);

//         // Register the GET handler for random numbers
//         httpd_uri_t random_number_uri = {
//             .uri       = "/random",
//             .method    = HTTP_GET,
//             .handler   = number_post_handler,
//             .user_ctx  = NULL
//         };
//         httpd_register_uri_handler(server, &random_number_uri);
//     }
//     return server;
// }

// static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
// {
//     if (event_id == WIFI_EVENT_AP_STACONNECTED) {
//         ESP_LOGI(TAGS, "Station connected");
//     } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
//         ESP_LOGI(TAGS, "Station disconnected");
//     }
// }

// void wifi_init_softap(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = EXAMPLE_ESP_WIFI_SSID,
//             .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
//             .channel = EXAMPLE_ESP_WIFI_CHANNEL,
//             .password = EXAMPLE_ESP_WIFI_PASS,
//             .max_connection = EXAMPLE_MAX_STA_CONN,
//             .authmode = WIFI_AUTH_WPA2_PSK,
//         },
//     };
//     if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
//         wifi_config.ap.authmode = WIFI_AUTH_OPEN;
//     }

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());

//     ESP_LOGI(TAGS, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
//              EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
// }


// void icm42670_test(void *pvParameters)
// {
//     // init device descriptor and device

//     icm42670_t dev = { 0 };
//     ESP_ERROR_CHECK(
//         icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
//     ESP_ERROR_CHECK(icm42670_init(&dev));

//     // enable accelerometer and gyro in low-noise (LN) mode
//     ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
//     ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

//     /* OPTIONAL */
//     // enable low-pass-filters on accelerometer and gyro
//     ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
//     ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
//     // set output data rate (ODR)
//     ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
//     ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
//     // set full scale range (FSR)
//     ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
//     ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

//     // read temperature sensor value once
//     float temperature;
//     ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
//     ESP_LOGI(TAG, "Temperature reading: %f", temperature);

    
//     uint8_t data_register;
//     int16_t raw_reading;
//     /* select which acceleration or gyro value should be read: */
//     // data_register = ICM42670_REG_ACCEL_DATA_X1;
//     data_register = ICM42670_REG_ACCEL_DATA_Y1;
//     // data_register = ICM42670_REG_ACCEL_DATA_Z1;
//     // data_register = ICM42670_REG_GYRO_DATA_X1;
//     // data_register = ICM42670_REG_GYRO_DATA_Y1;
//     // data_register = ICM42670_REG_GYRO_DATA_Z1;
//     int g = 0;
//     // now poll selected accelerometer or gyro raw value directly from registers
//     while (1)
//     {
//         ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register, &raw_reading));
//         g = 0.9 * g + 0.1*raw_reading;
//         // ESP_LOGI(TAG, "Received: %d", received_number);
//         lin_accel_y = raw_reading - g;
//         vTaskDelay(pdMS_TO_TICKS(100));
        
//     }
// }



// void app_main(void) {
//     // Initialize NVS and start Wi-Fi in AP mode
//     ESP_ERROR_CHECK(i2cdev_init());
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);


//     ESP_LOGI(TAGS, "ESP_WIFI_MODE_AP");
//     wifi_init_softap();


//     xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

//     // Start the web server
//     start_webserver();
// }




#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include <icm42670.h>
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include <time.h>

#define BTN1 GPIO_NUM_19
#define BTN2 GPIO_NUM_21
#define BTN3 GPIO_NUM_18
#define VBUS_INTR GPIO_NUM_42

#define IN1    38
#define IN2    40
#define EN1    39
#define EN2    41
#define nSLEEP 45
#define GPIO_OUTPUT_PIN_SEL  ( (1ULL<<IN1) | (1ULL<<IN2) | (1ULL<<EN1) | (1ULL<<EN2) |  (1ULL<<nSLEEP)  )


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (8) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // 8192 for 100%. To be decided.
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


#define SEG_A_GPIO 12
#define SEG_B_GPIO 13
#define SEG_C_GPIO 35
#define SEG_D_GPIO 33
#define SEG_E_GPIO 34
#define SEG_F_GPIO 10
#define SEG_G_GPIO 11

#define LED_BLUE1 3
#define LED_BLUE2 4
#define RGB_RED 15
#define RGB_GREEN 16
#define RGB_BLUE 17

#define DEFAULT_VREF    1100  // Reference voltage in mV (adjust based on your board calibration)
#define ADC_WIDTH       ADC_WIDTH_BIT_13  // 12-bit resolution
#define ADC_ATTEN       ADC_ATTEN_DB_11   // 11dB attenuation (for higher voltage range)
#define ADC_CHANNEL     ADC1_CHANNEL_8 

static const char *TAG = "icm42670";
static const char *TAGS = "wifi softAP";

#define INT_SOURCE1 0x2C

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif
#define ALPHA 0.8 


#define EXAMPLE_ESP_WIFI_SSID "bioSTEP+"
#define EXAMPLE_ESP_WIFI_PASS "biostim.in"
#define EXAMPLE_ESP_WIFI_CHANNEL 1
#define EXAMPLE_MAX_STA_CONN 4

static int16_t lin_accel_y = 0;
int thres = -325;

bool t_detect_flag = true;
static bool fire_on = false;
bool plot = false;
/* Find gpio definitions in sdkconfig */
// float lin_accel_y=0.0;
static esp_timer_handle_t fire_timer;
static SemaphoreHandle_t data_mutex;

xQueueHandle BTN1Queue;
xQueueHandle BTN2Queue;
xQueueHandle BTN3Queue;
xQueueHandle VBUS_INTRQueue;
xQueueHandle SETFireon;
xQueueHandle SETFireoff;

bool isLed_menu = true;
bool isLed_setup = false;
bool isLed_walk = false;   
bool long_press_detected = false;
bool error_flag = false;

uint64_t btn1pre_time = 0;
uint64_t btn1intr_time = 0;
uint64_t btn1curr_time = 0;

uint64_t btn2pre_time = 0;
uint64_t btn2intr_time = 0;
uint64_t btn2curr_time = 0;
uint64_t pre_time_vbus_intr = 0;

uint64_t btn3pre_time = 0;
uint64_t btn3intr_time = 0;
uint64_t btn3curr_time = 0;

int menu_cursor=0;
int setup_cursor=0;
int STIMStrength[] = {0, 819, 1638, 2457, 3276, 4095, 4914, 5733, 6552, 7371, 8190};


bool vbus_flag =true;


esp_err_t number_post_handler(httpd_req_t *req)
{
    char response[64];
    snprintf(response, sizeof(response), "{\"sensor\":%d, \"threshold\":%d}", lin_accel_y, thres);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "Sent JSON: %s", response);
    return ESP_OK;
}


// HTTP POST handler to receive messages
esp_err_t message_post_handler(httpd_req_t *req) {
    // Check the Content-Type header to ensure it’s text/plain
    if (httpd_req_get_hdr_value_len(req, "Content-Type") == 9) { // length of "text/plain"
        char content_type[10];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) == ESP_OK) {
            if (strcmp(content_type, "text/plain") != 0) {
                ESP_LOGE(TAGS, "Invalid Content-Type: %s", content_type);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Type");
                return ESP_FAIL;
            }
        }
    }

    // Buffer to hold received message
    char content[100];
    int content_len = req->content_len;

    // Limit content length to buffer size
    if (content_len >= sizeof(content)) {
        content_len = sizeof(content) - 1;
    }

    // Read the message content from the HTTP request body
    int ret = httpd_req_recv(req, content, content_len);
    if (ret <= 0) {
        // If receiving failed, respond with an error
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    content[ret] = '\0';  // Null-terminate the received string
    ESP_LOGI(TAGS, "Received message as string: %s", content);

    // Convert received message to an integer
    int received_integer = atoi(content);
    ESP_LOGI(TAGS, "Received integer: %d", received_integer);
    thres = received_integer;
    // Respond with a success message
    httpd_resp_send(req, "Integer received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// Function to start the web server
httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        // Register the POST handler
        httpd_uri_t message_uri = {
            .uri       = "/message",
            .method    = HTTP_POST,
            .handler   = message_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &message_uri);

        // Register the GET handler for random numbers
        httpd_uri_t random_number_uri = {
            .uri       = "/random",
            .method    = HTTP_GET,
            .handler   = number_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &random_number_uri);
    }
    return server;
}



static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAGS, "Station connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAGS, "Station disconnected");
    }
}


void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAGS, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}


void display_E()
{
    // For 'E': A, D, E, F, G are ON
    gpio_set_level(SEG_A_GPIO, 1);
    gpio_set_level(SEG_B_GPIO, 0);
    gpio_set_level(SEG_C_GPIO, 0);
    gpio_set_level(SEG_D_GPIO, 1);
    gpio_set_level(SEG_E_GPIO, 1);
    gpio_set_level(SEG_F_GPIO, 1);
    gpio_set_level(SEG_G_GPIO, 1);
}

void display_1()
{
    // For '1': B, C are ON
    gpio_set_level(SEG_A_GPIO, 0);
    gpio_set_level(SEG_B_GPIO, 1);
    gpio_set_level(SEG_C_GPIO, 1);
    gpio_set_level(SEG_D_GPIO, 0);
    gpio_set_level(SEG_E_GPIO, 0);
    gpio_set_level(SEG_F_GPIO, 0);
    gpio_set_level(SEG_G_GPIO, 0);
}


// bool trough_detected(float curr_val, float prev_val)
// {
// 	if(curr_val > prev_val)
// 		return true;
// 	return false;
// }

bool trough_detected(float curr_val, float prev_val) {
    return (prev_val > curr_val && curr_val <= thres);  // Detect only if below -310
}

void fire_off_callback(void* arg) {
    printf("-----Fire OFF (Timer)----\n");
    t_detect_flag = true;
    fire_on = false;
    int btn_num= 0;
    gpio_set_level(RGB_GREEN, 1);
    gpio_set_level(RGB_RED, 1);
    xQueueSend(SETFireoff,&btn_num,NULL);
    vTaskDelay(75 / portTICK_PERIOD_MS);
}


void error()
{
    error_flag = true;
    // gpio_set_level(LED_BLUE1,0);
    // gpio_set_level(LED_BLUE2,0);
}

void enter_deep_sleep() {
    
    ESP_LOGI("NO TAG", "Entering deep sleep in 500ms");
    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 500ms to ensure button is released

    // Ensure button is released before entering deep sleep
    while (gpio_get_level(BTN2) == 0) {
        ESP_LOGI("NO TAG", "Waiting for button release...");
        vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid busy waiting
    }

    esp_sleep_enable_ext0_wakeup(BTN2, 0); // Wake up when button is pressed (falling edge)
    esp_deep_sleep_start();
}

void display_error()
{
    display_E();  // Display 'E'
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for 1 second
    display_1();  // Display '1'
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void get_battery_voltage() {
    // Set up ADC configuration
    // adc2_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    adc1_config_width(ADC_WIDTH);
    // Read raw ADC value
    int raw_out=0;
    int adc_reading = adc1_get_raw(ADC_CHANNEL);
    printf("Raw Batt: %d\n",adc_reading);
    if(adc_reading<100)
    {
        setup_cursor = 0;
        error();
        display_error();
        display_error();
        display_error();
        enter_deep_sleep();
        
    }
}


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


static void led_menu()
{
    isLed_menu = true;
    isLed_setup = false;
    isLed_walk = false;
    gpio_set_direction(SEG_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_C_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_D_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_E_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_F_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE2, GPIO_MODE_OUTPUT);
    gpio_set_level(SEG_A_GPIO, 0);
    gpio_set_level(SEG_B_GPIO, 0);
    gpio_set_level(SEG_C_GPIO, 0);
    gpio_set_level(SEG_D_GPIO, 0);
    gpio_set_level(SEG_E_GPIO, 0);
    gpio_set_level(SEG_F_GPIO, 0);
    gpio_set_level(SEG_G_GPIO, 1);
    if(menu_cursor==0)
    {
        gpio_set_level(LED_BLUE2, 1);
        // gpio_set_level(LED_BLUE1, 0);
    }
    else if(menu_cursor == 1)
    {
        gpio_set_level(LED_BLUE1, 1);
        // gpio_set_level(LED_BLUE2, 0);
    }
    gpio_set_direction(RGB_GREEN,GPIO_MODE_OUTPUT);
    gpio_set_level(RGB_GREEN,0);
    gpio_set_level(RGB_RED,1);


    printf("menu cursor %d\n",menu_cursor);
}

static void display_number(int number)
{
    // Set segments based on the number to display
    switch (number)
    {
        case 0: // Display '0'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 1: // Display '1'
            gpio_set_level(SEG_A_GPIO, 0);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 2: // Display '2'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 0);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 3: // Display '3'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 4: // Display '4'
            gpio_set_level(SEG_A_GPIO, 0);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 5: // Display '5'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 0);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 6: // Display '6'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 0);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 7: // Display '7'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 8: // Display '8'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 9: // Display '9'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;

        default:
            printf("Invalid setup_cursor value\n");
            break;
    }
}

static void turn_off_segments()
{
    gpio_set_level(SEG_A_GPIO, 0);
    gpio_set_level(SEG_B_GPIO, 0);
    gpio_set_level(SEG_C_GPIO, 0);
    gpio_set_level(SEG_D_GPIO, 0);
    gpio_set_level(SEG_E_GPIO, 0);
    gpio_set_level(SEG_F_GPIO, 0);
    gpio_set_level(SEG_G_GPIO, 0);
}


static void led_setup()
{
    isLed_menu = false;
    isLed_setup = true;
    isLed_walk = false;
    // Set GPIO direction for seven-segment display
    gpio_set_direction(SEG_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_C_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_D_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_E_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_F_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_G_GPIO, GPIO_MODE_OUTPUT);
                gpio_set_direction(RGB_GREEN,GPIO_MODE_OUTPUT);
                gpio_set_level(RGB_GREEN,1);


    printf("Setup cursor %d\n", setup_cursor);

}



static void led_walk()
{
    isLed_menu = false;
    isLed_setup = false;
    isLed_walk = true;
    gpio_set_direction(SEG_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_C_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_D_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_E_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_F_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SEG_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE2, GPIO_MODE_OUTPUT);
    printf("setup cursor %d\n",setup_cursor);
                gpio_set_direction(RGB_GREEN,GPIO_MODE_OUTPUT);
                gpio_set_level(RGB_GREEN,1);
    gpio_set_level(LED_BLUE2, 0);
    switch (setup_cursor)
    {
        case 0: // Display '0'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 1: // Display '1'
            gpio_set_level(SEG_A_GPIO, 0);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 2: // Display '2'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 0);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 3: // Display '3'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 4: // Display '4'
            gpio_set_level(SEG_A_GPIO, 0);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 5: // Display '5'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 0);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 6: // Display '6'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 0);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 7: // Display '7'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 0);
            gpio_set_level(SEG_G_GPIO, 0);
            break;
        case 8: // Display '8'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        case 9: // Display '9'
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 1);
            gpio_set_level(SEG_D_GPIO, 1);
            gpio_set_level(SEG_E_GPIO, 0);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);
            break;
        default:
            printf("Invalid setup_cursor value\n");
            break;

        
    }

}


void icm42670_test(void *pvParameters)
{
    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(
        icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG, "Temperature reading: %f", temperature);

    int16_t raw_reading;
    uint8_t data_register1;
    uint8_t data_register2;
    uint8_t data_register3;
    uint8_t data_register4;
    uint8_t data_register5;
    uint8_t data_register6;

    /* select which acceleration or gyro value should be read: */
    data_register1 = ICM42670_REG_ACCEL_DATA_X1;
    data_register2= ICM42670_REG_ACCEL_DATA_Y1;
    data_register3= ICM42670_REG_ACCEL_DATA_Z1;
    data_register4= ICM42670_REG_GYRO_DATA_X1;
    data_register5 = ICM42670_REG_GYRO_DATA_Y1;
    data_register6= ICM42670_REG_GYRO_DATA_Z1;
    // ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, INT_SOURCE1, &raw_reading));
    // printf("%d\n", raw_reading);
    int g=0;
    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register2, &raw_reading));

        //ESP_LOGI(TAG, "Raw accelerometer / gyro reading: %d", raw_reading);
        // printf("%d %d %d\n",5000,raw_reading,-5000);
        //float accel_y = raw_reading/2048.0;
        // g=0.9*g+0.1*raw_reading;
        // lin_accel_y = raw_reading-g;
        
        // vTaskDelay(pdMS_TO_TICKS(75));
        float filtered_value = 0.9 * g + 0.1 * raw_reading;
        g = filtered_value;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        lin_accel_y = raw_reading - g;
        xSemaphoreGive(data_mutex);
        // printf("2000 %.2f -2000\n",lin_accel_y);
        vTaskDelay(pdMS_TO_TICKS(75));
    }
}

void gait_detect(void *params)
{
    vTaskDelay(pdMS_TO_TICKS(10000));//initialization delay :)
    float prev_value = 0.0;
	float curr_value = 0.0;
    float fire_value = 0;

    while(1)
    {
        if(isLed_walk)
        {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
        prev_value = curr_value;
		curr_value = lin_accel_y;
        xSemaphoreGive(data_mutex);
        if (trough_detected(curr_value, prev_value) && t_detect_flag)
        {
           printf("--->->->->->Fire on<-<-<-<-<----\n");
           xQueueSend(SETFireon,&fire_value,NULL);
           t_detect_flag = false;
           fire_on = true;
           fire_value = curr_value;
           plot = true;
           esp_timer_start_once(fire_timer, 700000);
        }

        // if(lin_accel_y > (fire_value+1000) && fire_on)
        // {
        //     printf("-----Fire OFF----\n");
        //     xQueueSend(SETFireoff,&fire_value,NULL);
        //     fire_on = false;
        //     t_detect_flag = true;
        //     esp_timer_stop(fire_timer);
        // }
        // if(plot)
        // {
        // printf("%d %.2f %.2f %d\n",2000,lin_accel_y,fire_value,-2000);
        // plot = false;
        // }
        // if(!plot)
        // {
        //     printf("%d %.2f 0 %d\n",2000,lin_accel_y,-2000);
        // }


    
    
        }
        vTaskDelay(pdMS_TO_TICKS(75));
    }
}

void Stimul(void *params)
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    example_ledc_init();
    gpio_set_level(nSLEEP, 1);
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_direction(RGB_RED, GPIO_MODE_OUTPUT);
    int btn_num =0 ;

    while(1)
    {
         if(isLed_setup)
         {
            printf("Driver on, strength set : %d \n", STIMStrength[setup_cursor]);
            for(int i= 0;i<20;i++)
            {
                
                gpio_set_level(IN1, 1);
                gpio_set_level(IN2, 0);
                if((i==2 && setup_cursor!=0) || (i==19 && setup_cursor!=0)  || (i==11 && setup_cursor!=0) || (i==6 && setup_cursor!=0)  || (i==16 && setup_cursor!=0))
                {
                    get_battery_voltage();
                }
                // get_battery_voltage();
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,STIMStrength[setup_cursor] ));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                vTaskDelay(10 / portTICK_PERIOD_MS);
                gpio_set_level(IN1, 0);
                gpio_set_level(IN2, 1);
                vTaskDelay(10 / portTICK_PERIOD_MS);

                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

                gpio_set_level(IN1, 0);
                gpio_set_level(IN2, 0);
                vTaskDelay(12.5 / portTICK_PERIOD_MS);
                
                // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, strength_current));
                // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            }

            printf("Stimulation fired : driver off for 2 sec\n");
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            vTaskDelay(2000 / portTICK_PERIOD_MS);
         }   
         if(isLed_walk)
         {
        if(xQueueReceive(SETFireon,&btn_num,portMAX_DELAY))
         {
                printf("Stimualtion on\n");
                ESP_LOGI("NO TAG ","Driver on, strength set : %d \n", STIMStrength[setup_cursor]);
                gpio_set_level(RGB_GREEN, 0);
                gpio_set_level(RGB_RED, 0);
                for(int i= 0;i<20;i++)
                {

                    gpio_set_level(IN1, 1);
                    gpio_set_level(IN2, 0);
                    if((i==1 && setup_cursor!=0) || (i==19 && setup_cursor!=0)  || (i==11 && setup_cursor!=0) || (i==6 && setup_cursor!=0)  || (i==16 && setup_cursor!=0))
                      get_battery_voltage();
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,STIMStrength[setup_cursor]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    vTaskDelay(10/ portTICK_PERIOD_MS);
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 1);
                    vTaskDelay(10/ portTICK_PERIOD_MS);
        
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
     
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    vTaskDelay(12.5/ portTICK_PERIOD_MS);
                    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, STIMStrength[1]));
                    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
         }
                

         }
         
            if(xQueueReceive(SETFireoff,&btn_num,portMAX_DELAY))
                {
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    printf("Stimulation off\n");
                    xQueueReset(SETFireoff);
                    gpio_set_level(RGB_GREEN, 1);
                    gpio_set_level(RGB_RED, 1);
                }
                xQueueReset(SETFireon);
                    
         }

 
 
           vTaskDelay(10 / portTICK_PERIOD_MS);
         }

          
    }


void Fading_Task(void *params)
{
    while(1)
    {
        if(isLed_menu && menu_cursor ==0)
        {
            gpio_set_level(LED_BLUE2,1);
            gpio_set_level(LED_BLUE1,0);
            vTaskDelay(500/portTICK_PERIOD_MS);
            gpio_set_level(LED_BLUE1,1);

        }
        else if (isLed_menu && menu_cursor ==1)
        {
            gpio_set_level(LED_BLUE1,1);
            gpio_set_level(LED_BLUE2,0);
            vTaskDelay(500/portTICK_PERIOD_MS);
            gpio_set_level(LED_BLUE2,1);
        }

        else if(isLed_setup)
        {
            gpio_set_level(LED_BLUE1, 0);
        }
        else if(isLed_walk)
        {
            gpio_set_level(LED_BLUE2, 0);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void Blink_Task(void *params)
{
    while(1)
    {
        if(isLed_setup && !error_flag)
        {
            // Display the current number on the seven-segment display
            display_number(setup_cursor);
            
            // Wait for a short period to create the blink effect
            vTaskDelay(400/ portTICK_PERIOD_MS);
    
            // Turn off all segments to create the blink effect
            turn_off_segments();
    
            // Wait for a short period to create the blink effect
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else if(isLed_menu)
        {
                gpio_set_direction(LED_BLUE1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE2, GPIO_MODE_OUTPUT);
    gpio_set_level(SEG_A_GPIO, 0);
    gpio_set_level(SEG_B_GPIO, 0);
    gpio_set_level(SEG_C_GPIO, 0);
    gpio_set_level(SEG_D_GPIO, 0);
    gpio_set_level(SEG_E_GPIO, 0);
    gpio_set_level(SEG_F_GPIO, 0);
    gpio_set_level(SEG_G_GPIO, 1);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}


void VBUS_INTRTask(void *params)
{
    gpio_set_direction(VBUS_INTR, GPIO_MODE_INPUT);
    gpio_set_intr_type(VBUS_INTR, GPIO_INTR_POSEDGE);
    int BTN_NUMBER = 0;
    while (1)
    {
        if (xQueueReceive(VBUS_INTRQueue, &BTN_NUMBER, portMAX_DELAY))
        {
            
            if(vbus_flag)
            {
                printf("Charging....!\n");
                vbus_flag=false;
                isLed_menu = false;
                isLed_setup = false;
                isLed_walk = false;
                gpio_set_level(LED_BLUE1,1);
                gpio_set_level(LED_BLUE2,1);
                gpio_set_direction(RGB_GREEN,GPIO_MODE_OUTPUT);
                gpio_set_level(RGB_GREEN,0);
            gpio_set_level(SEG_A_GPIO, 1);
            gpio_set_level(SEG_B_GPIO, 1);
            gpio_set_level(SEG_C_GPIO, 0);
            gpio_set_level(SEG_D_GPIO, 0);
            gpio_set_level(SEG_E_GPIO, 1);
            gpio_set_level(SEG_F_GPIO, 1);
            gpio_set_level(SEG_G_GPIO, 1);

            }
            else if(!vbus_flag)
            {
                printf("pluged out....!\n");
                vbus_flag=true;
                led_menu();
            }

  
            xQueueReset(VBUS_INTRQueue);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}


void BTN1Task(void *params)
{
    gpio_set_direction(BTN1, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN1, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;

    while (1)
    {
        if (xQueueReceive(BTN1Queue, &BTN_NUMBER, portMAX_DELAY))
        {
            // Wait for button release
            while (gpio_get_level(BTN1) == 0)
            {
                btn1curr_time = esp_timer_get_time();
            }

            // Check if the button was pressed for a short duration
            if (btn1curr_time - btn1intr_time < 1000000) // Adjust the time threshold for short press detection as needed
            {
                ESP_LOGI("NO TAG", "Short Press Detected on BTN1");
                if(isLed_setup && setup_cursor<9)
                {
                    setup_cursor++;
                    led_setup();
                }
                else if(isLed_menu && menu_cursor==1)
                {
                    menu_cursor=0;
                    led_menu();
                }

            }

            xQueueReset(BTN1Queue);
        }
    }
}





void BTN2Task(void *params)
{
    gpio_set_direction(BTN2, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN2, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;
    gpio_set_direction(LED_BLUE1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE2, GPIO_MODE_OUTPUT);

    while (1)
    {
        if (xQueueReceive(BTN2Queue, &BTN_NUMBER, portMAX_DELAY))
        {
            long_press_detected = false; 
            btn2intr_time = esp_timer_get_time();  // Record the interrupt time
            
            // Wait for button release or long press detection
            while (gpio_get_level(BTN2) == 0)
            {
                btn2curr_time = esp_timer_get_time();

                // Check for long press duration
                if (btn2curr_time - btn2intr_time >= 1500000) // 1 second threshold for long press
                {
                    ESP_LOGI("NO TAG", "Long Press Detected on BTN2");
                    long_press_detected = true; // Set long press flag
                    isLed_menu = false;
                    isLed_setup = false;
                    isLed_walk = false;
                        gpio_set_level(LED_BLUE2,1);
    gpio_set_level(SEG_A_GPIO, 0);
    gpio_set_level(SEG_B_GPIO, 0);
    gpio_set_level(SEG_C_GPIO, 0);
    gpio_set_level(SEG_D_GPIO, 0);
    gpio_set_level(SEG_E_GPIO, 0);
    gpio_set_level(SEG_F_GPIO, 0);
    gpio_set_level(SEG_G_GPIO, 0);
     gpio_set_level(RGB_GREEN, 1);
     gpio_set_level(LED_BLUE1,1);
                    enter_deep_sleep(); 
                    break; // Exit loop after long press is detected
                }
            }

            // Check if the button was released for a short press duration
            if (!long_press_detected && (btn2curr_time - btn2intr_time < 1500000))
            {
                ESP_LOGI("NO TAG", "Short Press Detected on BTN2");

                if (isLed_menu && menu_cursor == 0)
                {
                    isLed_menu = false;
                    isLed_setup = true;
                    setup_cursor=0;
                    led_setup();
                }
                else if (isLed_menu && menu_cursor == 1)
                {
                    isLed_menu = false;
                    isLed_walk = true;
                    led_walk();
                }
                else if (isLed_setup)
                {
                    isLed_menu = true;
                    isLed_setup = false;
                    led_menu();
                }
                else if (isLed_walk)
                {
                    led_menu();
                }
            }

            xQueueReset(BTN2Queue); // Reset the queue after processing
        }
    }
}


void BTN3Task(void *params)
{
    gpio_set_direction(BTN3, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN3, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;

    while (1)
    {
        if (xQueueReceive(BTN3Queue, &BTN_NUMBER, portMAX_DELAY))
        {
            // Wait for button release
            while (gpio_get_level(BTN3) == 0)
            {
                btn3curr_time = esp_timer_get_time();
            }

            // Check if the button was pressed for a short duration
            if (btn3curr_time - btn3intr_time < 1000000) // Adjust the time threshold for short press detection as needed
            {
                ESP_LOGI("NO TAG", "Short Press Detected on BTN3");

                if(isLed_setup && setup_cursor>0)
                {
                    setup_cursor--;
                    led_setup();
                }

                else if(isLed_menu && menu_cursor==0)
                {
                    menu_cursor=1;
                    led_menu();
                }

            }

            xQueueReset(BTN3Queue);
        }
    }
}

static void IRAM_ATTR BTN1_interrupt_handler(void *args)
{
    int pinNumber = (int)args;

    if (esp_timer_get_time() - btn1pre_time > 400000)
    {
        xQueueSendFromISR(BTN1Queue, &pinNumber, NULL);
        btn1intr_time = esp_timer_get_time();
    }

    btn1pre_time = esp_timer_get_time();
}


static void IRAM_ATTR BTN2_interrupt_handler(void *args)
{
    int pinNumber = (int)args;

    if (esp_timer_get_time() - btn2curr_time > 400000)
    {
        xQueueSendFromISR(BTN2Queue, &pinNumber, NULL);
    }

    btn2curr_time = esp_timer_get_time();
}


static void IRAM_ATTR BTN3_interrupt_handler(void *args)
{
    int pinNumber = (int)args;

    if (esp_timer_get_time() - btn3pre_time > 400000)
    {
        xQueueSendFromISR(BTN3Queue, &pinNumber, NULL);
        btn3intr_time = esp_timer_get_time();
    }

    btn3pre_time = esp_timer_get_time();
}

static void IRAM_ATTR VBUS_INTR_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    if(esp_timer_get_time() - pre_time_vbus_intr > 400000){
    xQueueSendFromISR(VBUS_INTRQueue, &pinNumber, NULL);
    }
    pre_time_vbus_intr = esp_timer_get_time();
}



void app_main(void)
{
        BTN1Queue = xQueueCreate(10, sizeof(int));
    BTN2Queue = xQueueCreate(10, sizeof(int));
    BTN3Queue = xQueueCreate(10, sizeof(int));
     VBUS_INTRQueue = xQueueCreate(10, sizeof(int));
     SETFireon = xQueueCreate(10, sizeof(int));
     SETFireoff = xQueueCreate(10, sizeof(int));
     data_mutex = xSemaphoreCreateMutex();
     ESP_ERROR_CHECK(i2cdev_init());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
    ESP_ERROR_CHECK(ret);


    ESP_LOGI(TAGS, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
        gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN1, BTN1_interrupt_handler, (void *)BTN1);
    gpio_isr_handler_add(BTN2, BTN2_interrupt_handler, (void *)BTN2);
    gpio_isr_handler_add(BTN3, BTN3_interrupt_handler, (void *)BTN3);
    gpio_isr_handler_add(VBUS_INTR, VBUS_INTR_interrupt_handler, (void *)VBUS_INTR);
    const esp_timer_create_args_t fire_timer_args = {
        .callback = &fire_off_callback,
        .name = "fire_timer"
    };

    if (esp_timer_create(&fire_timer_args, &fire_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fire timer");
        return;
    }

        xTaskCreate(BTN1Task, "BTN1_Task", 2048, NULL, 1, NULL);
    xTaskCreate(BTN2Task, "BTN2_Task", 2048, NULL, 1, NULL);
    xTaskCreate(BTN3Task, "BTN3_Task", 2048, NULL, 1, NULL);
    xTaskCreate(Blink_Task, "Blink_Task", 2048, NULL, 1, NULL);
    xTaskCreate(Fading_Task, "Fading_Task", 2048, NULL, 1, NULL);
    xTaskCreate(VBUS_INTRTask, "VBUS_INTRTask", 8000, NULL, 1, NULL);
    xTaskCreate(Stimul, "Stimulation", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(gait_detect, "set fire", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    led_menu();
    start_webserver();
}