
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#define EXAMPLE_ESP_WIFI_SSID "bioSTEP+"
#define EXAMPLE_ESP_WIFI_PASS "biostim.in"
#define EXAMPLE_ESP_WIFI_CHANNEL 1
#define EXAMPLE_MAX_STA_CONN 4

static const char *TAG = "wifi softAP";

// HTTP GET handler to send a random number
esp_err_t random_number_get_handler(httpd_req_t *req) {
    int random_number = rand() % 100;  // Generate random number between 0-99
    char response[20];
    snprintf(response, sizeof(response), "%d", random_number);

    ESP_LOGI(TAG, "Sending random number: %s", response);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP POST handler to receive messages
esp_err_t message_post_handler(httpd_req_t *req) {
    // Check the Content-Type header to ensure it’s text/plain
    if (httpd_req_get_hdr_value_len(req, "Content-Type") == 9) { // length of "text/plain"
        char content_type[10];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) == ESP_OK) {
            if (strcmp(content_type, "text/plain") != 0) {
                ESP_LOGE(TAG, "Invalid Content-Type: %s", content_type);
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
    ESP_LOGI(TAG, "Received message as string: %s", content);

    // Convert received message to an integer
    int received_integer = atoi(content);
    ESP_LOGI(TAG, "Received integer: %d", received_integer);

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
            .handler   = random_number_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &random_number_uri);
    }
    return server;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Station connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Station disconnected");
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

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}


void app_main(void) {
    // Initialize NVS and start Wi-Fi in AP mode
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    // Start the web server
    start_webserver();
}