#include "wifi_ota.h"

#include <sys/socket.h>

#include "bootscreen.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gui_element_header.h"
#include "hardware.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "wifi.h"
#include "wifi_cert.h"
#include "wifi_connect.h"

#define HASH_LEN 32

static const char *TAG = "OTA update";

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADERS_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADERS_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t         running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s, available firmware version: %s", running_app_info.version, new_app_info->version);
        if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
            ESP_LOGW(TAG, "Already up-to-date!");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "Unable to check current firmware version");
    }

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client) {
    if (esp_http_client_set_header(http_client, "Badge-Type", "TROOPERS24") != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add type header");
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t         running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        if (esp_http_client_set_header(http_client, "Badge-Firmware", running_app_info.version) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to add version header");
        }
    }
    return ESP_OK;
}

/*static void print_sha256(const uint8_t *image_hash, const char *label) {
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void) {
    uint8_t         sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size    = ESP_PARTITION_TABLE_OFFSET;
    partition.type    = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}*/

void display_ota_state(const char *text, bool nightly) {
    pax_buf_t *pax_buffer = get_pax_buffer();
    pax_noclip(pax_buffer);
    const pax_font_t *font = pax_font_saira_regular;
    pax_background(pax_buffer, nightly ? 0x000000 : 0xFFFFFF);
    render_header(pax_buffer, 0, 0, pax_buffer->width, 34, 18, 0xFF491d88, 0xFF43b5a0, NULL, "Firmware update");
    pax_vec1_t title_size = pax_text_size(font, 18, nightly ? "Experimental firmware" : "Firmware update");
    pax_draw_text(pax_buffer, nightly ? 0xFFFF0000 : 0xFF000000, font, 18, (320 / 2) - (title_size.x / 2), 120 - 30,
                  nightly ? "Experimental firmware" : "Firmware update");
    pax_vec1_t size = pax_text_size(font, 18, text);
    pax_draw_text(pax_buffer, nightly ? 0xFFFFFFFF : 0xFF000000, font, 18, (320 / 2) - (size.x / 2), 120 + 10, text);
    display_flush();
}

void ota_update(bool nightly) {
    display_ota_state("Connecting to WiFi...", nightly);

    char *ota_url = nightly ? "https://mch2022.ota.bodge.team/troopers24_dev.bin" : "https://mch2022.ota.bodge.team/troopers24.bin";

    if (!wifi_connect_to_stored()) {
        display_ota_state("Failed to connect to WiFi", nightly);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        return;
    }

    display_ota_state("Starting update...", nightly);
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable any WiFi power save mode

    ESP_LOGI(TAG, "Starting OTA update");

    esp_http_client_config_t config = {.url = ota_url, .use_global_ca_store = true, .event_handler = _http_event_handler, .keep_alive_enable = true};

    esp_https_ota_config_t ota_config = {
        .http_config         = &config,
        .http_client_init_cb = _http_client_init_cb,  // Register a callback to be invoked after esp_http_client is initialized
#ifdef CONFIG_EXAMPLE_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .partial_http_download = true,
        .max_http_request_size = CONFIG_EXAMPLE_HTTP_REQUEST_SIZE,
#endif
    };

    // config.skip_cert_common_name_check = true;

    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);

    display_ota_state("Starting download...", nightly);

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t              err              = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        wifi_disconnect_and_disable();
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        display_ota_state("Failed to start download", nightly);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        esp_https_ota_abort(https_ota_handle);
        wifi_disconnect_and_disable();
        display_ota_state("Failed to read image desc", nightly);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) {
        esp_https_ota_abort(https_ota_handle);
        wifi_disconnect_and_disable();
        display_ota_state("Already up-to-date!", nightly);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return;
    }

    esp_err_t ota_finish_err = ESP_OK;
    int       percent_shown  = -1;
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        int len_total = esp_https_ota_get_image_size(https_ota_handle);
        int len_read  = esp_https_ota_get_image_len_read(https_ota_handle);
        int percent   = (len_read * 100) / len_total;

        if (percent != percent_shown) {
            ESP_LOGI(TAG, "Downloading %d / %d (%d%%)", len_read, len_total, percent);
            percent_shown = percent;
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Updating... %d%%", percent);
            display_ota_state(buffer, nightly);
        }
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received and user can customise the response to this situation.
        ESP_LOGE(TAG, "Complete data was not received.");
        display_ota_state("Download failed", nightly);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
            display_ota_state("Update installed", nightly);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                display_ota_state("Image validation failed", nightly);
            } else {
                display_ota_state("Update failed", nightly);
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        }
    }

    esp_https_ota_abort(https_ota_handle);
    esp_restart();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
