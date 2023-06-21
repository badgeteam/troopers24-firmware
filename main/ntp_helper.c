#include "ntp_helper.h"
#include "wifi_connect.h"

static const char* TAG = "ntp";

bool ntp_synced = false;

bool sync_ntp() {
    // Set timezone to Europe/Berlin
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    // Check if time is already synced
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2020 - 1900)) {
        ESP_LOGI(TAG, "Time already synced.");
        return true;
    }

    if (!wifi_connect_to_stored()) {
        ESP_LOGE(TAG, "Couldn't connect to wifi, not syncing time.");
        wifi_disconnect_and_disable();
        return false;
    }

    ESP_LOGI(TAG, "Starting NTP sync");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();

    int retry = 0;
    const int retry_count = 5;
    sntp_sync_status_t sync_status;
    while ((sync_status = sntp_get_sync_status()) == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGD(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    bool res = false;
    if (sync_status == SNTP_SYNC_STATUS_COMPLETED) {
        res = true;
        time(&now);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "Successfully updated time: %s", asctime(&timeinfo));
    } else {
        ESP_LOGE(TAG, "Couldn't update time");
    }

    wifi_disconnect_and_disable();
    return res;
}