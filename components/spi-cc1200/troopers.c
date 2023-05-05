#include <mbedtls/sha256.h>
#include <string.h>


#include "include/cc1200_troopers.h"
#include "include/driver_cc1200.h"

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "driver_cc1200";

static const char *handle = "cc1200_troopers";
static const char *key    = "state";

xSemaphoreHandle driver_cc1200_troopers_task_mux    = NULL;
TaskHandle_t     driver_cc1200_troopers_task_handle = NULL;

uint8_t     last_effect                              = 0;
uint32_t    remaining_ticks                          = 0;
extern bool driver_cc1200_troopers_animation_running = false;

#define TICKS_PER_MS (1. / portTICK_PERIOD_MS)
#define TICKS_PER_S  (1000 * TICKS_PER_MS)
#define LED_OFF      "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
#define LED_ORANGE   "\xea\xa3\x07\xea\xa3\x07\xea\xa3\x07\xea\xa3\x07\xea\xa3\x07\xea\xa3\x07"

void driver_cc1200_troopers_task(void *arg) {
    while (1) {
        if (xSemaphoreTake(driver_cc1200_troopers_task_mux, portMAX_DELAY) != pdTRUE) continue;  // TIMEOUT
        ESP_LOGI(TAG, "Countdown: start");
        uint32_t start;
        uint32_t diff;
        int32_t  delay;
        while (1) {
            start = xTaskGetTickCount();
            if (remaining_ticks < 100 * TICKS_PER_MS) {
                if (last_effect > 0) {
                    ESP_LOGI(TAG, "Countdown: 100ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 0;
                }
            } else if (remaining_ticks < 200 * TICKS_PER_MS) {
                if (last_effect > 1) {
                    ESP_LOGI(TAG, "Countdown: 200ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
                    last_effect = 1;
                }
            } else if (remaining_ticks < 300 * TICKS_PER_MS) {
                if (last_effect > 2) {
                    ESP_LOGI(TAG, "Countdown: 300ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 2;
                }
            } else if (remaining_ticks < 400 * TICKS_PER_MS) {
                if (last_effect > 3) {
                    ESP_LOGI(TAG, "Countdown: 400ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
                    last_effect = 3;
                }
            } else if (remaining_ticks < 500 * TICKS_PER_MS) {
                if (last_effect > 4) {
                    ESP_LOGI(TAG, "Countdown: 500ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 4;
                }
            } else if (remaining_ticks < 600 * TICKS_PER_MS) {
                if (last_effect > 5) {
                    ESP_LOGI(TAG, "Countdown: 600ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
                    last_effect = 5;
                }
            } else if (remaining_ticks < 700 * TICKS_PER_MS) {
                if (last_effect > 6) {
                    ESP_LOGI(TAG, "Countdown: 700ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 6;
                }
            } else if (remaining_ticks < 800 * TICKS_PER_MS) {
                if (last_effect > 7) {
                    ESP_LOGI(TAG, "Countdown: 800ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
                    last_effect = 7;
                }
            } else if (remaining_ticks < 900 * TICKS_PER_MS) {
                if (last_effect > 8) {
                    ESP_LOGI(TAG, "Countdown: 900ms");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 8;
                }
            } else if (remaining_ticks < 1 * TICKS_PER_S) {
                if (last_effect > 9) {
                    ESP_LOGI(TAG, "Countdown: 1s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play_async(16);
#endif
                    last_effect = 9;
                }
            } else if (remaining_ticks < 2 * TICKS_PER_S) {
                if (last_effect > 10) {
                    ESP_LOGI(TAG, "Countdown: 2s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(14);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 10;
                }
            } else if (remaining_ticks < 3 * TICKS_PER_S) {
                if (last_effect > 11) {
                    ESP_LOGI(TAG, "Countdown: 3s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(14);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 11;
                }
            } else if (remaining_ticks < 4 * TICKS_PER_S) {
                if (last_effect > 12) {
                    ESP_LOGI(TAG, "Countdown: 4s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(14);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 12;
                }
            } else if (remaining_ticks < 5 * TICKS_PER_S) {
                if (last_effect > 13) {
                    ESP_LOGI(TAG, "Countdown: 5s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(14);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 13;
                }
            } else if (remaining_ticks < 6 * TICKS_PER_S) {
                if (last_effect > 14) {
                    ESP_LOGI(TAG, "Countdown: 6s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(72);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 14;
                }
            } else if (remaining_ticks < 7 * TICKS_PER_S) {
                if (last_effect > 15) {
                    ESP_LOGI(TAG, "Countdown: 7s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(72);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 15;
                }
            } else if (remaining_ticks < 8 * TICKS_PER_S) {
                if (last_effect > 16) {
                    ESP_LOGI(TAG, "Countdown: 8s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(72);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 16;
                }
            } else if (remaining_ticks < 9 * TICKS_PER_S) {
                if (last_effect > 17) {
                    ESP_LOGI(TAG, "Countdown: 9s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(72);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 17;
                }
            } else if (remaining_ticks < 10 * TICKS_PER_S) {
                if (last_effect > 18) {
                    ESP_LOGI(TAG, "Countdown: 10s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(72);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 18;
                }
            } else if (remaining_ticks < 15 * TICKS_PER_S) {
                if (last_effect > 19) {
                    ESP_LOGI(TAG, "Countdown: 15s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(16);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 19;
                }
            } else if (remaining_ticks < 30 * TICKS_PER_S) {
                if (last_effect > 20) {
                    ESP_LOGI(TAG, "Countdown: 30s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(71);
                    driver_drv2605l_play(71);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 20;
                }
            } else if (remaining_ticks < 60 * TICKS_PER_S) {
                if (last_effect > 21) {
                    ESP_LOGI(TAG, "Countdown: 60s");
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_ORANGE, 18);
#endif
#ifdef CONFIG_DRIVER_DRV2605L_ENABLE
                    driver_drv2605l_play(71);
#endif
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
                    driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
                    last_effect = 21;
                }
            }
            diff = xTaskGetTickCount() - start;
            if (remaining_ticks < diff) break;
            remaining_ticks -= diff;
            delay = (100 * TICKS_PER_MS) - diff;
            if (delay > 0) {
                vTaskDelay(delay);
                if (remaining_ticks < delay) break;
                remaining_ticks -= delay;
            }
        }
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
        driver_neopixel_send_data((uint8_t *) LED_OFF, 18);
#endif
        ESP_LOGI(TAG, "Countdown finished");
        driver_cc1200_troopers_animation_running = false;
    }
}

esp_err_t cc1200_troopers_init() {
    driver_cc1200_troopers_task_mux = xSemaphoreCreateMutex();
    if (driver_cc1200_troopers_task_mux == NULL) return ESP_ERR_NO_MEM;
    xTaskCreate(&driver_cc1200_troopers_task, "CC1200 Troopers interrupt task", 4096, NULL, 10, &driver_cc1200_troopers_task_handle);
    return ESP_OK;
}

esp_err_t get_last_state(cc1200_troopers_state_t *value) {
    nvs_handle my_handle;
    size_t     bufflen = sizeof(cc1200_troopers_state_t);

    esp_err_t res = nvs_open(handle, NVS_READWRITE, &my_handle);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "NVS handle not found!");
        return ESP_FAIL;
    }

    if (ESP_ERR_NVS_NOT_FOUND == nvs_get_blob(my_handle, key, value, &bufflen)) {
        value->msgid = 0;
        memcpy(value->hash, CC1200_TROOPERS_CHAIN_TAIL, CC1200_TROOPERS_HASH_SIZE);
    } else if (bufflen != sizeof(cc1200_troopers_state_t)) {
        ESP_LOGE(TAG, "NVS expected to read %d bytes, got %d bytes", sizeof(cc1200_troopers_state_t), bufflen);
        return ESP_FAIL;
    }
    nvs_close(my_handle);

    return ESP_OK;
}

esp_err_t set_last_state(const cc1200_troopers_state_t *value) {
    nvs_handle my_handle;
    esp_err_t  res = nvs_open(handle, NVS_READWRITE, &my_handle);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "NVS handle not found!");
        return 1;
    }

    esp_err_t esp_err = nvs_set_blob(my_handle, key, value, sizeof(cc1200_troopers_state_t));
    if (ESP_OK == esp_err) {
        nvs_commit(my_handle);
        nvs_close(my_handle);
        return ESP_OK;
    } else if (ESP_ERR_NVS_NOT_ENOUGH_SPACE == esp_err || ESP_ERR_NVS_PAGE_FULL == esp_err || ESP_ERR_NVS_NO_FREE_PAGES == esp_err) {
        ESP_LOGE(TAG, "No space available.");
    } else if (ESP_ERR_NVS_INVALID_NAME == esp_err || ESP_ERR_NVS_KEY_TOO_LONG == esp_err) {
        ESP_LOGE(TAG, "Key invalid or too long");
    }
    nvs_close(my_handle);
    return 2;
}

bool cc1200_troopers_replay_protection(cc1200_troopers_message_t *msg) {
    cc1200_troopers_state_t state;
    esp_err_t               res;
    res = get_last_state(&state);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "get state error");
        return false;
    }

    if (msg->msgid <= state.msgid) {
        ESP_LOGW(TAG, "Replay %d %d", msg->msgid, state.msgid);
        return false;
    }

    if (msg->msgid - state.msgid > CC1200_TROOPERS_MAX_BEHIND) {
        ESP_LOGE(TAG, "Too faar behind %d %d", msg->msgid, state.msgid);
        return false;
    }

    uint8_t node[CC1200_TROOPERS_HASH_SIZE];
    memcpy(node, msg->hash, CC1200_TROOPERS_HASH_SIZE);
    ESP_LOGI(TAG, "msg=%d state=%d", msg->msgid, state.msgid);
    ESP_LOGI(TAG, "--GOT--------------------------------");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, node, 32, ESP_LOG_INFO);
    while (state.msgid != msg->msgid) {
        mbedtls_sha256(node, CC1200_TROOPERS_HASH_SIZE, node, 0);
        state.msgid++;
        ESP_LOGI(TAG, "-------------------------------------");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, node, 32, ESP_LOG_INFO);
    }

    ESP_LOGI(TAG, "--EXPECTED---------------------------");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, state.hash, 32, ESP_LOG_INFO);
    if (memcmp(state.hash, node, CC1200_TROOPERS_HASH_SIZE) == 0) {
        memcpy(state.hash, msg->hash, CC1200_TROOPERS_HASH_SIZE);
        res = set_last_state(&state);
        if (res != ESP_OK) {
            return false;
        }
        return true;
    } else
        ESP_LOGW(TAG, "Hash mismatch");

    return false;
}

void cc1200_debug_start_countdown() {
    remaining_ticks = 65 * TICKS_PER_S;
    last_effect     = -1;
    ESP_LOGI(TAG, "Start countdown, %d", remaining_ticks);
    ESP_LOGI(TAG, "Start countdown, %d", portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start countdown, %f", TICKS_PER_MS);
    ESP_LOGI(TAG, "Start countdown, %f", TICKS_PER_S);
    xSemaphoreGive(driver_cc1200_troopers_task_mux);
}

bool cc1200_troopers_cb(cc1200_message *pkg) {
    cc1200_troopers_message_t *msg = (cc1200_troopers_message_t *) pkg->data;
    if (msg->magic != CC1200_TROOPERS_MAGIC) {
        return false;
    }

    if (pkg->len != sizeof(cc1200_troopers_message_t)) {
        ESP_LOGW(TAG, "Received invalid packet: len %d expected %d", pkg->len, sizeof(cc1200_troopers_message_t));
        return false;
    }

    if (!cc1200_troopers_replay_protection(msg)) {
        ESP_LOGW(TAG, "Received invalid packet: replay");
        return false;
    }

    cc1200_troopers_state_t value;

    switch (msg->type) {
        case CC1200_TROOPERS_MSGTYPE_COUNTDOWN:
            remaining_ticks = msg->payload * TICKS_PER_S;
            last_effect     = -1;
            if (!driver_cc1200_troopers_animation_running) {
                ESP_LOGI(TAG, "Start countdown");
                driver_cc1200_troopers_animation_running = true;
                xSemaphoreGive(driver_cc1200_troopers_task_mux);
            } else
                ESP_LOGI(TAG, "Setting time");
            break;
        case CC1200_TROOPERS_MSGTYPE_TALK:
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
            driver_neopixel_send_data((uint8_t *) "\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00", 18);  // Green
#endif
            break;
        case CC1200_TROOPERS_MSGTYPE_BREAK:
#ifdef CONFIG_DRIVER_NEOPIXEL_ENABLE
            driver_neopixel_send_data((uint8_t *) "\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff", 18);  // Blue
#endif
            break;
        case CC1200_TROOPERS_MSGTYPE_RESET:
            // actually quite dangerous as we do not sign the command type

            // value.msgid = 0;
            // memcpy(value.hash, CC1200_TROOPERS_CHAIN_TAIL, CC1200_TROOPERS_HASH_SIZE);
            // set_last_state(&value);
            // ESP_LOGI(TAG, "RESET");
            // break;
        case CC1200_TROOPERS_MSGTYPE_COUNTDOWN_SET_TIME:  // Duplicate as CC1200_TROOPERS_MSGTYPE_COUNTDOWN: does now similar things
            remaining_ticks = msg->payload * portTICK_PERIOD_MS;
            last_effect     = -1;
            ESP_LOGI(TAG, "Set remaining ms to: %d", remaining_ticks / portTICK_PERIOD_MS);
            break;
        default:
            ESP_LOGW(TAG, "Received invalid packet: msg type");
    }

    return true;
}
