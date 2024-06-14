#include "st25r3911b.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>
#include <soc/spi_reg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtc_wdt.h"
#include "st25r3911.h"
#include "st25r3911_com.h"
#include "st25r3911b_global.h"

static const char *TAG = "st25r3911b";

/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

#define NFC_LOG_SPI 0

#define MAX_HEX_STR_LENGTH 64
char hexStr[MAX_HEX_STR_LENGTH * 2];

static char* hex2Str(unsigned char * data, size_t dataLen) {
    unsigned char *pin = data;
    const char *hex = "0123456789ABCDEF";
    char *pout = hexStr;
    uint8_t i = 0;
    if(dataLen == 0) {
        hexStr[0] = 0;
    } else {
        for(; i < dataLen - 1 && i < MAX_HEX_STR_LENGTH - 1; ++i) {
            *pout++ = hex[(*pin>>4)&0xF];
            *pout++ = hex[(*pin++)&0xF];
        }
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin)&0xF];
        *pout = 0;
    }

    return hexStr;
}

esp_err_t st25r3911b_rxtx(ST25R3911B *device, const uint8_t* tx, const uint8_t* rx, uint8_t length) {
    if (device->spi_device == NULL) return ESP_FAIL;

    esp_err_t res = ESP_OK;
    size_t len = length * 8;

#if NFC_LOG_SPI == 1
    if (tx != NULL) {
        if ((tx[0] & 0xC0) == 0xC0) {
            ESP_LOGI(TAG, "Command %x %d", tx[0], length);
        }
    }

    if (tx != NULL) {
        if (tx[0] == 0x80) {
            ESP_LOGI(TAG, "FIFO W %d, %x %x %x %x" , length, tx[0], tx[1], tx[2], tx[3]);
        }
    }
#endif


    spi_transaction_t tx_transaction = {
        .length    = len,
        .rxlength  = len,
        .tx_buffer = (void*) tx,
        .rx_buffer = (void*) rx,
        .user      = (void *) device,
        .flags     = 0,
    };
    res = spi_device_transmit(device->spi_device, &tx_transaction);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "rxtx failed with %d", res);
        return res;
    }

#if NFC_LOG_SPI == 1
    if (tx != NULL && rx != NULL) {
        if (tx[0] == 0xBF) {
                ESP_LOGI(TAG, "FIFO R %d, %x %x %x %x" , length, rx[0], rx[1], rx[2], rx[3]);
        }
    }
#endif

    return res;
}

bool st25r3911b_get_discovery_prams(rfalNfcDiscoverParam * discParam) {
    if (discParam == NULL) {
        return false;
    }

    rfalNfcDefaultDiscParams( discParam );

    discParam->devLimit      = 1U;

    memcpy( discParam->nfcid3, NFCID3, sizeof(NFCID3) );
    memcpy( discParam->GB, GB, sizeof(GB) );
    discParam->GBLen         = sizeof(GB);
    discParam->p2pNfcaPrio   = true;

    discParam->notifyCb             = NULL;
    discParam->totalDuration        = 1000U;
    discParam->techs2Find           = RFAL_NFC_TECH_NONE;          /* For the demo, enable the NFC Technologies based on RFAL Feature switches */


#if RFAL_FEATURE_NFCA
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_A;
#endif /* RFAL_FEATURE_NFCA */

#if RFAL_FEATURE_NFCB
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_B;
#endif /* RFAL_FEATURE_NFCB */

#if RFAL_FEATURE_NFCF
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_F;
#endif /* RFAL_FEATURE_NFCF */

#if RFAL_FEATURE_NFCV
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_V;
#endif /* RFAL_FEATURE_NFCV */

#if RFAL_FEATURE_ST25TB
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_ST25TB;
#endif /* RFAL_FEATURE_ST25TB */

#if RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP
    discParam->techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE
    discParam->techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE

#if RFAL_SUPPORT_MODE_LISTEN_NFCA
    /* Set configuration for NFC-A CE */
    ST_MEMCPY( discParam->lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN );     /* Set SENS_RES / ATQA */
    ST_MEMCPY( discParam->lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04 );           /* Set NFCID / UID */
    discParam->lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;                                  /* Set NFCID length to 7 bytes */
    discParam->lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                        /* Set SEL_RES / SAK */

    discParam->techs2Find |= RFAL_NFC_LISTEN_TECH_A;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
    /* Set configuration for NFC-F CE */
    ST_MEMCPY( discParam->lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN );                 /* Set System Code */
    ST_MEMCPY( &ceNFCF_SENSF_RES[RFAL_NFCF_CMD_LEN], ceNFCF_nfcid2, RFAL_NFCID2_LEN );     /* Load NFCID2 on SENSF_RES */
    ST_MEMCPY( discParam->lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN );  /* Set SENSF_RES / Poll Response */

    discParam->techs2Find |= RFAL_NFC_LISTEN_TECH_F;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */
    return true;
}

esp_err_t st25r3911b_discover(rfalNfcDevice *nfcDevice, uint32_t timeout_ms) {
    rfalNfcDiscoverParam discParam;
    st25r3911b_get_discovery_prams(&discParam);

    ReturnCode err = rfalNfcDiscover( &discParam );
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "Failed to call rfalNfcDiscover");
        return ESP_FAIL;
    }

    int64_t end = esp_timer_get_time() / 1000 + timeout_ms;

    while(esp_timer_get_time() / 1000 < end) {
        rfalNfcWorker();
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (rfalNfcIsDevActivated(rfalNfcGetState())) {
            rfalNfcGetActiveDevice( &nfcDevice );
            vTaskDelay(50 / portTICK_PERIOD_MS);

            ESP_LOGD(TAG, "Discovered NFC device type=%d, uid=%s", nfcDevice->type, hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen ));
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );

            return ESP_OK;
        }
    }

    rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );
    return ESP_ERR_TIMEOUT;
}

esp_err_t st25r3911b_test() {
    esp_err_t res;

    uint8_t   id = 0;
    res = st25r3911b_chip_id(&id);
    if (res != ESP_OK) {
        return res;
    }

    switch (id) {
        case 0x02:
            ESP_LOGI(TAG, "silicon rev. r3.1");
            break;
        case 0x03:
            ESP_LOGI(TAG, "silicon rev. r3.3");
            break;
        case 0x04:
            ESP_LOGI(TAG, "silicon rev. r4.0");
            break;
        case 0x05:
            ESP_LOGI(TAG, "silicon rev. r4.1");
            break;
        default:
            ESP_LOGE(TAG, "Currently unknown silicon rev. 0x%02x", id);
            return ESP_FAIL;
    }
    static rfalNfcDevice *nfcDevice;

    return ESP_OK;
}

esp_err_t st25r3911b_chip_id(uint8_t *id) {
    if (global_st25r3911b == NULL) {
        ESP_LOGE(TAG, "chip_id called before global device was initialized");
    }
    return st25r3911CheckChipID(id) ? ESP_OK : ESP_FAIL;
}

_Noreturn static void intr_task(void* arg) {
    ST25R3911B* device = (ST25R3911B*) arg;

    while (1) {
        if (xSemaphoreTake(device->intr_trigger, portMAX_DELAY)) {
            ESP_LOGD(TAG, "Received interrupt");
            if (device->irq_callback != NULL) {
                device->irq_callback();
            } else {
                ESP_LOGI(TAG, "No NFC callback defined");
            }
        }
    }
}

static void intr_handler(void* arg) { /* in interrupt handler */
    ST25R3911B* device = (ST25R3911B*) arg;
    xSemaphoreGiveFromISR(device->intr_trigger, NULL);
    portYIELD_FROM_ISR();
}

esp_err_t st25r3911b_init(ST25R3911B *device) {
    esp_err_t res;

    if (device->pin_cs < 0) return ESP_FAIL;
    if (device->spi_speed > 6000000) return ESP_FAIL;

    if (device->spi_device == NULL) {
        spi_device_interface_config_t devcfg = {.command_bits     = 0,
                                                .address_bits     = 0,
                                                .dummy_bits       = 0,
                                                .mode             = 1,
                                                .duty_cycle_pos   = 128,
                                                .cs_ena_pretrans  = 1,
                                                .cs_ena_posttrans = 0,
                                                .clock_speed_hz   = device->spi_speed,
                                                .input_delay_ns   = 0,
                                                .spics_io_num     = -1,
                                                .queue_size = 1,
                                                .pre_cb     = NULL,
                                                .post_cb    = NULL};
        res                                  = spi_bus_add_device(device->spi_bus, &devcfg, &device->spi_device);
        if (res != ESP_OK) return res;
    }

    /* Setup CS */
    gpio_config_t dc_io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1LL << device->pin_cs,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    res = gpio_config(&dc_io_conf);
    if (res != ESP_OK) return res;
    res = gpio_set_level(device->pin_cs, true);
    if (res != ESP_OK) return res;


    if (device->pin_irq) {
        // Create interrupt trigger
        device->intr_trigger = xSemaphoreCreateBinary();
        if (device->intr_trigger == NULL) return ESP_ERR_NO_MEM;

        // Attach interrupt to interrupt pin (if available)
        res = gpio_isr_handler_add(device->pin_irq, intr_handler, (void*) device);
        if (res != ESP_OK) return res;

        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_POSEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << device->pin_irq,
            .pull_down_en = 0,
            .pull_up_en   = 0,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;

        xTaskCreate(&intr_task, "NFC interrupt task", 4096, device, 10, &device->intr_task_handle);
    }

    global_st25r3911b = device;

    ReturnCode err;

    err = st25r3911Initialize();
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "failed to st25r3911Initialize: %d", err);
        return ESP_FAIL;
    }

    err = rfalNfcInitialize();
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "failed to initialize");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Finished initializing. Starting self test");
    
    return st25r3911b_test();
}
