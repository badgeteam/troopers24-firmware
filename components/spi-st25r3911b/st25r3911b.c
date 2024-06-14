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


esp_err_t rfid_reg_read(ST25R3911B *device, uint8_t reg, uint8_t *value) {
    if (device->spi_device == NULL) return ESP_FAIL;
    if (reg > 0x3f) return ESP_ERR_INVALID_ARG;

    platformProtectST25RComm();
    platformSpiSelect();

    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = 0x40 | reg;
    tx[1] = 0;
    rx[0] = 0xff;
    rx[1] = 0xff;

    esp_err_t res = platformSpiTxRx( tx, rx, 2 );

    platformSpiDeselect();
    platformUnprotectST25RComm();

    *value = rx[1];
    return res;
}

/* TODO: Combine with read ?, only 1 byte different*/
esp_err_t rfid_reg_write(ST25R3911B *device, uint8_t reg, uint8_t value) {
    if (device->spi_device == NULL) return ESP_FAIL;
    if (reg > 0x3f) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2];
    tx[0]                         = 0x00 | reg;
    tx[1]                         = value;
    spi_transaction_t transaction = {
        .length    = 16,  // transaction length is in bits
        .tx_buffer = &tx,
        .user = (void *) device,
    };

    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}

esp_err_t rfid_rxtx(ST25R3911B *device, const uint8_t* tx, const uint8_t* rx, uint8_t length) {
    if (device->spi_device == NULL) return ESP_FAIL;

//    ESP_LOGI(TAG, "rfid_rxtx, tx = %d, rx = %d, length = %d", tx != NULL, rx != NULL, length);
    esp_err_t res = ESP_OK;
    size_t len = length * 8;

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


    spi_transaction_t tx_transaction = {
        .length    = len,
        .rxlength  = len,
        .tx_buffer = (void*) tx,
        .rx_buffer = (void*) rx,
        .user      = (void *) device,
    };
    res = spi_device_transmit(device->spi_device, &tx_transaction);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "rfix_rxtx failed with %d", res);
        return res;
    }

    if (tx != NULL && rx != NULL) {
        if (tx[0] == 0xBF) {
                ESP_LOGI(TAG, "FIFO R %d, %x %x %x %x" , length, rx[0], rx[1], rx[2], rx[3]);
        }
    }

    return res;
}

/* TODO: Combine with read ?, only 1 byte different*/
esp_err_t rfid_send_command(ST25R3911B *device, uint8_t command) {
    if (device->spi_device == NULL) return ESP_FAIL;

    /* Dont check. ST include the flags in the value */
    //    if (command > 0x3f) return ESP_ERR_INVALID_ARG;

    uint8_t data = 0xC0 | command;

    spi_transaction_t transaction = {
        .length    = 8,  // transaction length is in bits
        .tx_buffer = &data,
        .user      = (void *) device,
    };
    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}

esp_err_t st25r_test() {
    ReturnCode err;


    rtc_wdt_disable();

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
    /* This is the same sequence as st25r3911Initialize() in en.STSW-ST25RFAL001/source/st25r3911/st25r3911.c*/
//    rfid_send_command(device, ST25R3911_CMD_SET_DEFAULT);                       /* 0xc1*/
//    rfid_reg_write(device, ST25R3911_REG_OP_CONTROL, 0);                        /* 0x02 0x00*/
//    rfid_reg_write(device, ST25R3911_REG_IO_CONF1, ST25R3911_REG_IO_CONF1_osc); /* 0x00 0x08*/
//    rfid_reg_write(device, ST25R3911_REG_IO_CONF2, 0);                          /* 0x01 0x00*/
//
//    uint8_t rmr = 0;
//    rfid_reg_read(device, ST25R3911_REG_IO_CONF2, &rmr);                                                                     /* 0x41 0xNN */
//    rfid_reg_write(device, ST25R3911_REG_IO_CONF2, rmr | ST25R3911_REG_IO_CONF2_miso_pd1 | ST25R3911_REG_IO_CONF2_miso_pd2); /* 0x01 0x18*/

    uint8_t   id = 0;
    if( !st25r3911CheckChipID( &id ) ) {
        ESP_LOGE(TAG, "failed to read chip ID");
        return RFAL_ERR_HW_MISMATCH;
    }

    switch (id & ST25R3911_REG_IC_IDENTITY_mask_ic_rev) {
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
            ESP_LOGE(TAG, "Currently unknown silicon rev. %x", id & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
            return ESP_FAIL;
    }
    static rfalNfcDevice *nfcDevice;

    static rfalNfcDiscoverParam discParam;

    rfalNfcDefaultDiscParams( &discParam );

    discParam.devLimit      = 1U;

    memcpy( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
    memcpy( &discParam.GB, GB, sizeof(GB) );
    discParam.GBLen         = sizeof(GB);
    discParam.p2pNfcaPrio   = true;

    discParam.notifyCb             = NULL;
    discParam.totalDuration        = 1000U;
    discParam.techs2Find           = RFAL_NFC_TECH_NONE;          /* For the demo, enable the NFC Technologies based on RFAL Feature switches */


#if RFAL_FEATURE_NFCA
    discParam.techs2Find          |= RFAL_NFC_POLL_TECH_A;
#endif /* RFAL_FEATURE_NFCA */

#if RFAL_FEATURE_NFCB
    discParam.techs2Find          |= RFAL_NFC_POLL_TECH_B;
#endif /* RFAL_FEATURE_NFCB */

#if RFAL_FEATURE_NFCF
    discParam.techs2Find          |= RFAL_NFC_POLL_TECH_F;
#endif /* RFAL_FEATURE_NFCF */

#if RFAL_FEATURE_NFCV
    discParam.techs2Find          |= RFAL_NFC_POLL_TECH_V;
#endif /* RFAL_FEATURE_NFCV */

#if RFAL_FEATURE_ST25TB
    discParam.techs2Find          |= RFAL_NFC_POLL_TECH_ST25TB;
#endif /* RFAL_FEATURE_ST25TB */

#if RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP
    discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE
    discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE

#if RFAL_SUPPORT_MODE_LISTEN_NFCA
    /* Set configuration for NFC-A CE */
    ST_MEMCPY( discParam.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN );     /* Set SENS_RES / ATQA */
    ST_MEMCPY( discParam.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04 );           /* Set NFCID / UID */
    discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;                                  /* Set NFCID length to 7 bytes */
    discParam.lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                        /* Set SEL_RES / SAK */

    discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
    /* Set configuration for NFC-F CE */
    ST_MEMCPY( discParam.lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN );                 /* Set System Code */
    ST_MEMCPY( &ceNFCF_SENSF_RES[RFAL_NFCF_CMD_LEN], ceNFCF_nfcid2, RFAL_NFCID2_LEN );     /* Load NFCID2 on SENSF_RES */
    ST_MEMCPY( discParam.lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN );  /* Set SENSF_RES / Poll Response */

    discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_F;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */

    ESP_LOGI(TAG, "state %d", rfalNfcGetState());

    err = rfalNfcDiscover( &discParam );
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "Failed to call rfalNfcDiscover");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "state %d", rfalNfcGetState());


    while(1) {
        rfalNfcWorker();
//        ESP_LOGI(TAG, "state %d", rfalNfcGetState());
        rtc_wdt_feed();
//        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (rfalNfcIsDevActivated(rfalNfcGetState())) {
            rfalNfcGetActiveDevice( &nfcDevice );
            vTaskDelay(50 / portTICK_PERIOD_MS);

            ESP_LOGI(TAG, "Device type %d", nfcDevice->type);
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    return ESP_OK;
}

esp_err_t st25r3911b_init(ST25R3911B *device) {
    esp_err_t res;

    if (device->pin_cs < 0) return ESP_FAIL;
    if (device->spi_speed > 6000000) return ESP_FAIL;

    /*if (device->mutex == NULL) {
        device->mutex = xSemaphoreCreateMutex();
    }*/

    //    if (device->mutex != NULL) xSemaphoreGive(device->mutex);

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
//                                                .flags            = SPI_DEVICE_HALFDUPLEX,
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


    global_st25r3911b = device;

    return st25r_test();

    //    if (device->callback != NULL) {
    //        device->callback(false);
    //    }

//    return ESP_OK;
}
