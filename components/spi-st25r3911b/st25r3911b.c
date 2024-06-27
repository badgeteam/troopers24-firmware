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
#include "rfal_platform.h"

static const char *TAG = "st25r3911b";

/* P2P communication data */
static uint8_t NFCID3_ACTIVE[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t NFCID3_PASSIVE[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

#define NFC_LOG_SPI 0

/* 4-byte UIDs with first byte 0x08 would need random number for the subsequent 3 bytes.
 * 4-byte UIDs with first byte 0x*F are Fixed number, not unique, use for this demo
 * 7-byte UIDs need a manufacturer ID and need to assure uniqueness of the rest.*/
static uint8_t ceNFCA_NFCID[]     = {0x5F, 'S', 'T', 'M'};    /* =_STM, 5F 53 54 4D NFCID1 / UID (4 bytes) */
static uint8_t ceNFCA_SENS_RES[]  = {0x02, 0x00};             /* SENS_RES / ATQA for 4-byte UID            */
static uint8_t ceNFCA_SEL_RES     = 0x20;                     /* SEL_RES / SAK                             */

static uint8_t              rawMessageBuf[MAX_NFC_BUFFER_SIZE];

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

bool st25r3911b_get_discovery_prams(rfalNfcDiscoverParam * discParam, st25r3911b_discover_mode discover_mode) {
    if (discParam == NULL) {
        return false;
    }

    rfalNfcDefaultDiscParams( discParam );

    discParam->devLimit      = 1U;

    memcpy( discParam->nfcid3, NFCID3_PASSIVE, sizeof(NFCID3_PASSIVE) );
    memcpy( discParam->GB, GB, sizeof(GB) );
    discParam->GBLen         = sizeof(GB);
    discParam->p2pNfcaPrio   = true;

    discParam->notifyCb             = NULL;
    discParam->totalDuration        = 1000U;
    discParam->techs2Find           = RFAL_NFC_TECH_NONE;          /* For the demo, enable the NFC Technologies based on RFAL Feature switches */
    discParam->techs2Find          |= RFAL_NFC_POLL_TECH_A;

    switch (discover_mode) {
        case DISCOVER_MODE_LISTEN_NFCA:
            discParam->techs2Find          |= RFAL_NFC_POLL_TECH_A;
            break;
        case DISCOVER_MODE_P2P_ACTIVE:
            memcpy( discParam->nfcid3, NFCID3_ACTIVE, sizeof(NFCID3_ACTIVE) );
            discParam->techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
            break;
        case DISCOVER_MODE_P2P_PASSIVE:
            discParam->techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
            break;
    }

    return true;
}

esp_err_t st25r3911b_discover(NfcDeviceCallback callback, uint32_t timeout_ms, st25r3911b_discover_mode discover_mode) {
    rfalNfcDevice nfcDevice = {0};
    rfalNfcDevice* pNfcDevice = &nfcDevice;
    rfalNfcDiscoverParam discParam;
    st25r3911b_get_discovery_prams(&discParam, discover_mode);

    ReturnCode err = rfalNfcDiscover( &discParam );
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "Failed to call rfalNfcDiscover");
        return ESP_FAIL;
    }

    int64_t end = esp_timer_get_time() / 1000 + timeout_ms;

    while(esp_timer_get_time() / 1000 < end) {
        rfalNfcWorker();
//        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (rfalNfcIsDevActivated(rfalNfcGetState())) {
            rfalNfcGetActiveDevice( &pNfcDevice );
//            vTaskDelay(50 / portTICK_PERIOD_MS);

            ESP_LOGI(TAG, "Discovered NFC device type=%d, uid=%s", pNfcDevice->type, hex2Str(pNfcDevice->nfcid, pNfcDevice->nfcidLen ));

            esp_err_t res = callback == NULL ? ESP_OK : callback(pNfcDevice);
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );
            return res;
        }
    }

    rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );
    return ESP_ERR_TIMEOUT;
}

esp_err_t st25r3911b_p2p_transceiveBlocking(uint32_t timeout_ms, uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt) {
    ReturnCode err;

    int64_t end = esp_timer_get_time() / 1000 + timeout_ms;
    err = rfalNfcDataExchangeStart( txBuf, txBufSize, rxData, rcvLen, fwt );
    if (err != ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start data exchange: %d", err);
        return ESP_FAIL;
    }
    do {
        if (esp_timer_get_time() / 1000 > end) {
            ESP_LOGE(TAG, "Timeout while transceiving");
            return ESP_ERR_TIMEOUT;
        }
        rfalNfcWorker();
        err = rfalNfcDataExchangeGetStatus();
    } while( err == ERR_BUSY );
    if (err != ERR_NONE) {
        ESP_LOGE(TAG, "Failed to transmit data: %d", err);
    }
    return ESP_OK;
}

esp_err_t st25r3911b_p2p_transmitBlocking(uint32_t timeout_ms, uint8_t *txBuf, uint16_t txBufSize) {
    uint16_t   *rxLen;
    uint8_t    *rxData;
    return st25r3911b_p2p_transceiveBlocking(timeout_ms, txBuf, txBufSize, &rxData, &rxLen, RFAL_FWT_NONE);
}

esp_err_t st25r3911b_p2p_receiveBlocking(uint32_t timeout_ms, uint8_t **rxData, uint16_t **rcvLen) {
    return st25r3911b_p2p_transceiveBlocking(timeout_ms, NULL, 0, rxData, rcvLen, RFAL_FWT_NONE);
}

static esp_err_t st25r3911b_activate_p2p(bool isActive, rfalNfcDepDevice *nfcDepDev) {
    rfalNfcDepAtrParam nfcDepParams;

    nfcDepParams.nfcid     = isActive ? NFCID3_ACTIVE : NFCID3_PASSIVE;
    nfcDepParams.nfcidLen  = RFAL_NFCDEP_NFCID3_LEN;
    nfcDepParams.BS        = RFAL_NFCDEP_Bx_NO_HIGH_BR;
#define ESP_BR BR
#undef BR
    nfcDepParams.BR        = RFAL_NFCDEP_Bx_NO_HIGH_BR;
#define BR ESP_BR
#undef ESP_BR
    nfcDepParams.LR        = RFAL_NFCDEP_LR_254;
    nfcDepParams.DID       = RFAL_NFCDEP_DID_NO;
    nfcDepParams.NAD       = RFAL_NFCDEP_NAD_NO;
    nfcDepParams.GBLen     = sizeof(GB);
    nfcDepParams.GB        = GB;
    nfcDepParams.commMode  = ((isActive) ? RFAL_NFCDEP_COMM_ACTIVE : RFAL_NFCDEP_COMM_PASSIVE);
    nfcDepParams.operParam = (RFAL_NFCDEP_OPER_FULL_MI_EN | RFAL_NFCDEP_OPER_EMPTY_DEP_DIS | RFAL_NFCDEP_OPER_ATN_EN | RFAL_NFCDEP_OPER_RTOX_REQ_EN);

    /* Initialize NFC-DEP protocol layer */
    rfalNfcDepInitialize();

    /* Handle NFC-DEP Activation (ATR_REQ and PSL_REQ if applicable) */
    return rfalNfcDepInitiatorHandleActivation( &nfcDepParams, RFAL_BR_424, nfcDepDev );
}

esp_err_t st25r3911b_p2p_active(rfalNfcDepDevice *nfcDepDev) {
    ReturnCode res;

    res = rfalSetMode(RFAL_MODE_POLL_ACTIVE_P2P, RFAL_BR_424, RFAL_BR_424);
    if (res != RFAL_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set mode: %d", res);
        return ESP_FAIL;
    }

    rfalSetErrorHandling(RFAL_ERRORHANDLING_EMD);
    rfalSetFDTListen(RFAL_FDT_LISTEN_AP2P_POLLER);
    rfalSetFDTPoll(RFAL_TIMING_NONE);

    rfalSetGT( RFAL_GT_AP2P_ADJUSTED );
    res = rfalFieldOnAndStartGT();
    if (res != RFAL_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to turn field on: %d", res);
        return ESP_FAIL;
    }

    res = st25r3911b_activate_p2p(true, nfcDepDev);
    if (res == RFAL_ERR_NONE) {
        ESP_LOGI(TAG, "Discovered NFC device uid=%s", hex2Str(nfcDepDev->activation.Target.ATR_RES.NFCID3, RFAL_NFCDEP_NFCID3_LEN));
    }

    rfalFieldOff();
    return res;
}

esp_err_t st25r3911b_poll_active_p2p(uint32_t timeout_ms) {
    rfalFieldOff();
    rfalWakeUpModeStop();
    vTaskDelay(pdMS_TO_TICKS(300));

    int64_t end = esp_timer_get_time() / 1000 + timeout_ms;

    rfalWakeUpModeStart(NULL);

    bool wokeup = false;

    while(esp_timer_get_time() / 1000 < end) {
        if(rfalWakeUpModeHasWoke()) {
            /* If awake, go directly to Poll */
            rfalWakeUpModeStop();
            wokeup = true;
            break;
        }
    }

    if (!wokeup) {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "wakeup complete");

    ReturnCode res;
    rfalNfcDepDevice nfcDepDev;
    while(esp_timer_get_time() / 1000 < end) {
        res = st25r3911b_p2p_active(&nfcDepDev);
        if (res == ESP_OK) {
            return res;
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t st25r3911b_listen_nfca() {
    ReturnCode        err;
    bool              found = false;
    uint8_t           devIt = 0;
    rfalNfcaSensRes   sensRes;
    rfalIsoDepDevice  isoDepDev;                                         /* ISO-DEP Device details                          */
    rfalNfcDepDevice  nfcDepDev;                                         /* NFC-DEP Device details                          */

    rfalNfcaPollerInitialize();   /* Initialize for NFC-A */
    rfalFieldOnAndStartGT();      /* Turns the Field On if not already and start GT timer */

    err = rfalNfcaPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensRes );
    if(err == ERR_NONE)
    {
        rfalNfcaListenDevice nfcaDevList[1];
        uint8_t                   devCnt;

        err = rfalNfcaPollerFullCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, 1, nfcaDevList, &devCnt);

        if ( (err == ERR_NONE) && (devCnt > 0) )
        {
            found = true;
            devIt = 0;

            platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);

            /* Check if it is Topaz aka T1T */
            if( nfcaDevList[devIt].type == RFAL_NFCA_T1T )
            {
                /********************************************/
                /* NFC-A T1T card found                     */
                /* NFCID/UID is contained in: t1tRidRes.uid */
                platformLog("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2Str(nfcaDevList[devIt].ridRes.uid, RFAL_T1T_UID_LEN));
            }
            else
            {
                /*********************************************/
                /* NFC-A device found                        */
                /* NFCID/UID is contained in: nfcaDev.nfcId1 */
                platformLog("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str(nfcaDevList[0].nfcId1, nfcaDevList[0].nfcId1Len));
            }


            /* Check if device supports P2P/NFC-DEP */
            if( (nfcaDevList[devIt].type == RFAL_NFCA_NFCDEP) || (nfcaDevList[devIt].type == RFAL_NFCA_T4T_NFCDEP))
            {
                /* Continue with P2P Activation .... */

                err = st25r3911b_activate_p2p(false, &nfcDepDev );
                if (err == ERR_NONE)
                {
                    /*********************************************/
                    /* Passive P2P device activated              */
                    platformLog("NFCA Passive P2P device found. NFCID: %s\r\n", hex2Str(nfcDepDev.activation.Target.ATR_RES.NFCID3, RFAL_NFCDEP_NFCID3_LEN));

                    /* Send an URI record */
//                    demoSendNdefUri();
                }
            }
            /* Check if device supports ISO14443-4/ISO-DEP */
            else if (nfcaDevList[devIt].type == RFAL_NFCA_T4T)
            {
                /* Activate the ISO14443-4 / ISO-DEP layer */

                rfalIsoDepInitialize();
                err = rfalIsoDepPollAHandleActivation((rfalIsoDepFSxI)RFAL_ISODEP_FSDI_DEFAULT, RFAL_ISODEP_NO_DID, RFAL_BR_424, &isoDepDev);
                if( err == ERR_NONE )
                {
                    platformLog("ISO14443-4/ISO-DEP layer activated. \r\n");

                    /* Exchange APDUs */
//                    demoSendAPDUs();
                }
            }
        }
    }
    return ESP_OK;
}

#define DEMO_BUF_LEN 255
static rfalNfcDepBufFormat      nfcDepRxBuf;                                /* NFC-DEP Rx buffer format (with header/prologue) */
static uint8_t                  rxBuf[DEMO_BUF_LEN];                        /* Generic buffer abstraction                      */
static uint16_t           gRxLen;
static bool               gIsRxChaining; /*!< Received data is not complete   */
static rfalNfcDepDevice   gNfcDepDev;    /*!< NFC-DEP device info             */

static bool handle_listen(uint8_t *state, rfalLmState *lmSt, rfalBitRate *bitRate, bool *dataFlag, uint8_t *hdrLen) {
    switch(*state){
        case 0:
            *lmSt = rfalListenGetState( dataFlag, bitRate );         /* Check if Initator has sent some data */
            if( (*lmSt != RFAL_LM_STATE_IDLE)) {
                break;
            }
            ESP_LOGI(TAG, "RFAL_LM_STATE_IDLE: dataFlag=%d, bitRate=%d", *dataFlag, *bitRate);
            if (!*dataFlag) {
                break;
            }
            *state = *state + 1;
        case 1:
            /* SB Byte only in NFC-A */
            if (*bitRate == RFAL_BR_106) {
                *hdrLen += RFAL_NFCDEP_SB_LEN;
            }
            ESP_LOGI(TAG, "Using %d as hdrLen", *hdrLen);
            *state = *state + 1;
        case 2:
            ESP_LOGI(TAG, "%s", hex2Str(rxBuf, rfalConvBitsToBytes(gRxLen)));
            if(!rfalNfcDepIsAtrReq( &rxBuf[*hdrLen], (rfalConvBitsToBytes(gRxLen) - *hdrLen), NULL ) ) {
                ESP_LOGI(TAG, "not rfalNfcDepIsAtrReq: len=%d", rfalConvBitsToBytes(gRxLen));
                break;
            }
            rfalNfcDepTargetParam      param;
            rfalNfcDepListenActvParam  rxParam;

            rfalListenSetState((RFAL_BR_106 == *bitRate) ? RFAL_LM_STATE_TARGET_A : RFAL_LM_STATE_TARGET_F);
            rfalSetMode( RFAL_MODE_LISTEN_ACTIVE_P2P, *bitRate, *bitRate);

            platformLog(" Activated as AP2P listener device \r\n" );

            memcpy(param.nfcid3, NFCID3_PASSIVE, RFAL_NFCDEP_NFCID3_LEN);
            param.bst = RFAL_NFCDEP_Bx_NO_HIGH_BR;
            param.brt = RFAL_NFCDEP_Bx_NO_HIGH_BR;
            param.to = RFAL_NFCDEP_WT_TRG_MAX_D11;
            param.ppt = (RFAL_NFCDEP_LR_254 << RFAL_NFCDEP_PP_LR_SHIFT);
            param.GBtLen = 0;
            param.operParam = (RFAL_NFCDEP_OPER_FULL_MI_DIS | RFAL_NFCDEP_OPER_EMPTY_DEP_EN | RFAL_NFCDEP_OPER_ATN_EN | RFAL_NFCDEP_OPER_RTOX_REQ_EN);
            param.commMode = RFAL_NFCDEP_COMM_ACTIVE;

            rxParam.rxBuf        = &nfcDepRxBuf;
            rxParam.rxLen        = &gRxLen;
            rxParam.isRxChaining = &gIsRxChaining;
            rxParam.nfcDepDev    = &gNfcDepDev;

            ReturnCode     res;

            /* ATR_REQ received, trigger NFC-DEP layer to handle activation (sends ATR_RES and handles PSL_REQ)  */
            res = rfalNfcDepListenStartActivation( &param, &rxBuf[*hdrLen], (rfalConvBitsToBytes(gRxLen) - *hdrLen), rxParam );
            if(res != RFAL_ERR_NONE) {
                ESP_LOGE(TAG, "rfalNfcDepListenStartActivation: %d", res);
                return false;
            }

            *state = *state + 1;
        case 3:
            res = rfalNfcDepListenGetActivationStatus();
            if( res == RFAL_ERR_BUSY ){
                break;
            }

            if (res != RFAL_ERR_NONE) {
                ESP_LOGE(TAG, "rfalNfcDepListenGetActivationStatus: %d", res);
                return false;
            }

            *state = *state + 1;
        case 4:
            res = rfalNfcDepGetTransceiveStatus();
            if( res == RFAL_ERR_BUSY ){
                break;
            }
            if( res != RFAL_ERR_NONE ){
                ESP_LOGE(TAG, "rfalNfcDepGetTransceiveStatus: %d", res);
                return false;
            }

            rfalNfcDepTxRxParam rfalNfcDepTxRx;

            platformLog(" Received %d bytes of data: %s\r\n", gRxLen, hex2Str((uint8_t*)nfcDepRxBuf.inf, gRxLen) );

            /* Loop/Send back the same data that has been received */
            rfalNfcDepTxRx.txBuf        = &nfcDepRxBuf;
            rfalNfcDepTxRx.txBufLen     = gRxLen;
            rfalNfcDepTxRx.rxBuf        = &nfcDepRxBuf;
            rfalNfcDepTxRx.rxLen        = &gRxLen;
            rfalNfcDepTxRx.DID          = RFAL_NFCDEP_DID_NO;
            rfalNfcDepTxRx.FSx          = rfalNfcDepLR2FS( rfalNfcDepPP2LR( gNfcDepDev.activation.Initiator.ATR_REQ.PPi ) );
            rfalNfcDepTxRx.FWT          = gNfcDepDev.info.FWT;
            rfalNfcDepTxRx.dFWT         = gNfcDepDev.info.dFWT;
            rfalNfcDepTxRx.isRxChaining = &gIsRxChaining;
            rfalNfcDepTxRx.isTxChaining = gIsRxChaining;

            res = rfalNfcDepStartTransceive( &rfalNfcDepTxRx );
            if (res != RFAL_ERR_NONE) {
                ESP_LOGE(TAG, "rfalNfcDepStartTransceive: %d", res);
                return false;
            }
    }
    return true;
}

esp_err_t st25r3911b_listen_p2p(uint32_t timeout_ms) {
    ReturnCode     res;
    bool           dataFlag;
    rfalLmState    lmSt;
    rfalBitRate    bitRate;
    uint8_t        hdrLen = RFAL_NFCDEP_LEN_LEN;

    rfalFieldOff();
    res = rfalListenStart( RFAL_LM_MASK_ACTIVE_P2P, NULL, NULL, NULL, rxBuf, DEMO_BUF_LEN, &gRxLen );
    if (res != RFAL_ERR_NONE) {
        ESP_LOGE(TAG, "failed to start listening: %d", res);
        rfalFieldOff();
        return ESP_FAIL;
    }

    uint8_t state = 0;

    int64_t end = esp_timer_get_time() / 1000 + timeout_ms;
    while(esp_timer_get_time() / 1000 < end) {
        rfalWorker();
        if (!handle_listen(&state, &lmSt, &bitRate, &dataFlag, &hdrLen)) {
            res = ESP_FAIL;
            break;
        }
    }

    rfalListenStop();
    rfalFieldOff();
    return res == RFAL_ERR_NONE ? ESP_ERR_TIMEOUT : res;
}

esp_err_t st25r3911b_read_data(rfalNfcDevice *nfcDevice, ndefConstBuffer* bufConstRawMessage) {
    if (nfcDevice == NULL) {
        ESP_LOGE(TAG, "No NFC device given");
        return ESP_ERR_INVALID_ARG;
    }
    if (nfcDevice->type != RFAL_NFC_LISTEN_TYPE_NFCA) {
        ESP_LOGE(TAG, "Invalid NFC device type: %d", nfcDevice->type);
        return ESP_ERR_INVALID_ARG;
    }

    ndefContext      ndefCtx;
    uint32_t         rawMessageLen;
    ndefInfo         info;

    /*
     * Perform NDEF Context Initialization
     */
    ReturnCode err = ndefPollerContextInitialization(&ndefCtx, nfcDevice);
    if( err != RFAL_ERR_NONE ) {
        ESP_LOGE(TAG, "context init failed: %d", err);
        return ESP_FAIL;
    }
    /*
     * Perform NDEF Detect procedure
     */
    err = ndefPollerNdefDetect(&ndefCtx, &info);
    if(err != RFAL_ERR_NONE) {
        ESP_LOGE(TAG, "NFC tag not found");
        return ESP_FAIL;
    }

    err = ndefPollerReadRawMessage(&ndefCtx, rawMessageBuf, sizeof(rawMessageBuf), &rawMessageLen, true);
    if(err != RFAL_ERR_NONE) {
        return ESP_FAIL;
    }
    bufConstRawMessage->buffer = rawMessageBuf;
    bufConstRawMessage->length = rawMessageLen;

    //    err = ndefMessageDecode(&bufConstRawMessage, &message);
    //    if (err != RFAL_ERR_NONE) {
    //        return ESP_FAIL;
    //    }

    ESP_LOGI(TAG, "found message with length %d", rawMessageLen);

    //    err = ndefMessageDump(&message, verbose);
    //    if (err != RFAL_ERR_NONE) {
    //        return ESP_FAIL;
    //    }

    rfalNfcaPollerSleep();
    return ESP_OK;
}

static ReturnCode transceiveBlocking(uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt) {
    ReturnCode err;

    err = rfalNfcDataExchangeStart(txBuf, txBufSize, rxData, rcvLen, fwt);
    if (err == RFAL_ERR_NONE) {
        do {
            rfalNfcWorker();
            err = rfalNfcDataExchangeGetStatus();
        } while(err == RFAL_ERR_BUSY);
    }
    return err;
}

esp_err_t st25r3911b_p2p_listen(rfalNfcDevice *nfcDevice, ndefConstBuffer* bufConstRawMessage) {
    if (nfcDevice == NULL) {
        ESP_LOGE(TAG, "No NFC device given");
        return ESP_ERR_INVALID_ARG;
    }
    //                    case RFAL_NFC_LISTEN_TYPE_AP2P:
    //                    case RFAL_NFC_POLL_TYPE_AP2P:
    if (nfcDevice->type != RFAL_NFC_POLL_TYPE_NFCA) {
        ESP_LOGE(TAG, "Invalid NFC device type: %d", nfcDevice->type);
        return ESP_ERR_INVALID_ARG;
    }
    if (nfcDevice->rfInterface != RFAL_NFC_INTERFACE_NFCDEP) {
        ESP_LOGE(TAG, "NFC DEP not supported: %d", nfcDevice->rfInterface);
        return ESP_ERR_INVALID_ARG;
    }

//
//
//    ReturnCode err = RFAL_ERR_INTERNAL;
//    uint8_t *rxData;
//    uint16_t *rcvLen;
//    uint8_t  txBuf[150];
//    uint16_t txLen;
//
//    do
//    {
//        rfalNfcWorker();
//
//        switch( rfalNfcGetState() )
//        {
//            case RFAL_NFC_STATE_ACTIVATED:
//                err = transceiveBlocking( NULL, 0, &rxData, &rcvLen, 0);
//                break;
//
//            case RFAL_NFC_STATE_DATAEXCHANGE:
//            case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
//
//                txLen = demoCeT4T( rxData, *rcvLen, txBuf, sizeof(txBuf) );
//                err   = transceiveBlocking( txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE );
//                break;
//
//            case RFAL_NFC_STATE_START_DISCOVERY:
//                return ESP_OK;
//
//            case RFAL_NFC_STATE_LISTEN_SLEEP:
//            default:
//                break;
//        }
//    }
//    while( (err == RFAL_ERR_NONE) || (err == RFAL_ERR_SLEEP_REQ) );
//    if (err != RFAL_ERR_NONE) {
//        ESP_LOGE(TAG, "Failed to write to NFC device: %d", err);
//        return ESP_FAIL;
//    }
    return ESP_OK;
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
