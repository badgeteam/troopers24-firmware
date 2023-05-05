/*
 * Copyright (c) 2015, Weptech elektronik GmbH Germany
 * http://www.weptech.de
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "include/cc1200_troopers.h"
#include "include/driver_cc1200.h"
#include "include/driver_cc1200_statemachine.h"

/*
 * RF test mode. Blocks inside "driver_cc1200_configure()".
 * - Set this parameter to 1 in order to produce an modulated carrier (PN9)
 * - Set this parameter to 2 in order to produce an unmodulated carrier
 * - Set this parameter to 3 in order to switch to rx synchronous mode
 * The channel is set according to CC1200_DEFAULT_CHANNEL
 */
#ifndef CC1200_RF_TESTMODE
#define CC1200_RF_TESTMODE 0
#endif

#if CC1200_RF_TESTMODE
#undef CC1200_RF_CFG
#if CC1200_RF_TESTMODE == 1
#define CC1200_RF_CFG cc1200_802154g_863_870_fsk_50kbps
#elif CC1200_RF_TESTMODE == 2
#define CC1200_RF_CFG cc1200_802154g_863_870_fsk_50kbps
#elif CC1200_RF_TESTMODE == 3
#define CC1200_RF_CFG cc1200_802154g_863_870_fsk_50kbps
#endif
#endif


static const char *TAG = "driver_cc1200";

/*---------------------------------------------------------------------------*/
/* Configure the radio (write basic configuration). */
void driver_cc1200_configure(CC1200* device) {
    uint8_t reg;
#if CC1200_RF_TESTMODE
    uint32_t freq;
#endif
    INFOS("Configure begin");
    ESP_LOGI(TAG, "marc state %d", single_read(device, CC1200_MARCSTATE) & 0x1f);
    /*
     * As we only write registers which are different from the chip's reset
     * state, let's assure that the chip is in a clean state
     */
    cc1200_reset(device);

    /* Write the configuration as exported from SmartRF Studio */
    write_reg_settings(device, CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

    /* Write frequency offset */
#if CC1200_FREQ_OFFSET
    /* MSB */
    single_write(device, CC1200_FREQOFF1, (uint8_t) (CC1200_FREQ_OFFSET >> 8));
    /* LSB */
    single_write(device, CC1200_FREQOFF0, (uint8_t) (CC1200_FREQ_OFFSET));
#endif

    /* RSSI offset */
    single_write(device, CC1200_AGC_GAIN_ADJUST, (int8_t) CC1200_RF_CFG.rssi_offset);

    /***************************************************************************
     * RF test modes needed during hardware development
     **************************************************************************/

#if (CC1200_RF_TESTMODE == 1) || (CC1200_RF_TESTMODE == 2)

    strobe(device, CC1200_SFTX);
    single_write(device, CC1200_TXFIRST, 0);
    single_write(device, CC1200_TXLAST, 0);
    driver_cc1200_set_txpower(CC1200_CONST_TX_POWER_MAX);
    single_write(device, CC1200_PKT_CFG2, 0x02);
    freq = calculate_freq(CC1200_DEFAULT_CHANNEL - CC1200_RF_CFG.min_channel);
    single_write(device, CC1200_FREQ0, ((uint8_t *) &freq)[0]);
    single_write(device, CC1200_FREQ1, ((uint8_t *) &freq)[1]);
    single_write(device, CC1200_FREQ2, ((uint8_t *) &freq)[2]);

    printf("RF: Freq0 0x%02x", ((uint8_t *) &freq)[0]);
    printf("RF: Freq1 0x%02x", ((uint8_t *) &freq)[1]);
    printf("RF: Freq2 0x%02x", ((uint8_t *) &freq)[2]);

#if (CC1200_RF_TESTMODE == 1)
    single_write(device, CC1200_SYNC_CFG1, 0xE8);
    single_write(device, CC1200_PREAMBLE_CFG1, 0x00);
    single_write(device, CC1200_MDMCFG1, 0x46);
    single_write(device, CC1200_PKT_CFG0, 0x40);
    single_write(device, CC1200_FS_DIG1, 0x07);
    single_write(device, CC1200_FS_DIG0, 0xAA);
    single_write(device, CC1200_FS_DVC1, 0xFF);
    single_write(device, CC1200_FS_DVC0, 0x17);
#endif

#if (CC1200_RF_TESTMODE == 2)
    single_write(device, CC1200_SYNC_CFG1, 0xE8);
    single_write(device, CC1200_PREAMBLE_CFG1, 0x00);
    single_write(device, CC1200_MDMCFG1, 0x06);
    single_write(device, CC1200_PA_CFG1, 0x3F);
    single_write(device, CC1200_MDMCFG2, 0x03);
    single_write(device, CC1200_FS_DIG1, 0x07);
    single_write(device, CC1200_FS_DIG0, 0xAA);
    single_write(device, CC1200_FS_DVC0, 0x17);
    single_write(device, CC1200_SERIAL_STATUS, 0x08);
#endif

    strobe(device, CC1200_STX);

    while (1) {
#if (CC1200_RF_TESTMODE == 1)
        vTaskDelay(10000);
        leds_off(LEDS_YELLOW);
        leds_on(LEDS_RED);
        vTaskDelay(10000);
        leds_off(LEDS_RED);
        leds_on(LEDS_YELLOW);
#else
        vTaskDelay(10000);
        leds_off(LEDS_GREEN);
        leds_on(LEDS_RED);
        vTaskDelay(10000);
        leds_off(LEDS_RED);
        leds_on(LEDS_GREEN);
#endif
    }

#elif (CC1200_RF_TESTMODE == 3)

    /* CS on GPIO3 */
    single_write(CC1200_IOCFG3, CC1200_IOCFG_CARRIER_SENSE);
    single_write(CC1200_IOCFG2, CC1200_IOCFG_SERIAL_CLK);
    single_write(CC1200_IOCFG0, CC1200_IOCFG_SERIAL_RX);
    driver_cc1200_set_cca_threshold(CC1200_RF_CFG.cca_threshold);
    freq = calculate_freq(CC1200_DEFAULT_CHANNEL - CC1200_RF_CFG.min_channel);
    single_write(CC1200_FREQ0, ((uint8_t *) &freq)[0]);
    single_write(CC1200_FREQ1, ((uint8_t *) &freq)[1]);
    single_write(CC1200_FREQ2, ((uint8_t *) &freq)[2]);
    strobe(CC1200_SRX);

    while (1) {
        vTaskDelay(10000);
        leds_off(LEDS_GREEN);
        leds_on(LEDS_YELLOW);
        vTaskDelay(10000);
        leds_off(LEDS_YELLOW);
        leds_on(LEDS_GREEN);
        clock_delay_usec(1000);

        /* CS on GPIO3 */
        if (cc1200_arch_gpio3_read_pin() == 1) {
            leds_on(LEDS_RED);
        } else {
            leds_off(LEDS_RED);
        }
    }

#endif /* #if CC1200_RF_TESTMODE == ... */

    /***************************************************************************
     * Set the stuff we need for this driver to work. Don't touch!
     **************************************************************************/

    /* GPIOx configuration */
    single_write(device, CC1200_IOCFG3, GPIO3_IOCFG);
    single_write(device, CC1200_IOCFG2, GPIO2_IOCFG);
    single_write(device, CC1200_IOCFG0, GPIO0_IOCFG);

    reg = single_read(device, CC1200_SETTLING_CFG);
    /*
     * Turn of auto calibration. This gives us better control
     * over the timing (RX/TX & TX /RX turnaround!). We calibrate manually:
     * - Upon wake-up (on())
     * - Before going to TX (transmit())
     * - When setting an new channel (set_channel())
     */
    reg &= ~(3 << 3);
#if CC1200_AUTOCAL
    /* We calibrate when going from idle to RX or TX */
    reg |= (1 << 3);
#endif
    single_write(device, CC1200_SETTLING_CFG, reg);

    /* Configure RXOFF_MODE */
    reg = single_read(device, CC1200_RFEND_CFG1);
    reg &= ~(3 << 4); /* RXOFF_MODE = IDLE */
#if RXOFF_MODE_RX
    reg |= (3 << 4);  /* RXOFF_MODE = RX */
#endif
    reg |= 0x0F;      /* Disable RX timeout */
    single_write(device, CC1200_RFEND_CFG1, reg);

    /* Configure TXOFF_MODE */
    reg = single_read(device, CC1200_RFEND_CFG0);
    reg &= ~(3 << 4); /* TXOFF_MODE = IDLE */
#if TXOFF_MODE_RX
    reg |= (3 << 4);  /* TXOFF_MODE = RX */
#endif
    single_write(device, CC1200_RFEND_CFG0, reg);

    /*
     * CCA Mode 0: Always give clear channel indication.
     * CCA is done "by hand". Keep in mind: automatic CCA would also
     * affect the transmission of the ACK and is not implemented yet!
     */
#if CC1200_802154G
    single_write(device, CC1200_PKT_CFG2, (1 << 5));
#else
    single_write(device, CC1200_PKT_CFG2, 0x00);
#endif

    /* Configure appendix */
    reg = single_read(device, CC1200_PKT_CFG1);
#if APPEND_STATUS
    reg |= (1 << 0);
#else
    reg &= ~(1 << 0);
#endif
    single_write(device, CC1200_PKT_CFG1, reg);

    /* Variable packet length mode */
    reg = single_read(device, CC1200_PKT_CFG0);
    reg &= ~(3 << 5);
    reg |= (1 << 5);
    single_write(device, CC1200_PKT_CFG0, reg);

#ifdef FIFO_THRESHOLD
    /* FIFO threshold */
    single_write(device, CC1200_FIFO_CFG, FIFO_THRESHOLD);
#endif

    INFOS("configure done");
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*
 * Netstack API radio driver functions
 */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
esp_err_t cc1200_init(CC1200* device) {
    ESP_LOGD(TAG, "init called");
    INFO("testing INFO");
    ESP_LOGD(TAG, "^^^ tested INFO?");

    INFOS(CC1200_RF_CFG.cfg_descriptor);

    if (!(rf_flags & RF_INITIALIZED)) {
        /* initialize mutex for SPI access */
        if (device->mutex == NULL) {
            device->mutex = xSemaphoreCreateMutex();
            if (device->mutex != NULL) xSemaphoreGive(device->mutex);
        }

        /* Perform low level initialization */
        cc1200_arch_init(device);

        if (device->spi_semaphore == NULL) {
            device->spi_semaphore = xSemaphoreCreateMutex();
            xSemaphoreGive(device->spi_semaphore);
        }

        /* Configure GPIO interrupts */
        SETUP_GPIO_INTERRUPTS(device);

        /* Write initial configuration */
        driver_cc1200_configure(device);

        /* Enable address filtering + auto ack */
        // rx_mode_value = (RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_ADDRESS_FILTER);

        /* Enable CCA */
        // tx_mode_value = (RADIO_TX_MODE_SEND_ON_CCA);

        /* Set output power */
        driver_cc1200_set_txpower(device, CC1200_RF_CFG.max_txpower);

        /* Adjust CAA threshold */
        driver_cc1200_set_cca_threshold(device, CC1200_RF_CFG.cca_threshold);

        /* Setup queues */
        cc1200_tx_queue  = xQueueCreate(10, sizeof(cc1200_message));
        cc1200_rx_queue  = xQueueCreate(10, sizeof(cc1200_message));
        cc1200_cmd_queue = xQueueCreate(10, sizeof(cc1200_cmd));

        /* Setup RX thread */
        cc1200_event_group = xEventGroupCreate();
        xTaskCreate(&cc1200_event_task, "cc1200 events", 2048, device, 10, NULL);

        /* We are on + initialized at this point */
        rf_flags |= (RF_INITIALIZED | RF_ON);

        /* Set default channel. This will also force initial calibration! */
        driver_cc1200_set_frequency(868 * 1000 * 1000);

        /*
         * We have to call off() before on() because on() relies on the
         * configuration of the GPIO0 pin
         */
        cc1200_arch_gpio0_enable_irq(device);
    }

    return cc1200_troopers_init();
}

esp_err_t driver_cc1200_rx_packet(cc1200_message *msg) {
    msg->data = NULL;
    msg->len  = 0;
    if (xQueueReceive(cc1200_rx_queue, msg, 1000) == pdPASS)
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t driver_cc1200_tx_packet(uint8_t *data, uint8_t len) {
    ESP_LOGD(TAG, "transmitting packet...");
    xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_TX_FINISHED);

    cc1200_message msg;
    msg.len  = len;
    msg.data = malloc(msg.len + 1);
    memcpy(msg.data, data, msg.len);
    msg.data[msg.len] = 0x00;

    if (xQueueSend(cc1200_tx_queue, (void *) &msg, (TickType_t) 1) != pdPASS) {
        WARNING("Failed to queue received packet, dropping");
        free(msg.data);
        return ESP_FAIL;
    }

    xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_HANDLE_TX);
    EventBits_t event_bits = xEventGroupWaitBits(cc1200_event_group, CC1200_EVENT_TX_FINISHED, pdFALSE, pdFALSE, 1000 / portTICK_PERIOD_MS);

    if (event_bits & CC1200_EVENT_TX_FINISHED)
        return ESP_OK;
    else
        return ESP_FAIL;
}

uint8_t driver_cc1200_read_reg(CC1200* device, uint16_t addr) {
    uint8_t ret = DEVICE_BUSY;
//    if (xSemaphoreTake(device->spi_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        ret = single_read(device, addr);
//        xSemaphoreGive(device->spi_semaphore);
//    }
    return ret;
}

uint8_t driver_cc1200_write_reg(CC1200* device, uint16_t addr, uint8_t data) {
    uint8_t ret = DEVICE_BUSY;
//    if (xSemaphoreTake(device->spi_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        ret = single_write(device, addr, data);
//        xSemaphoreGive(device->spi_semaphore);
//    }
    return ret;
}

esp_err_t driver_cc1200_send_cmd(uint8_t cmd, uint32_t arg) {
    cc1200_cmd cc1200cmd;
    cc1200cmd.cmd = cmd;
    cc1200cmd.arg = arg;

    if (xQueueSend(cc1200_cmd_queue, (void *) &cc1200cmd, (TickType_t) 1) != pdPASS) {
        WARNING("Failed to queue received packet, dropping");
        return ESP_FAIL;
    }

    xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_CMD);
    EventBits_t event_bits = xEventGroupWaitBits(cc1200_event_group, CC1200_EVENT_CMD_DONE, pdFALSE, pdFALSE, 1000 / portTICK_PERIOD_MS);

    if (event_bits & CC1200_EVENT_CMD_DONE)
        return ESP_OK;
    else
        return ESP_FAIL;
}

int driver_cc1200_set_frequency(uint32_t freq) {
    if (freq < 863000000 || freq > 870000000) {
        return CHANNEL_OUT_OF_LIMITS;
    }

    return driver_cc1200_send_cmd(CMD_SET_FREQUENCY, freq);
}

uint32_t driver_cc1200_get_frequency(CC1200* device) {
    uint32_t freq = 0;
//    if (xSemaphoreTake(device->spi_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        ((uint8_t *) &freq)[0] = single_read(device, CC1200_FREQ0);
        ((uint8_t *) &freq)[1] = single_read(device, CC1200_FREQ1);
        ((uint8_t *) &freq)[2] = single_read(device, CC1200_FREQ2);

        freq = freq * FREQ_DIVIDER / FREQ_MULTIPLIER;
//    }

//    xSemaphoreGive(device->spi_semaphore);
    return freq;
}

int driver_cc1200_get_state(CC1200* device) { return single_read(device, CC1200_MARCSTATE) & 0x1f; }

esp_err_t driver_cc1200_on() { return driver_cc1200_send_cmd(CMD_ON, 0); }

esp_err_t driver_cc1200_off() { return driver_cc1200_send_cmd(CMD_OFF, 0); }
