#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "include/cc1200_troopers.h"
#include "include/driver_cc1200.h"

#define RADIO_TX_ERR              1
#define RADIO_TX_OK               0
#define RADIO_TX_MODE_SEND_ON_CCA 1
#define RADIO_TX_COLLISION        2

#define CMD_OK 0


static const char *TAG = "driver_cc1200";

QueueHandle_t      cc1200_tx_queue;
QueueHandle_t      cc1200_rx_queue;
QueueHandle_t      cc1200_cmd_queue;
EventGroupHandle_t cc1200_event_group;

/*---------------------------------------------------------------------------*/
/* Variables */
/*---------------------------------------------------------------------------*/
/*
 * The current channel in the range CC1200_RF_CHANNEL_MIN
 * to CC1200_RF_CHANNEL_MAX
 */
/* The radio drivers state */
uint8_t rf_flags = 0;
#if !CC1200_AUTOCAL && CC1200_CAL_TIMEOUT_SECONDS
/* Use a timeout to decide when to calibrate */
unsigned long cal_timer;
#endif

/* processes a command for the radio */
void cc1200_handle_cmd(CC1200* device, cc1200_cmd *cmd);
void cc1200_cmd_set_frequency(CC1200* device, cc1200_cmd *cmd);
/* handles the reception of a packet */
static void cc1200_handle_rx(CC1200* device);
/* Send the packet that has previously been prepared. */
static int cc1200_handle_tx(CC1200* device, cc1200_message *msg);
/*
 * Perform a Clear-Channel Assessment (CCA) to find out if there is
 * a packet in the air or not.
 */
static int channel_clear(CC1200* device);
/* Enter IDLE state. */
static void enter_idle_state(CC1200* device);
/* Enter RX state. */
static void idle_calibrate_rx(CC1200* device);
/* Restart RX from within RX interrupt. */
static void enter_rx_state(CC1200* device);
/* Fill TX FIFO, start TX and wait for TX to complete (blocking!). */
static int idle_tx_rx(CC1200* device, const uint8_t *payload, uint16_t payload_len);
/* Calculate FREQ register from channel */
static uint32_t calculate_freq(uint8_t channel);
/* Validate address and send ACK if requested. */
static int addr_check_auto_ack(uint8_t *frame, uint16_t frame_len);
/* Turn the radio on. */
static int cc1200_cmd_on(CC1200* device);
/* Turn the radio off. */
static int cc1200_cmd_off(CC1200* device);
/* Return the radio's state. */
static uint8_t state(CC1200* device);
/* Perform manual calibration. */
static void calibrate(CC1200* device);

/*---------------------------------------------------------------------------*/
/*
 * The CC1200 interrupt handler: called by the hardware interrupt
 * handler, which is defined as part of the cc1200-arch interface.
 */
int cc1200_rx_interrupt(void) {
    xEventGroupSetBitsFromISR(cc1200_event_group, CC1200_EVENT_HANDLE_RX, false);
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Handle tasks left over from rx interrupt or because SPI was locked */
void cc1200_event_task(void *arg) {
    CC1200* device = (CC1200*) arg;
    while (1) {
        EventBits_t event_bits = xEventGroupWaitBits(
            cc1200_event_group, CC1200_EVENT_CMD | CC1200_EVENT_HANDLE_RX | CC1200_EVENT_RX_DONE | CC1200_EVENT_HANDLE_TX | CC1200_EVENT_TX_DONE, pdFALSE,
            pdFALSE, 1000 / portTICK_PERIOD_MS);
        if (xSemaphoreTake(device->mutex, 1000 / portTICK_PERIOD_MS) != pdTRUE) continue;

        if (event_bits & CC1200_EVENT_HANDLE_RX) {
            INFOS("CC1200_EVENT_HANDLE_RX");
            xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_HANDLE_RX);
            cc1200_handle_rx(device);
        }

        if (event_bits & CC1200_EVENT_RX_DONE) {
            INFOS("CC1200_EVENT_RX_DONE");
            enter_rx_state(device);
            xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
        }

        if (event_bits & CC1200_EVENT_CMD) {
            INFOS("CC1200_EVENT_HANDLE_CMD");
            cc1200_cmd cmd;
            if (xQueueReceive(cc1200_cmd_queue, &cmd, 0) == pdPASS) {
                cc1200_handle_cmd(device, &cmd);
                if (!uxQueueMessagesWaiting(cc1200_cmd_queue)) {
                    xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_CMD);
                    xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_CMD_DONE);
                }
            } else
                xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_CMD);
        }

        if (event_bits & CC1200_EVENT_HANDLE_TX) {
            INFOS("CC1200_EVENT_HANDLE_TX");
            INFOD(uxQueueMessagesWaiting(cc1200_tx_queue));
            if (uxQueueMessagesWaiting(cc1200_tx_queue)) {
                cc1200_message msg;
                if (xQueueReceive(cc1200_tx_queue, &msg, 0) == pdPASS) {
                    int r = cc1200_handle_tx(device, &msg);
                    if (r == RADIO_TX_OK)
                        ESP_LOGD(TAG, "transmitted ok");
                    else
                        ESP_LOGE(TAG, "transmission failed with error code %d", r);

                    free(msg.data);
                    xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_TX_DONE);
                }
            } else
                xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_TX_FINISHED);

            xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_HANDLE_TX);
        }

        if (event_bits & CC1200_EVENT_TX_DONE) {
            INFOS("CC1200_EVENT_TX_DONE");
            xEventGroupClearBits(cc1200_event_group, CC1200_EVENT_TX_DONE);
            INFOD(uxQueueMessagesWaiting(cc1200_tx_queue));
            if (uxQueueMessagesWaiting(cc1200_tx_queue))
                xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_HANDLE_TX);
            else
                xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_TX_FINISHED);
        }

        xSemaphoreGive(device->mutex);
    }
}

void cc1200_handle_cmd(CC1200* device, cc1200_cmd *cmd) {
    if (cmd->cmd == CMD_SET_FREQUENCY)
        cc1200_cmd_set_frequency(device, cmd);
    else if (cmd->cmd == CMD_ON)
        cc1200_cmd_on(device);
    else if (cmd->cmd == CMD_OFF)
        cc1200_cmd_off(device);
}

/*---------------------------------------------------------------------------*/
/* Send the packet that has previously been prepared. */
static int cc1200_handle_tx(CC1200* device, cc1200_message *msg) {
    uint8_t was_off = 0;
    int     ret     = RADIO_TX_OK;

    if ((msg->len < ACK_LEN) || (msg->len > CC1200_MAX_PAYLOAD_LEN)) {
        ERROR("RF: Invalid payload length!");
        return RADIO_TX_ERR;
    }

    /* TX ongoing. Inhibit channel update & ACK as soon as possible */

    if (!(rf_flags & RF_ON)) {
        /* Radio is off - turn it on */
        was_off = 1;
        cc1200_cmd_on(device);
        /* Radio is in RX now (and calibrated...) */
    }

    /* Perform clear channel assessment */
    if (!channel_clear(device)) {
        /* Channel occupied */
        if (was_off) {
            cc1200_cmd_off(device);
        }
        return RADIO_TX_COLLISION;
    }

    /*
     * Make sure we start from a sane state. enter_idle_state(device) also disables
     * the GPIO interrupt(s).
     */
    enter_idle_state(device);

#if !CC1200_AUTOCAL
    /* Perform manual calibration unless just turned on */
    if (!was_off) {
        calibrate(device);
    }
#endif

    INFOX(state(device));
    /* Send data using TX FIFO */
    if (idle_tx_rx(device, (const uint8_t *) msg->data, msg->len) == RADIO_TX_OK) {
        /*
         * TXOFF_MODE is set to RX,
         * let's wait until we are in RX and turn on the GPIO IRQs
         * again as they were turned off in enter_idle_state(device)
         */

        INFOX(state(device));
        BUSYWAIT_UNTIL_STATE(device, STATE_RX, CC1200_RF_CFG.tx_rx_turnaround);
        INFOX(state(device));

        ENABLE_GPIO_INTERRUPTS(device);

    } else {
        /*
         * Something went wrong during TX, idle_tx_rx() returns in IDLE
         * state in this case.
         * Turn on RX again unless we turn off anyway
         */

        ret = RADIO_TX_ERR;
        if (!was_off) {
#ifdef RF_FORCE_CALIBRATION
            rf_flags |= RF_FORCE_CALIBRATION;
#endif
            idle_calibrate_rx(device);
        }
    }

    if (was_off) {
        cc1200_cmd_off(device);
    }

    /* TX completed */

    return ret;
}
/*---------------------------------------------------------------------------*/
/*
 * Perform a Clear-Channel Assessment (CCA) to find out if there is a
 * packet in the air or not.
 */
static int channel_clear(CC1200* device) {
    ESP_LOGW(TAG, "channel_clear() disabled for now");
    return 1; /* TODO: fix this */
    uint8_t cca, was_off = 0;

    if (!(rf_flags & RF_ON)) {
        /* We are off */
        was_off = 1;
        cc1200_cmd_on(device);
    }

    RF_ASSERT(state(device) == STATE_RX);

    /*
     * At this point we should be in RX. If GPIO0 is set, we are currently
     * receiving a packet, no need to check the RSSI. Or is there any situation
     * when we want access the channel even if we are currently receiving a
     * packet???
     */

    if (cc1200_arch_gpio0_read_pin(device) == 1) {
        /* Channel occupied */
        INFO("RF: CCA (0)");
        cca = 0;
    } else {
        uint8_t rssi0;

        /* Wait for CARRIER_SENSE_VALID signal */
        BUSYWAIT_UNTIL(((rssi0 = single_read(device, CC1200_RSSI0)) & CC1200_CARRIER_SENSE_VALID), RTIMER_SECOND / 100);
        RF_ASSERT(rssi0 & CC1200_CARRIER_SENSE_VALID);

        if (rssi0 & CC1200_CARRIER_SENSE) {
            /* Channel occupied */
            INFO("RF: CCA (0)");
            cca = 0;
        } else {
            /* Channel clear */
            INFO("RF: CCA (1)");
            cca = 1;
        }
    }

    if (was_off) {
        cc1200_cmd_off(device);
    }

    return cca;
}

/*---------------------------------------------------------------------------*/
/* Enter IDLE state. */
static void enter_idle_state(CC1200* device) {
    uint8_t s;

    DISABLE_GPIO_INTERRUPTS(device);

    TX_LEDS_OFF();
    RX_LEDS_OFF();

    s = state(device);

    if (s == STATE_IDLE) {
        return;
    } else if (s == STATE_RX_FIFO_ERR) {
        WARNING("RF: RX FIFO error!");
        strobe(device, CC1200_SFRX);
    } else if (s == STATE_TX_FIFO_ERR) {
        WARNING("RF: TX FIFO error!");
        strobe(device, CC1200_SFTX);
    }

    strobe(device, CC1200_SIDLE);
    BUSYWAIT_UNTIL_STATE(device, STATE_IDLE, RTIMER_SECOND / 100);

} /* enter_idle_state(device), 21.05.2015 */
/*---------------------------------------------------------------------------*/
/* Enter RX state. */

/* Perform manual calibration. */
static void calibrate(CC1200* device) {
    INFOS("Calibrate");

    strobe(device, CC1200_SCAL);
    BUSYWAIT_UNTIL_STATE(device, STATE_CALIBRATE, RTIMER_SECOND / 100);
}

void idle_calibrate_rx(CC1200* device) {
    RF_ASSERT(state(device) == STATE_IDLE);

#if !CC1200_AUTOCAL
    calibrate(device);
#endif

    strobe(device, CC1200_SFRX);
    strobe(device, CC1200_SRX);
    BUSYWAIT_UNTIL_STATE(device, STATE_RX, RTIMER_SECOND / 100);

    ENABLE_GPIO_INTERRUPTS(device);
}
/*---------------------------------------------------------------------------*/
/* Restart RX from within RX interrupt. */
static void enter_rx_state(CC1200* device) {
    uint8_t s = state(device);

    if (s == STATE_IDLE) {
        /* Proceed to rx */
    } else if (s == STATE_RX_FIFO_ERR) {
        WARNING("RF: RX FIFO error!");
        strobe(device, CC1200_SFRX);
    } else if (s == STATE_TX_FIFO_ERR) {
        WARNING("RF: TX FIFO error!");
        strobe(device, CC1200_SFTX);
    } else {
        strobe(device, CC1200_SIDLE);
        BUSYWAIT_UNTIL_STATE(device, STATE_IDLE, RTIMER_SECOND / 100);
    }

    RX_LEDS_OFF();

    /* Clear pending GPIO interrupts */
    ENABLE_GPIO_INTERRUPTS(device);

    strobe(device, CC1200_SFRX);
    strobe(device, CC1200_SRX);
    BUSYWAIT_UNTIL_STATE(device, STATE_RX, RTIMER_SECOND / 100);
}
/*---------------------------------------------------------------------------*/
/* Fill TX FIFO, start TX and wait for TX to complete (blocking!). */
static int idle_tx_rx(CC1200* device, const uint8_t *payload, uint16_t payload_len) {
#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
    uint16_t       bytes_left_to_write;
    uint8_t        to_write;
    const uint8_t *p;
#endif

#if CC1200_802154G
    /* Prepare PHR for 802.15.4g frames */
    struct {
        uint8_t phra;
        uint8_t phrb;
    } phr;
#if CC1200_802154G_CRC16
    payload_len += 2;
#else
    payload_len += 4;
#endif
    /* Frame length */
    phr.phrb = (uint8_t) (payload_len & 0x00FF);
    phr.phra = (uint8_t) ((payload_len >> 8) & 0x0007);
#if CC1200_802154G_WHITENING
    /* Enable Whitening */
    phr.phra |= (1 << 3);
#endif /* #if CC1200_802154G_WHITENING */
#if CC1200_802154G_CRC16
    /* FCS type 1, 2 Byte CRC */
    phr.phra |= (1 << 4);
#endif /* #if CC1200_802154G_CRC16 */
#endif /* #if CC1200_802154G */

    /* Prepare for RX */
    strobe(device, CC1200_SFRX);

    /* Flush TX FIFO */
    strobe(device, CC1200_SFTX);

#if USE_SFSTXON
    /*
     * Enable synthesizer. Saves us a few µs especially if it takes
     * long enough to fill the FIFO. This strobe must not be
     * send before SFTX!
     */
    strobe(device, CC1200_SFSTXON);
#endif

#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
    /*
     * We already checked that GPIO2 is used if
     * CC1200_MAX_PAYLOAD_LEN > 127 / 126 in the header of this file
     */
    single_write(device, CC1200_IOCFG2, CC1200_IOCFG_TXFIFO_THR);
#endif

#if CC1200_802154G
    /* Write PHR */
    burst_write(device, CC1200_TXFIFO, (uint8_t *) &phr, PHR_LEN);
#else
    /* Write length byte */
    burst_write(device, CC1200_TXFIFO, (uint8_t *) &payload_len, PHR_LEN);
#endif /* #if CC1200_802154G */

       /*
        * Fill FIFO with data. If SPI is slow it might make sense
        * to divide this process into several chunks.
        * The best solution would be to perform TX FIFO refill
        * using an interrupt, but we are blocking here (= in TX) anyway...
        */

#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
    to_write = MIN(payload_len, (CC1200_FIFO_SIZE - PHR_LEN));
    burst_write(device, CC1200_TXFIFO, payload, to_write);
    bytes_left_to_write = payload_len - to_write;
    p                   = payload + to_write;
#else
    burst_write(device, CC1200_TXFIFO, payload, payload_len);
#endif

#if USE_SFSTXON
    /* Wait for synthesizer to be ready */
    BUSYWAIT_UNTIL_STATE(device, STATE_FSTXON, RTIMER_SECOND / 100);
#endif

    /* Start TX */
    strobe(device, CC1200_STX);
    INFOX(state(device));

    /* Wait for TX to start. */
    BUSYWAIT_UNTIL((state(device) != STATE_TX), RTIMER_SECOND / 10);
    INFOX(state(device))

    /* Turned off at the latest in enter_idle_state(device) */
    TX_LEDS_ON();

    if ((cc1200_arch_gpio0_read_pin(device) == 0) && (single_read(device, CC1200_NUM_TXBYTES) != 0)) {
        /*
         * TX didn't start in time. We also check NUM_TXBYES
         * in case we missed the rising edge of the GPIO signal
         */

        ERROR("RF: TX doesn't start!");
#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
        single_write(device, CC1200_IOCFG2, GPIO2_IOCFG);
#endif
        enter_idle_state(device);

        return RADIO_TX_ERR;
    }

#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
    if (bytes_left_to_write != 0) {
        rtimer_clock_t t0;
        uint8_t        s;
        t0 = RTIMER_NOW();
        do {
            if ((bytes_left_to_write != 0) && (cc1200_arch_gpio2_read_pin() == 0)) {
                /* TX TIFO is drained below FIFO_THRESHOLD. Re-fill... */
                to_write = MIN(bytes_left_to_write, FIFO_THRESHOLD);
                burst_write(device, CC1200_TXFIFO, p, to_write);
                bytes_left_to_write -= to_write;
                p += to_write;
                t0 += CC1200_RF_CFG.tx_pkt_lifetime;
            }
        } while ((cc1200_arch_gpio0_read_pin(device) == 1) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CC1200_RF_CFG.tx_pkt_lifetime));

        /*
         * At this point we either left TX or a timeout occurred. If all went
         * well, we are in RX (or at least settling) now.
         * If we didn't manage to refill the TX FIFO, an underflow might
         * have occur-ed - the radio might be still in TX here!
         */

        s = state(device);
        if ((s != STATE_RX) && (s != STATE_SETTLING)) {
            /*
             * Something bad happened. Wait for radio to enter a
             * stable state (in case of an error we are in TX here)
             */

            INFOS("TX failure!");
            BUSYWAIT_UNTIL((state(device) != STATE_TX), RTIMER_SECOND / 100);
            /* Re-configure GPIO2 */
            single_write(device, CC1200_IOCFG2, GPIO2_IOCFG);
            enter_idle_state(device);

            return RADIO_TX_ERR;
        }

    } else {
        /* Wait for TX to complete */
        BUSYWAIT_UNTIL((cc1200_arch_gpio0_read_pin() == 0), CC1200_RF_CFG.tx_pkt_lifetime);
    }
#else
    /* Wait for TX to complete */
    BUSYWAIT_UNTIL((state(device) != STATE_TX), CC1200_RF_CFG.tx_pkt_lifetime);
#endif

    if (state(device) == STATE_TX) {
        /* TX takes to long - abort */
        ERROR("RF: TX takes to long!");
#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
        /* Re-configure GPIO2 */
        single_write(device, CC1200_IOCFG2, GPIO2_IOCFG);
#endif
        enter_idle_state(device);

        return RADIO_TX_ERR;
    }

#if (CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN))
    /* Re-configure GPIO2 */
    single_write(device, CC1200_IOCFG2, GPIO2_IOCFG);
#endif

    TX_LEDS_OFF();

    return RADIO_TX_OK;
}

/*---------------------------------------------------------------------------*/
/* Check broadcast address. */
static int is_broadcast_addr(uint8_t mode, uint8_t *addr) {
    /* int i = mode == FRAME802154_SHORTADDRMODE ? 2 : 8; */

    /* while(i-- > 0) { */
    /*   if(addr[i] != 0xff) { */
    /*     return 0; */
    /*   } */
    /* } */

    /* return 1; */
    return -1;
}
/*---------------------------------------------------------------------------*/
static int addr_check_auto_ack(uint8_t *frame, uint16_t frame_len) {
    /*   frame802154_t info154; */

    /*   if(frame802154_parse(frame, frame_len, &info154) != 0) { */

    /*     /\* We received a valid 802.15.4 frame *\/ */

    /*     if(!(rx_mode_value & RADIO_RX_MODE_ADDRESS_FILTER) || */
    /*        info154.fcf.frame_type == FRAME802154_ACKFRAME || */
    /*        is_broadcast_addr(info154.fcf.dest_addr_mode, */
    /*                          (uint8_t *)&info154.dest_addr) || */
    /*        linkaddr_cmp((linkaddr_t *)&info154.dest_addr, */
    /*                     &linkaddr_node_addr)) { */

    /*       /\* */
    /*        * Address check succeeded or address filter disabled. */
    /*        * We send an ACK in case a corresponding data frame */
    /*        * is received even in promiscuous mode (if auto-ack is */
    /*        * enabled)! */
    /*        *\/ */

    /*       if((rx_mode_value & RADIO_RX_MODE_AUTOACK) && */
    /*          info154.fcf.frame_type == FRAME802154_DATAFRAME && */
    /*          info154.fcf.ack_required != 0 && */
    /*          (!(rx_mode_value & RADIO_RX_MODE_ADDRESS_FILTER) || */
    /*           linkaddr_cmp((linkaddr_t *)&info154.dest_addr, */
    /*                        &linkaddr_node_addr))) { */

    /*         /\* */
    /*          * Data frame destined for us & ACK request bit set -> send ACK. */
    /*          * Make sure the preamble length is configured accordingly as */
    /*          * MAC timing parameters rely on this! */
    /*          *\/ */

    /*         uint8_t ack[ACK_LEN] = { FRAME802154_ACKFRAME, 0, info154.seq }; */

    /* #if (RXOFF_MODE_RX == 1) */
    /*         /\* */
    /*          * This turns off GPIOx interrupts. Make sure they are turned on */
    /*          * in enter_rx_state(device) later on! */
    /*          *\/ */
    /*         enter_idle_state(device); */
    /* #endif */

    /*         idle_tx_rx((const uint8_t *)ack, ACK_LEN); */

    /*         /\* enter_rx_state(device) will follow *\/ */

    /*         return ADDR_CHECK_OK_ACK_SEND; */

    /*       } */

    /*       return ADDR_CHECK_OK; */

    /*     } else { */

    /*       return ADDR_CHECK_FAILED; */

    /*     } */

    /*   } */

    /*   return INVALID_FRAME; */
    return -1;
}

static void cc1200_handle_rx(CC1200* device) {
    /* The radio's state */
    uint8_t s;
    /* The number of bytes in the RX FIFO waiting for read-out */
    uint8_t num_rxbytes;
    /* The payload length read as the first byte from the RX FIFO */
    uint16_t payload_len;

    /*
     * If CC1200_USE_GPIO2 is enabled, we come here either once RX FIFO
     * threshold is reached (GPIO2 rising edge)
     * or at the end of the packet (GPIO0 falling edge).
     */

    /* Make sure we are in a sane state. Sane means: either RX or IDLE */
    s = state(device);
    if ((s == STATE_RX_FIFO_ERR) || (s == STATE_TX_FIFO_ERR)) {
        ERROR("FIFO error");
        xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
        return;
    }

    num_rxbytes = single_read(device, CC1200_NUM_RXBYTES);

    if (num_rxbytes == 0) {
        /*
         * This might happen from time to time because
         * this function is also called by the pollhandler and / or
         * from TWO interrupts which can occur at the same time.
         */

        INFOS("RX FIFO empty!");
        return;
    }

#if CC1200_802154G
    struct {
        uint8_t phra;
        uint8_t phrb;
    } phr;

    if (num_rxbytes < PHR_LEN) {
        WARNING("RF: PHR incomplete!");
        xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
        return;
    }

    burst_read(device, CC1200_RXFIFO, &phr, PHR_LEN);
    payload_len = (phr.phra & 0x07);
    payload_len <<= 8;
    payload_len += phr.phrb;

    if (phr.phra & (1 << 4)) {
        /* CRC16, payload_len += 2 */
        payload_len -= 2;
    } else {
        /* CRC16, payload_len += 4 */
        payload_len -= 4;
    }
#else
    /* Read first byte in RX FIFO (payload length) */
    burst_read(device, CC1200_RXFIFO, (uint8_t *) &payload_len, PHR_LEN);
#endif

    if (payload_len < ACK_LEN) {
        /* Packet to short. Discard it */
        WARNING("RF: Packet too short!");
        xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
        return;
    }

    if (payload_len > CC1200_MAX_PAYLOAD_LEN) {
        /* Packet to long. Discard it */
        WARNING("RF: Packet to long!");
        xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
        return;
    }

    RX_LEDS_ON();
    num_rxbytes -= PHR_LEN;
    /*
     * Read out remaining bytes unless FIFO is empty.
     * We have at least num_rxbytes in the FIFO to be read out.
     */

    if (num_rxbytes == (payload_len + CC_APPENDIX_LEN)) {
        /*
         * End of packet. Read appendix (if available), check CRC
         * and copy the data from temporary buffer to rx_pkt
         * RSSI offset already set using AGC_GAIN_ADJUST.GAIN_ADJUSTMENT
         */

        // int ret = addr_check_auto_ack(buf, PHR_LEN);
        cc1200_message msg;
        msg.len  = payload_len;
        msg.data = malloc(num_rxbytes + 1);
        burst_read(device, CC1200_RXFIFO, msg.data, num_rxbytes);
        msg.data[msg.len] = 0x00;
#if APPEND_STATUS
        if (msg.len >= 2) {
            msg.crc_lqi = msg.data[num_rxbytes - 1];
            msg.rssi    = msg.data[num_rxbytes - 1];
        } else {
            msg.crc_lqi = 0x00;
            msg.rssi    = 0x00;
            WARNING("Not enough bytes for rssi and crc_lqi");
        }
#else
        msg.crc_lqi = single_read(device, CC1200_LQI_VAL);
        msg.rssi    = single_read(device, CC1200_RSSI1);
        ;
#endif

        if (!(msg.crc_lqi & (1 << 7))) {
            /* CRC error. Drop the packet */
            WARNING("CRC error!");
            free(msg.data);
            xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
            return;
        }

        if (!cc1200_troopers_cb(&msg)) {
            WARNING("Received message %s", msg.data);

            if (xQueueSend(cc1200_rx_queue, (void *) &msg, (TickType_t) 1) != pdPASS) {
                WARNING("Failed to queue received packet, dropping");
                free(msg.data);
            }
        }
    } else { /* num_rxbytes != (payload_len + CC_APPENDIX_LEN) */
        /*
         * We have a mismatch between the number of bytes in the RX FIFO
         * and the payload_len. This would lead to an buffer overflow,
         * so we catch this error here.
         */

        WARNING("RF: RX length mismatch %d %d!", num_rxbytes, payload_len);
    }

    xEventGroupSetBits(cc1200_event_group, CC1200_EVENT_RX_DONE);
    return;
}

/*---------------------------------------------------------------------------*/
/* Turn the radio on. */
static int cc1200_cmd_on(CC1200* device) {
    INFO("RF: On");

    /* Don't turn on if we are on already */
    if (!(rf_flags & RF_ON)) {
        /* Radio is IDLE now, re-configure GPIO0 (modified inside cc1200_cmd_off()) */
        driver_cc1200_configure(device);

        /* Wake-up procedure. Wait for GPIO0 to de-assert (CHIP_RDYn) */
        BUSYWAIT_UNTIL((cc1200_arch_gpio0_read_pin(device) == 0), RTIMER_SECOND / 100);
        RF_ASSERT((cc1200_arch_gpio0_read_pin(device) == 0));

        rf_flags |= RF_ON;

        /* Turn on RX */
        enter_idle_state(device);
        idle_calibrate_rx(device);
        enter_rx_state(device);

    } else {
        INFO("RF: Already on");
    }

    return 1;
}
/*---------------------------------------------------------------------------*/
/* Turn the radio off. */
static int cc1200_cmd_off(CC1200* device) {
    INFO("RF: Off");

    /* Don't turn off if we are off already */
    if (rf_flags & RF_ON) {
        enter_idle_state(device);

        /*
         * As we use GPIO as CHIP_RDYn signal on wake-up / cc1200_cmd_on(),
         * we re-configure it for CHIP_RDYn.
         */
        single_write(device, CC1200_IOCFG0, CC1200_IOCFG_RXFIFO_CHIP_RDY_N);

        /* Say goodbye ... */
        strobe(device, CC1200_SPWD);

        /* Clear all but the initialized flag */
        rf_flags = RF_INITIALIZED;

#if CC1200_USE_RX_WATCHDOG
        etimer_stop(&et);
#endif /* #if CC1200_USE_RX_WATCHDOG */

    } else {
        INFO("RF: Already off");
    }

    return 1;
}

/* Return the radio's state. */
static uint8_t state(CC1200* device) {
#if STATE_USES_MARC_STATE
    return single_read(device, CC1200_MARCSTATE) & 0x1f;
#else
    return strobe(device, CC1200_SNOP) & 0x70;
#endif
}

/*---------------------------------------------------------------------------*/
/* Update TX power */
void driver_cc1200_set_txpower(CC1200* device, int8_t txpower_dbm) {
//    if (xSemaphoreTake(device->spi_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        uint8_t reg = single_read(device, CC1200_PA_CFG1);

        reg &= ~0x3F;
        /* Up to now we don't handle the special power levels PA_POWER_RAMP < 3 */
        reg |= ((((txpower_dbm + 18) * 2) - 1) & 0x3F);
        single_write(device, CC1200_PA_CFG1, reg);

//        xSemaphoreGive(device->spi_semaphore);
//    }
}

/*---------------------------------------------------------------------------*/
/* Update CCA threshold */
void driver_cc1200_set_cca_threshold(CC1200* device, int8_t threshold_dbm) {
//    if (xSemaphoreTake(device->spi_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        single_write(device, CC1200_AGC_CS_THR, (uint8_t) threshold_dbm);
//        xSemaphoreGive(device->spi_semaphore);
//    }
}

void cc1200_cmd_set_frequency(CC1200* device, cc1200_cmd *cmd) {
    uint32_t freq    = cmd->arg;
    uint8_t  was_off = 0;

    INFO("Frequency update (%.3f)", freq / 1e6);

    if (!(rf_flags & RF_ON)) {
        was_off = 1;
//         cc1200_cmd_on(device);
    }

    enter_idle_state(device);

    freq = (freq / 1000) * FREQ_MULTIPLIER / FREQ_DIVIDER;

    single_write(device, CC1200_FREQ0, ((uint8_t *) &freq)[0]);
    single_write(device, CC1200_FREQ1, ((uint8_t *) &freq)[1]);
    single_write(device, CC1200_FREQ2, ((uint8_t *) &freq)[2]);

    strobe(device, CC1200_SCAL);
    BUSYWAIT_UNTIL_STATE(device, STATE_CALIBRATE, RTIMER_SECOND / 100);
    BUSYWAIT_UNTIL(state(device) != STATE_CALIBRATE, RTIMER_SECOND / 100);

    enter_rx_state(device);
}
