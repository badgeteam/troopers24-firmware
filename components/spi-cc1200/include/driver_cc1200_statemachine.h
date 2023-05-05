#ifndef DRIVER_CC1200_STATEMACHINE_H
#define DRIVER_CC1200_STATEMACHINE_H

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

extern QueueHandle_t      cc1200_tx_queue;
extern QueueHandle_t      cc1200_rx_queue;
extern QueueHandle_t      cc1200_cmd_queue;
extern EventGroupHandle_t cc1200_event_group;
#define CC1200_EVENT_HANDLE_RX   (1 << 0) /* packet reception */
#define CC1200_EVENT_RX_DONE     (1 << 1) /*packet reception finished */
#define CC1200_EVENT_HANDLE_TX   (1 << 2) /*request packet to be transmitted */
#define CC1200_EVENT_TX_DONE     (1 << 3) /* a packet transmission has finished */
#define CC1200_EVENT_TX_FINISHED (1 << 4) /* tx queue is empty and tx has finished */

#define CC1200_EVENT_CMD      (1 << 5)
#define CC1200_EVENT_CMD_DONE (1 << 6)

typedef struct cc1200_cmd {
    uint8_t  cmd;
    uint32_t arg;
} cc1200_cmd;
#define CMD_SET_FREQUENCY (1 << 0)
#define CMD_ON            (1 << 1)
#define CMD_OFF           (1 << 2)

/*
 * Set this parameter to 1 in order to use the MARC_STATE register when
 * polling the chips's status. Else use the status byte returned when sending
 * a NOP strobe.
 *
 * TODO: Option to be removed upon approval of the driver
 */
#define STATE_USES_MARC_STATE 0
/*
 * Set this parameter to 1 in order to speed up transmission by
 * sending a FSTXON strobe before filling the FIFO.
 *
 * TODO: Option to be removed upon approval of the driver
 */
#define USE_SFSTXON 1
/*---------------------------------------------------------------------------*/
/* Phy header length */
#if CC1200_802154G
/* Phy header = 2 byte */
#define PHR_LEN 2
#else
/* Phy header = length byte = 1 byte */
#define PHR_LEN 1
#endif /* #if CC1200_802154G */
/*---------------------------------------------------------------------------*/
/* Size of appendix (rssi + lqi) appended to the rx pkt */
#define APPENDIX_LEN 2
/*---------------------------------------------------------------------------*/
/* Verify payload length */
/*---------------------------------------------------------------------------*/
#if CC1200_802154G
#if CC1200_USE_GPIO2
#if CC1200_MAX_PAYLOAD_LEN > (2048 - PHR_LEN)
#error Payload length not supported by this driver
#endif
#else
#if CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN)
/* PHR_LEN = 2 -> we can only place 126 payload bytes bytes in the FIFO */
#error Payload length not supported without GPIO2
#endif
#endif /* #if CC1200_USE_GPIO2 */
#else  /* #if CC1200_802154G */
#if CC1200_MAX_PAYLOAD_LEN > (CC1200_FIFO_SIZE - PHR_LEN)
/* PHR_LEN = 1 -> we can only place 127 payload bytes bytes in the FIFO */
#error Payload length not supported without enabling 802.15.4g mode
#endif
#endif /* #if CC1200_802154G */
/*---------------------------------------------------------------------------*/
/* Main driver configurations settings. Don't touch! */
/*---------------------------------------------------------------------------*/
#if CC1200_USE_GPIO2
/* Use GPIO2 as RX / TX FIFO threshold indicator pin */
#define GPIO2_IOCFG CC1200_IOCFG_RXFIFO_THR
/* This is the FIFO threshold we use */
#define FIFO_THRESHOLD 32
/* Turn on RX after packet reception */
#define RXOFF_MODE_RX 1
/* Let the CC1200 append RSSI + LQI */
#define APPEND_STATUS 1
#else
/* Arbitrary configuration for GPIO2 */
#define GPIO2_IOCFG CC1200_IOCFG_MARC_2PIN_STATUS_0
#if (CC1200_MAX_PAYLOAD_LEN <= (CC1200_FIFO_SIZE - PHR_LEN - APPENDIX_LEN))
/*
 * Read out RX FIFO at the end of the packet (GPIO0 falling edge). RX restarts
 * automatically
 */
#define RXOFF_MODE_RX 1
/* Let the CC1200 append RSSI + LQI */
#define APPEND_STATUS 1
#else
/*
 * Read out RX FIFO at the end of the packet (GPIO0 falling edge). RX has
 * to be started manually in this case
 */
#define RXOFF_MODE_RX 0
/* No space for appendix in the RX FIFO. Read it out by hand */
#define APPEND_STATUS 0
#endif /* #if CC1200_MAX_PAYLOAD_LEN <= 125 */
#endif /* #if CC1200_USE_GPIO2 */

/* Read out packet on falling edge of GPIO0 */
#define GPIO0_IOCFG CC1200_IOCFG_PKT_SYNC_RXTX
/* Arbitrary configuration for GPIO3 */
#define GPIO3_IOCFG CC1200_IOCFG_MARC_2PIN_STATUS_0
/* Turn on RX automatically after TX */
#define TXOFF_MODE_RX 1
#if APPEND_STATUS
/* CC1200 places two bytes in the RX FIFO */
#define CC_APPENDIX_LEN 2
#else
/* CC1200 doesn't add appendix to RX FIFO */
#define CC_APPENDIX_LEN 0
#endif /* #if APPEND_STATUS */
/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */
extern const cc1200_rf_cfg_t CC1200_RF_CFG;
/*---------------------------------------------------------------------------*/
/* This defines the way we calculate the frequency registers */
/*---------------------------------------------------------------------------*/
/* XTAL frequency in kHz */
#define XTAL_FREQ_KHZ 40000
/*
 * Divider + multiplier for calculation of FREQ registers
 * f * 2^16 * 4 / 40000 = f * 2^12 / 625 (no overflow up to frequencies of
 * 1048.576 MHz using uint32_t)
 */
#define LO_DIVIDER 4
#if (XTAL_FREQ_KHZ == 40000) && (LO_DIVIDER == 4)
#define FREQ_DIVIDER    625
#define FREQ_MULTIPLIER 4096
#else
#error Invalid settings for frequency calculation
#endif

/*---------------------------------------------------------------------------*/
/* Various flags indicating the operating state of the radio. See rf_flags */
/*---------------------------------------------------------------------------*/
/* Radio was initialized (= init() was called) */
#define RF_INITIALIZED 0x01
/* The radio is on (= not in standby) */
#define RF_ON 0x02
/* An incoming packet was detected (at least payload length was received */
#define RF_RX_PROCESSING_PKT 0x04
/* TX is ongoing */
#define RF_TX_ACTIVE 0x08
/* Channel update required */
#define RF_UPDATE_CHANNEL 0x10
/* Force calibration in case we don't use CC1200 AUTOCAL + timeout */
#if !CC1200_AUTOCAL
#if CC1200_CAL_TIMEOUT_SECONDS
#define RF_FORCE_CALIBRATION 0x40
#endif
#endif

/* Return value for mutx busy */
#define DEVICE_BUSY 42
/*---------------------------------------------------------------------------*/
/* Return values for addr_check_auto_ack() */
/*---------------------------------------------------------------------------*/
/* Frame cannot be parsed / is to short */
#define INVALID_FRAME 0
/* Address check failed */
#define ADDR_CHECK_FAILED 1
/* Address check succeeded */
#define ADDR_CHECK_OK 2
/* Address check succeeded and ACK was send */
#define ADDR_CHECK_OK_ACK_SEND 3
/*---------------------------------------------------------------------------*/
/* Return values for set_channel() */
/*---------------------------------------------------------------------------*/
/* Channel update was performed */
#define CHANNEL_UPDATE_SUCCEEDED 0
/* Busy, channel update postponed */
#define CHANNEL_UPDATE_POSTPONED 1
/* Invalid channel */
#define CHANNEL_OUT_OF_LIMITS 2
/*---------------------------------------------------------------------------*/
/* Length of 802.15.4 ACK. We discard packets with a smaller size */
#if CC1200_802154G
#define ACK_LEN 3
#else
#define ACK_LEN 1
#endif

/*---------------------------------------------------------------------------*/
#if STATE_USES_MARC_STATE
/* We use the MARC_STATE register to poll the chip's status */
#define STATE_IDLE          CC1200_MARC_STATE_IDLE
#define STATE_RX            CC1200_MARC_STATE_RX
#define STATE_TX            CC1200_MARC_STATE_TX
#define STATE_RX_FIFO_ERROR CC1200_MARC_STATE_RX_FIFO_ERR
#define STATE_TX_FIFO_ERROR CC1200_MARC_STATE_TX_FIFO_ERR
#else
/* We use the status byte read out using a NOP strobe */
#define STATE_IDLE        CC1200_STATUS_BYTE_IDLE
#define STATE_RX          CC1200_STATUS_BYTE_RX
#define STATE_TX          CC1200_STATUS_BYTE_TX
#define STATE_FSTXON      CC1200_STATUS_BYTE_FSTXON
#define STATE_CALIBRATE   CC1200_STATUS_BYTE_CALIBRATE
#define STATE_SETTLING    CC1200_STATUS_BYTE_SETTLING
#define STATE_RX_FIFO_ERR CC1200_STATUS_BYTE_RX_FIFO_ERR
#define STATE_TX_FIFO_ERR CC1200_STATUS_BYTE_TX_FIFO_ERR
#endif /* #if STATE_USES_MARC_STATE */

extern uint8_t rf_flags;
#if !CC1200_AUTOCAL && CC1200_CAL_TIMEOUT_SECONDS
/* Use a timeout to decide when to calibrate */
extern unsigned long cal_timer;
#endif

void cc1200_event_task(void *arg);

#endif
