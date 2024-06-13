#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//Probably unneeded
////#include "spi.h"
////#include "timer.h"
////#include "main.h"
////#include "logger.h"
////
////#define ST25R_SS_PIN            SPI1_CS_Pin         /*!< GPIO pin used for ST25R SPI SS                */
////#define ST25R_SS_PORT           SPI1_CS_GPIO_Port   /*!< GPIO port used for ST25R SPI SS port          */
////
#define ST25R_INT_PIN            -1            /*!< GPIO pin used for ST25R External Interrupt    */
////#define ST25R_INT_PORT           ST25R_IRQ_GPIO_Port      /*!< GPIO port used for ST25R External Interrupt   */
////
////#ifdef LED_FIELD_Pin
////#define PLATFORM_LED_FIELD_PIN      LED_FIELD_Pin       /*!< GPIO pin used as field LED                        */
////#endif
////
////#ifdef LED_FIELD_GPIO_Port
////#define PLATFORM_LED_FIELD_PORT     LED_FIELD_GPIO_Port /*!< GPIO port used as field LED                       */
////#endif
////
////
////#define PLATFORM_LED_A_PIN           LED_A_Pin             /*!< GPIO pin used for LED A    */
////#define PLATFORM_LED_A_PORT          LED_A_GPIO_Port       /*!< GPIO port used for LED A   */
////#define PLATFORM_LED_B_PIN           LED_B_Pin             /*!< GPIO pin used for LED B    */
////#define PLATFORM_LED_B_PORT          LED_B_GPIO_Port       /*!< GPIO port used for LED B   */
////#define PLATFORM_LED_F_PIN           LED_F_Pin             /*!< GPIO pin used for LED F    */
////#define PLATFORM_LED_F_PORT          LED_F_GPIO_Port       /*!< GPIO port used for LED F   */
////#define PLATFORM_LED_V_PIN           LED_V_Pin             /*!< GPIO pin used for LED V    */
////#define PLATFORM_LED_V_PORT          LED_V_GPIO_Port       /*!< GPIO port used for LED V   */
////#define PLATFORM_LED_AP2P_PIN        LED_AP2P_Pin          /*!< GPIO pin used for LED AP2P */
////#define PLATFORM_LED_AP2P_PORT       LED_AP2P_GPIO_Port    /*!< GPIO port used for LED AP2P*/
////
////#define PLATFORM_USER_BUTTON_PIN     B1_Pin                /*!< GPIO pin user button       */
////#define PLATFORM_USER_BUTTON_PORT    B1_GPIO_Port          /*!< GPIO port user button      */
////

#define platformProtectST25RComm()                  if (global_st25r3911b->spi_semaphore != NULL) xSemaphoreTake(global_st25r3911b->spi_semaphore, portMAX_DELAY);/*!< Protect unique access to ST25R communication channel - IRQ disable on single thread environment (MCU) ; Mutex lock on a multi thread environment      */
#define platformUnprotectST25RComm()                if (global_st25r3911b->spi_semaphore != NULL) xSemaphoreGive(global_st25r3911b->spi_semaphore);/*!< Unprotect unique access to ST25R communication channel - IRQ enable on a single thread environment (MCU) ; Mutex unlock on a multi thread environment */

//#define platformProtectST25RIrqStatus()           platformProtectST25RComm()                /*!< Protect unique access to IRQ status var - IRQ disable on single thread environment (MCU) ; Mutex lock on a multi thread environment */
//#define platformUnprotectST25RIrqStatus()         platformUnprotectST25RComm()              /*!< Unprotect the IRQ status var - IRQ enable on a single thread environment (MCU) ; Mutex unlock on a multi thread environment         */
//
//#define platformProtectWorker()                                                                     /* Protect RFAL Worker/Task/Process from concurrent execution on multi thread platforms   */
//#define platformUnprotectWorker()                                                                   /* Unprotect RFAL Worker/Task/Process from concurrent execution on multi thread platforms */
//
//#define platformIrqST25RSetCallback( cb )
//#define platformIrqST25RPinInitialize()
//
//#define platformIrqST25RSetCallback( cb )
//#define platformIrqST25RPinInitialize()
//
//
//#define platformLedsInitialize()                                                                    /*!< Initializes the pins used as LEDs to outputs*/
//
//#define platformLedOff( port, pin )                   platformGpioClear((port), (pin))              /*!< Turns the given LED Off                     */
//#define platformLedOn( port, pin )                    platformGpioSet((port), (pin))                /*!< Turns the given LED On                      */
//#define platformLedToggle( port, pin )                platformGpioToggle((port), (pin))             /*!< Toggle the given LED                        */
//
//#define platformGpioSet( port, pin )                  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)    /*!< Turns the given GPIO High                   */
//#define platformGpioClear( port, pin )                HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)  /*!< Turns the given GPIO Low                    */
//#define platformGpioToggle( port, pin )               HAL_GPIO_TogglePin(port, pin)                 /*!< Toggles the given GPIO                      */
#define platformGpioIsHigh( port, pin )               (gpio_get_level(pin) == true) /*!< Checks if the given LED is High             */
#define platformGpioIsLow( port, pin )                (!platformGpioIsHigh(port, pin))              /*!< Checks if the given LED is Low              */
//

#define MAX(a, b) ((a > b) ? a : b)
#define platformTimerCreate( t )                      xTaskGetTickCount() + (MAX(t, 10) / portTICK_PERIOD_MS)/*!< Create a timer with the given time (ms)     */
#define platformTimerIsExpired( timer )               (xTaskGetTickCount() > timer)                         /*!< Checks if the given timer is expired        */
#define platformTimerDestroy( timer )                                                               /*!< Stop and release the given timer            */
#define platformDelay( t )                            vTaskDelay( t / portTICK_PERIOD_MS )                                /*!< Performs a delay for the given time (ms)    */

#define platformGetSysTick()                                                           /*!< Get System Tick ( 1 tick = 1 ms)            */

//#define platformErrorHandle()                         _Error_Handler(__FILE__,__LINE__)             /*!< Global error handler or trap                */
//
#define platformSpiSelect()                         gpio_set_level(global_st25r3911b->pin_cs, false) /*!< SPI SS\CS: Chip|Slave Select                */
#define platformSpiDeselect()                       gpio_set_level(global_st25r3911b->pin_cs, true) /*!< SPI SS\CS: Chip|Slave Deselect              */
#define platformSpiTxRx( txBuf, rxBuf, len )          rfid_rxtx(global_st25r3911b, txBuf, rxBuf, len)                    /*!< SPI transceive                              */
//
//#define platformLog(...)                              ESP_LOGI("nfc", __VA_ARGS__)                         /*!< Log  method                                 */

//extern uint8_t globalCommProtectCnt;                      /* Global Protection Counter provided per platform - instantiated in main.c    */


#define RFAL_FEATURE_LISTEN_MODE               false      /*!< Enable/Disable RFAL support for Listen Mode                               */
#define RFAL_FEATURE_WAKEUP_MODE               true       /*!< Enable/Disable RFAL support for the Wake-Up mode                          */
#define RFAL_FEATURE_LOWPOWER_MODE             false      /*!< Enable/Disable RFAL support for the Low Power mode                        */
#define RFAL_FEATURE_NFCA                      true       /*!< Enable/Disable RFAL support for NFC-A (ISO14443A)                         */
#define RFAL_FEATURE_NFCB                      true       /*!< Enable/Disable RFAL support for NFC-B (ISO14443B)                         */
#define RFAL_FEATURE_NFCF                      true       /*!< Enable/Disable RFAL support for NFC-F (FeliCa)                            */
#define RFAL_FEATURE_NFCV                      true       /*!< Enable/Disable RFAL support for NFC-V (ISO15693)                          */
#define RFAL_FEATURE_T1T                       true       /*!< Enable/Disable RFAL support for T1T (Topaz)                               */
#define RFAL_FEATURE_T2T                       true       /*!< Enable/Disable RFAL support for T2T                                       */
#define RFAL_FEATURE_T4T                       true       /*!< Enable/Disable RFAL support for T4T                                       */
#define RFAL_FEATURE_ST25TB                    true       /*!< Enable/Disable RFAL support for ST25TB                                    */
#define RFAL_FEATURE_ST25xV                    true       /*!< Enable/Disable RFAL support for ST25TV/ST25DV                             */
#define RFAL_FEATURE_DYNAMIC_ANALOG_CONFIG     false      /*!< Enable/Disable Analog Configs to be dynamically updated (RAM)             */
#define RFAL_FEATURE_DPO                       false      /*!< Enable/Disable RFAL Dynamic Power Output support                          */
#define RFAL_FEATURE_ISO_DEP                   true       /*!< Enable/Disable RFAL support for ISO-DEP (ISO14443-4)                      */
#define RFAL_FEATURE_ISO_DEP_POLL              true       /*!< Enable/Disable RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
#define RFAL_FEATURE_ISO_DEP_LISTEN            false      /*!< Enable/Disable RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
#define RFAL_FEATURE_NFC_DEP                   true       /*!< Enable/Disable RFAL support for NFC-DEP (NFCIP1/P2P)                      */

#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256U       /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
#define RFAL_FEATURE_NFC_DEP_BLOCK_MAX_LEN     254U       /*!< NFC-DEP Block/Payload length. Allowed values: 64, 128, 192, 254           */
#define RFAL_FEATURE_NFC_RF_BUF_LEN            258U       /*!< RF buffer length used by RFAL NFC layer                                   */

#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      512U       /*!< ISO-DEP APDU max length. Please use multiples of I-Block max length       */
#define RFAL_FEATURE_NFC_DEP_PDU_MAX_LEN       512U       /*!< NFC-DEP PDU max length.                                                   */
