#ifndef DRIVER_CC1200_MISC_H
#define DRIVER_CC1200_MISC_H

/*---------------------------------------------------------------------------*/
/* This is the way we handle the LEDs */
/*---------------------------------------------------------------------------*/
#ifdef CC1200_TX_LEDS
#define TX_LEDS_ON()  leds_on(CC1200_TX_LEDS)
#define TX_LEDS_OFF() leds_off(CC1200_TX_LEDS)
#else
#define TX_LEDS_ON()
#define TX_LEDS_OFF()
#endif /* #ifdef CC1200_TX_LEDS */

#ifdef CC1200_RX_LEDS
#define RX_LEDS_ON()  leds_on(CC1200_RX_LEDS)
#define RX_LEDS_OFF() leds_off(CC1200_RX_LEDS)
#else
#define RX_LEDS_ON()
#define RX_LEDS_OFF()
#endif /* #ifdef CC1200_RX_LEDS */
/*---------------------------------------------------------------------------*/

#define BUSYWAIT_UNTIL(cond, max_time)                                        \
    do {                                                                      \
        uint32_t t0;                                                          \
        uint32_t now;                                                         \
        t0 = xTaskGetTickCount();                                             \
        while (!(cond)) {                                                     \
            now = xTaskGetTickCount();                                        \
            if (now - t0 > max_time / portTICK_PERIOD_MS) {                   \
                ESP_LOGE(TAG, "RF: Timeout exceeded in line %d!n", __LINE__); \
                break;                                                        \
            }                                                                 \
            vTaskDelay(10);                                                   \
        }                                                                     \
    } while (0)

#define BUSYWAIT_UNTIL_STATE(X, s, t)                                            \
    do {                                                                      \
        uint32_t t0;                                                          \
        uint32_t now;                                                         \
        t0 = xTaskGetTickCount();                                             \
        while (state(X) != s) {                                                \
            now = xTaskGetTickCount();                                        \
            if (now - t0 > t / portTICK_PERIOD_MS) {                          \
                ESP_LOGE(TAG, "RF: Timeout exceeded in line %d!n", __LINE__); \
                break;                                                        \
            }                                                                 \
            vTaskDelay(10);                                                   \
        }                                                                     \
    } while (0)

/*---------------------------------------------------------------------------*/
#if CC1200_USE_GPIO2
/* Configure GPIO interrupts. GPIO0: falling, GPIO2: rising edge */
#define SETUP_GPIO_INTERRUPTS()         \
    do {                                \
        cc1200_arch_gpio0_setup_irq(0); \
        cc1200_arch_gpio2_setup_irq(1); \
    } while (0)
#define ENABLE_GPIO_INTERRUPTS()        \
    do {                                \
        cc1200_arch_gpio0_enable_irq(); \
        cc1200_arch_gpio2_enable_irq(); \
    } while (0)
#define DISABLE_GPIO_INTERRUPTS()        \
    do {                                 \
        cc1200_arch_gpio0_disable_irq(); \
        cc1200_arch_gpio2_disable_irq(); \
    } while (0)
#else
#define SETUP_GPIO_INTERRUPTS(X)   cc1200_arch_gpio0_setup_irq(X, 0)
#define ENABLE_GPIO_INTERRUPTS(X)  cc1200_arch_gpio0_enable_irq(X)
#define DISABLE_GPIO_INTERRUPTS(X) cc1200_arch_gpio0_disable_irq(X)
#endif /* #if CC1200_USE_GPIO2 */

/*---------------------------------------------------------------------------*/
/* Various implementation specific defines */
/*---------------------------------------------------------------------------*/
/*
 * The debug level to use
 * - 0: No output at all
 * - 1: Print errors (unrecoverable)
 * - 2: Print errors + warnings (recoverable errors)
 * - 3: Print errors + warnings + information (what's going on...)
 */
#define DEBUG_LEVEL 3
/*---------------------------------------------------------------------------*/
/* Debug macros */
/*---------------------------------------------------------------------------*/
#if DEBUG_LEVEL > 0
/* Show all kind of errors e.g. when passing invalid payload length */
#define ERROR(...) ESP_LOGE(TAG, __VA_ARGS__)
#else
#define ERROR(...)
#endif

#if DEBUG_LEVEL > 0
/* This macro is used to check if the radio is in a valid state */
#define RF_ASSERT(condition)                                            \
    do {                                                                \
        if (!(condition)) {                                             \
            ESP_LOGE(TAG, "RF: Assertion failed in line %d", __LINE__); \
        }                                                               \
    } while (0)
#else
#define RF_ASSERT(condition)
#endif

#if DEBUG_LEVEL > 1
/* Show warnings e.g. for FIFO errors */
#define WARNING(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define WARNING(...)
#endif

#if DEBUG_LEVEL > 2
/* We just print out what's going on */
#define INFO(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define INFO(...)
#endif

#define trc      INFO("%s():%d", __func__, __LINE__);
#define INFOX(x) INFO("%s():%d " #x " = 0x%x", __func__, __LINE__, (x));
#define INFOD(x) INFO("%s():%d " #x " = %d", __func__, __LINE__, (x));
#define INFOS(x) INFO("%s():%d %s", __func__, __LINE__, (x));

#endif
