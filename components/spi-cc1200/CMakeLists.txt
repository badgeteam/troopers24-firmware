idf_component_register(
        SRCS "cc1200.c"
        "cc1200_hal.c"
        "cc1200_spi.c"
        "cc1200_statemachine.c"
        "cc1200-868-fsk-1-2kbps.c"
        "cc1200-802154g-863-870-fsk-50kbps.c"
        "troopers.c"
        INCLUDE_DIRS include
        REQUIRES "mbedtls"
        "nvs_flash"
)
