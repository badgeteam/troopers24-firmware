#pragma once

#include "driver_cc1200.h"
#define CC1200_TROOPERS_MAX_BEHIND 500
#define CC1200_TROOPERS_HASH_SIZE  32  // sha256
#define CC1200_TROOPERS_CHAIN_TAIL \
    "\x65\xa1\xaf\xbd\x13\x84\x33\xc3\xbd\x29\xbb\x37\xb8\x45\xbb\xc1\xdb\x2e\xc5\x0a\x0f\xe9\xed\x9d\x48\x5d\xeb\xd2\x62\x84\xe3\xd3"
#define CC1200_TROOPERS_MAGIC 0xc0ffee00

#define CC1200_TROOPERS_MSGTYPE_COUNTDOWN          1
#define CC1200_TROOPERS_MSGTYPE_TALK               2
#define CC1200_TROOPERS_MSGTYPE_BREAK              3
#define CC1200_TROOPERS_MSGTYPE_RESET              4
#define CC1200_TROOPERS_MSGTYPE_COUNTDOWN_SET_TIME 5

extern bool driver_cc1200_troopers_animation_running;

typedef struct cc1200_troopers_message_t {
    uint32_t magic;
    uint8_t  type;
    uint16_t payload;
    // Replay protection
    uint16_t msgid;
    uint8_t  hash[CC1200_TROOPERS_HASH_SIZE];
} __attribute__((packed, aligned(1))) cc1200_troopers_message_t;

typedef struct cc1200_troopers_state_t {
    uint16_t msgid;
    uint8_t  hash[CC1200_TROOPERS_HASH_SIZE];
} cc1200_troopers_state_t;

esp_err_t cc1200_troopers_init();

bool cc1200_troopers_replay_protection(cc1200_troopers_message_t *msg);

bool cc1200_troopers_cb(cc1200_message *msg);

void cc1200_debug_start_countdown();
