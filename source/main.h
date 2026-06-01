/*
 * main.h - Shared types and module entry points
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

/* Key message types - shared between key.c and lcd.c */
typedef enum {
    MSG_INCREASE_PAGE,
    MSG_DECREASE_PAGE,
    MSG_INCREASE_VALUE,
    MSG_DECREASE_VALUE,
    MSG_CONFIRM
} MessageType_e;

typedef struct {
    MessageType_e type;
    bool confirm;
} key_message_t;

#endif /* MAIN_H */
