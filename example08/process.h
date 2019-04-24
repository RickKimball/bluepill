/*----------------------------------------------------------------------
  process.h - scanner enums and structure for tokenizing command line
 */

#pragma once

#include "process_defines.h"

enum process_token_t
{
    END=256,  // must be greater than all ascii characters
    ERR,
    BLINK,
    DEC,
    HELP,
    LED,
    OFF,
    ON,
    UPTIME,
    WS,
    LAST
};

typedef struct {
    const char *stok;       // pointer to first character of token string
    const char *cursor;     // pointer to first character of next token to be parsed
    int len;                // length of strok
    int tok;                // enum token representation of string
    int value;              // if numeric, contains the string converted to int

    void print(const char *msg="") {
        printf("%sscanner->tok=%d [%c], stok='%.*s' value=%d\n",
            msg,
            tok,
            ((tok < 127) ? tok : '.'), (len),
            stok, value);
    }

} process_t;

bool process(process_t *scanner);

/* vim: set ts=4 sw=4 expandtab : */

/* makefile replaces this with real value*/
