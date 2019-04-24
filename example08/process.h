/*----------------------------------------------------------------------
  process.h - scanner enums and structure for tokenizing command line
 */

#pragma once

enum num_t
{
    END=256,
    ERR,
    BLINK,
    DEC,
    HELP,
    LED,
    OFF,
    ON,
    WS,
    LAST
};

typedef struct {
    const char *cursor;
    const char *stok;
    int len;
    int tok;
    int value;

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
