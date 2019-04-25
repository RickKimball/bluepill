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


//#define SCANNER_DEBUG

#ifdef SCANNER_DEBUG
 #define SCANNER_DEBUG_MSG(s) (s)
#else
 #define SCANNER_DEBUG_MSG(s) 0
#endif

typedef struct {
    char *cursor;  // pointer to first character of next token_str to be parsed
    char *token;   // pointer to first character of token string
    int token_id;  // process_token_t representation of string
    int len;       // length of strok
    int value;     // if process_token_t::DEC, contains the string converted to int

    void print(const char *msg="") {
#ifdef SCANNER_DEBUG
        printf("%stoken_id=%3d, len=%2d, value=%-5d, ", msg, token_id,len,value);
        if ( token_id != END )
            printf("token=\"%.*s\"\n", len, token);
        else
            printf("token=\"\\r\"\n");
#endif
    }

    void print_cursor(const char *msg="") {
#ifdef SCANNER_DEBUG
        char *s=cursor, *p;
        for (p=s; *p != '\r'; ++p);

        printf("target:[%s] cursor=\"%.*s\\r\"\n", msg, p-s, s);
#endif
    }

} process_t;

bool process(process_t *scanner, const char *msg=0);

/* vim: set ts=4 sw=4 expandtab : */
