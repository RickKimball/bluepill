/*----------------------------------------------------------------------
  process.re - re2c source to generate command line tokenizer
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "process.h"

/*!max:re2c*/

bool process(process_t *scanner)
{
    const char *YYMARKER=0;
    const char *s = scanner->stok;
loop:
    /*!re2c
        re2c:define:YYCTYPE = char;
        re2c:define:YYCURSOR = s;
        re2c:yyfill:enable = 0;

        end     = [\x00\r];
        ws      = [ \t]+;
        dec     = [1-9][0-9]*;
        help    = 'help' | 'h' | '?';
        uptime  = 'uptime';

        end      { scanner->tok=END;     goto process_exit; }
        *        { scanner->tok=yych;    goto process_exit; }
        ws       { scanner->stok=s;      goto loop;         }
        dec      { scanner->tok=DEC;     goto process_exit; }
        help     { scanner->tok=HELP;    goto process_exit; }
        'led'    { scanner->tok=LED;     goto process_exit; }
        'blink'  { scanner->tok=BLINK;   goto process_exit; }
        'uptime' { scanner->tok=UPTIME;  goto process_exit; }
        'off'    { scanner->tok=OFF;     goto process_exit; }
        'on'     { scanner->tok=ON;      goto process_exit; }
    */

process_exit:
    scanner->cursor = s; // set up YYCURSOR scan
    scanner->len = (scanner->cursor - scanner->stok);

    if ( scanner->tok == DEC) {
        scanner->value = strtol(scanner->stok,0,10);
    }
    else {
        scanner->value = 0;
    }
        
    return ( scanner->tok == END ) ? true : false;
}
