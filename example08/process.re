/*----------------------------------------------------------------------
  process.re - re2c source to generate command line tokenizer
 */
#include <stdio.h>
#include <stdlib.h>
#include "process.h"

/*!max:re2c*/

bool process(process_t *scanner, const char *msg)
{
  char *cursor = scanner->token = scanner->cursor;
  char *marker=0;

  #ifdef SCANNER_DEBUG
  if ( msg ) {
    scanner->print_cursor(msg);
  }
  #endif

  loop:
    /*!re2c
      re2c:define:YYCTYPE = char;
      re2c:define:YYCURSOR = cursor;
      re2c:define:YYMARKER = marker;
      re2c:yyfill:enable = 0;

      end     = [\x00\r];
      ws      = [ \t]+;
      dec     = [1-9][0-9]*;
      help    = 'help' | 'h' | '?';
      uptime  = 'uptime';

      end      { scanner->token_id=END;       goto process_exit; }
      *        { scanner->token_id=yych;      goto process_exit; }
      ws       { scanner->token=cursor;       goto loop;         }
      dec      { scanner->token_id=DEC;       goto process_exit; }
      'blink'  { scanner->token_id=BLINK;     goto process_exit; }
      help     { scanner->token_id=HELP;      goto process_exit; }
      'led'    { scanner->token_id=LED;       goto process_exit; }
      'off'    { scanner->token_id=OFF;       goto process_exit; }
      'on'     { scanner->token_id=ON;        goto process_exit; }
      'uptime' { scanner->token_id=UPTIME;    goto process_exit; }
    */

  process_exit:
    scanner->cursor = cursor;  // save cursor for next scan
    scanner->len = (scanner->cursor - scanner->token);

  if ( scanner->token_id == DEC) {
    scanner->value = strtol(scanner->token,0,10);
  }
  else {
    scanner->value = 0;
  }

  #ifdef SCANNER_DEBUG
  scanner->print("found:");
  #endif

  return ( scanner->token_id == END ) ? true : false;
}

// vim: set ts=2 sw=2 expandtab :
