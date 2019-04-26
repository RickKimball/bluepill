/* Generated by re2c 0.16 */
#line 1 "process.re"
/*----------------------------------------------------------------------
  process.re - re2c source to generate command line tokenizer
 */
#include <stdio.h>
#include <stdlib.h>
#include "process.h"

#define YYMAXFILL 6

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
    
#line 26 "process.cpp"
{
	char yych;
	unsigned int yyaccept = 0;
	yych = *cursor;
	switch (yych) {
	case 0x00:
	case '\r':	goto yy2;
	case '\t':
	case ' ':	goto yy6;
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':	goto yy9;
	case '?':	goto yy12;
	case 'B':
	case 'b':	goto yy14;
	case 'H':
	case 'h':	goto yy15;
	case 'L':
	case 'l':	goto yy16;
	case 'O':
	case 'o':	goto yy17;
	case 'U':
	case 'u':	goto yy18;
	default:	goto yy4;
	}
yy2:
	++cursor;
#line 35 "process.re"
	{ scanner->token_id=END;       goto process_exit; }
#line 62 "process.cpp"
yy4:
	++cursor;
yy5:
#line 36 "process.re"
	{ scanner->token_id=yych;      goto process_exit; }
#line 68 "process.cpp"
yy6:
	++cursor;
	yych = *cursor;
	switch (yych) {
	case '\t':
	case ' ':	goto yy6;
	default:	goto yy8;
	}
yy8:
#line 37 "process.re"
	{ scanner->token=cursor;       goto loop;         }
#line 80 "process.cpp"
yy9:
	++cursor;
	yych = *cursor;
	switch (yych) {
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':	goto yy9;
	default:	goto yy11;
	}
yy11:
#line 38 "process.re"
	{ scanner->token_id=DEC;       goto process_exit; }
#line 100 "process.cpp"
yy12:
	++cursor;
yy13:
#line 40 "process.re"
	{ scanner->token_id=HELP;      goto process_exit; }
#line 106 "process.cpp"
yy14:
	yyaccept = 0;
	yych = *(marker = ++cursor);
	switch (yych) {
	case 'L':
	case 'l':	goto yy19;
	default:	goto yy5;
	}
yy15:
	yyaccept = 1;
	yych = *(marker = ++cursor);
	switch (yych) {
	case 'E':
	case 'e':	goto yy21;
	case 'Z':
	case 'z':	goto yy22;
	default:	goto yy13;
	}
yy16:
	yyaccept = 0;
	yych = *(marker = ++cursor);
	switch (yych) {
	case 'E':
	case 'e':	goto yy24;
	default:	goto yy5;
	}
yy17:
	yyaccept = 0;
	yych = *(marker = ++cursor);
	switch (yych) {
	case 'F':
	case 'f':	goto yy25;
	case 'N':
	case 'n':	goto yy26;
	default:	goto yy5;
	}
yy18:
	yyaccept = 0;
	yych = *(marker = ++cursor);
	switch (yych) {
	case 'P':
	case 'p':	goto yy28;
	default:	goto yy5;
	}
yy19:
	yych = *++cursor;
	switch (yych) {
	case 'I':
	case 'i':	goto yy29;
	default:	goto yy20;
	}
yy20:
	cursor = marker;
	if (yyaccept == 0) {
		goto yy5;
	} else {
		goto yy13;
	}
yy21:
	yych = *++cursor;
	switch (yych) {
	case 'L':
	case 'l':	goto yy30;
	default:	goto yy20;
	}
yy22:
	++cursor;
#line 41 "process.re"
	{ scanner->token_id=HZ;        goto process_exit; }
#line 176 "process.cpp"
yy24:
	yych = *++cursor;
	switch (yych) {
	case 'D':
	case 'd':	goto yy31;
	default:	goto yy20;
	}
yy25:
	yych = *++cursor;
	switch (yych) {
	case 'F':
	case 'f':	goto yy33;
	default:	goto yy20;
	}
yy26:
	++cursor;
#line 44 "process.re"
	{ scanner->token_id=ON;        goto process_exit; }
#line 195 "process.cpp"
yy28:
	yych = *++cursor;
	switch (yych) {
	case 'T':
	case 't':	goto yy35;
	default:	goto yy20;
	}
yy29:
	yych = *++cursor;
	switch (yych) {
	case 'N':
	case 'n':	goto yy36;
	default:	goto yy20;
	}
yy30:
	yych = *++cursor;
	switch (yych) {
	case 'P':
	case 'p':	goto yy12;
	default:	goto yy20;
	}
yy31:
	++cursor;
#line 42 "process.re"
	{ scanner->token_id=LED;       goto process_exit; }
#line 221 "process.cpp"
yy33:
	++cursor;
#line 43 "process.re"
	{ scanner->token_id=OFF;       goto process_exit; }
#line 226 "process.cpp"
yy35:
	yych = *++cursor;
	switch (yych) {
	case 'I':
	case 'i':	goto yy37;
	default:	goto yy20;
	}
yy36:
	yych = *++cursor;
	switch (yych) {
	case 'K':
	case 'k':	goto yy38;
	default:	goto yy20;
	}
yy37:
	yych = *++cursor;
	switch (yych) {
	case 'M':
	case 'm':	goto yy40;
	default:	goto yy20;
	}
yy38:
	++cursor;
#line 39 "process.re"
	{ scanner->token_id=BLINK;     goto process_exit; }
#line 252 "process.cpp"
yy40:
	yych = *++cursor;
	switch (yych) {
	case 'E':
	case 'e':	goto yy41;
	default:	goto yy20;
	}
yy41:
	++cursor;
#line 45 "process.re"
	{ scanner->token_id=UPTIME;    goto process_exit; }
#line 264 "process.cpp"
}
#line 46 "process.re"


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
