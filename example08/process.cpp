/* Generated by re2c 0.16 */
#line 1 "process.re"
/*----------------------------------------------------------------------
  process.re - re2c source to generate command line tokenizer
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "process.h"

#define YYMAXFILL 6

bool process(process_t *scanner)
{
    const char *YYMARKER=0;
    const char *s = scanner->stok;
loop:
    
#line 20 "process.cpp"
{
	char yych;
	unsigned int yyaccept = 0;
	yych = *s;
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
	++s;
#line 27 "process.re"
	{ scanner->tok=END;     goto process_exit; }
#line 56 "process.cpp"
yy4:
	++s;
yy5:
#line 28 "process.re"
	{ scanner->tok=yych;    goto process_exit; }
#line 62 "process.cpp"
yy6:
	++s;
	yych = *s;
	switch (yych) {
	case '\t':
	case ' ':	goto yy6;
	default:	goto yy8;
	}
yy8:
#line 29 "process.re"
	{ scanner->stok=s;      goto loop;         }
#line 74 "process.cpp"
yy9:
	++s;
	yych = *s;
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
#line 30 "process.re"
	{ scanner->tok=DEC;     goto process_exit; }
#line 94 "process.cpp"
yy12:
	++s;
yy13:
#line 31 "process.re"
	{ scanner->tok=HELP;    goto process_exit; }
#line 100 "process.cpp"
yy14:
	yyaccept = 0;
	yych = *(YYMARKER = ++s);
	switch (yych) {
	case 'L':
	case 'l':	goto yy19;
	default:	goto yy5;
	}
yy15:
	yyaccept = 1;
	yych = *(YYMARKER = ++s);
	switch (yych) {
	case 'E':
	case 'e':	goto yy21;
	default:	goto yy13;
	}
yy16:
	yyaccept = 0;
	yych = *(YYMARKER = ++s);
	switch (yych) {
	case 'E':
	case 'e':	goto yy22;
	default:	goto yy5;
	}
yy17:
	yyaccept = 0;
	yych = *(YYMARKER = ++s);
	switch (yych) {
	case 'F':
	case 'f':	goto yy23;
	case 'N':
	case 'n':	goto yy24;
	default:	goto yy5;
	}
yy18:
	yyaccept = 0;
	yych = *(YYMARKER = ++s);
	switch (yych) {
	case 'P':
	case 'p':	goto yy26;
	default:	goto yy5;
	}
yy19:
	yych = *++s;
	switch (yych) {
	case 'I':
	case 'i':	goto yy27;
	default:	goto yy20;
	}
yy20:
	s = YYMARKER;
	if (yyaccept == 0) {
		goto yy5;
	} else {
		goto yy13;
	}
yy21:
	yych = *++s;
	switch (yych) {
	case 'L':
	case 'l':	goto yy28;
	default:	goto yy20;
	}
yy22:
	yych = *++s;
	switch (yych) {
	case 'D':
	case 'd':	goto yy29;
	default:	goto yy20;
	}
yy23:
	yych = *++s;
	switch (yych) {
	case 'F':
	case 'f':	goto yy31;
	default:	goto yy20;
	}
yy24:
	++s;
#line 36 "process.re"
	{ scanner->tok=ON;      goto process_exit; }
#line 182 "process.cpp"
yy26:
	yych = *++s;
	switch (yych) {
	case 'T':
	case 't':	goto yy33;
	default:	goto yy20;
	}
yy27:
	yych = *++s;
	switch (yych) {
	case 'N':
	case 'n':	goto yy34;
	default:	goto yy20;
	}
yy28:
	yych = *++s;
	switch (yych) {
	case 'P':
	case 'p':	goto yy12;
	default:	goto yy20;
	}
yy29:
	++s;
#line 32 "process.re"
	{ scanner->tok=LED;     goto process_exit; }
#line 208 "process.cpp"
yy31:
	++s;
#line 35 "process.re"
	{ scanner->tok=OFF;     goto process_exit; }
#line 213 "process.cpp"
yy33:
	yych = *++s;
	switch (yych) {
	case 'I':
	case 'i':	goto yy35;
	default:	goto yy20;
	}
yy34:
	yych = *++s;
	switch (yych) {
	case 'K':
	case 'k':	goto yy36;
	default:	goto yy20;
	}
yy35:
	yych = *++s;
	switch (yych) {
	case 'M':
	case 'm':	goto yy38;
	default:	goto yy20;
	}
yy36:
	++s;
#line 33 "process.re"
	{ scanner->tok=BLINK;   goto process_exit; }
#line 239 "process.cpp"
yy38:
	yych = *++s;
	switch (yych) {
	case 'E':
	case 'e':	goto yy39;
	default:	goto yy20;
	}
yy39:
	++s;
#line 34 "process.re"
	{ scanner->tok=UPTIME;  goto process_exit; }
#line 251 "process.cpp"
}
#line 37 "process.re"


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
