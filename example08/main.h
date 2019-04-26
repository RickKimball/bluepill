/*--------------------------------------------------------------------------------
 * main.h
 */
#pragma once

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <common.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <inttypes.h>
#include <string.h>
#include <Scheduler.h>
#include "vectors.h"
#include "sysclock.h"
#include "process.h"

#define BAUD 115200
#define sizeofs(a) (int)(sizeof(a)/sizeof(a[0]))

// shared variables used by blink task and command handler 
enum led_states { LED_ON_STATE, LED_OFF_STATE, LED_BLINK_STATE };

// cmd_t - describes cmds using token match pattern along with its handler function
struct cmd_t {
  int arg_cnt;
  int token_pattern[6];
  const char *help;
  void (*cmd_handler)(process_t *);
};

/*---------------------------------------------------------------------
 function signatures
 */
extern "C" void _init(void);
extern "C" int main(void);

static void task0();
static void blink_task();

static void add_command_ch(int c);
static void cmd_led_display(process_t *tokens);
static void cmd_help(process_t *tokens);
static void command_handler();
static void delay_until(unsigned & tick_start, const unsigned msecs);

// turn off blinky and turn it always on or off
static void update_blink_settings(led_states state);
// default blink is 1Hz 50% duty
static void update_blink_settings(unsigned on=500, unsigned off=500);

/*---------------------------------------------------------------------
  globals
  */
static const uint32_t APB2_DIV = 1;
static const uint32_t APB1_DIV = (F_CPU>36000000) ? 2 : 1;

volatile unsigned tickcnt;            // SysTick_Handler msec counter

// shared variables used by blink task and command handler 
// default startup state is to blink at 1Hz with short msec on time
volatile led_states led_state=LED_BLINK_STATE;
volatile unsigned on_time=100, off_time=900; 

// command line task globals
#define BACKSPACE 127 /* DEL key */
static const char prompt[] = { "command> "};

// buffers used to hold command line data
static char input_buf[79+YYMAXFILL+1];
static char *pbuf = input_buf;
static char *pbufend = &input_buf[sizeof(input_buf)-YYMAXFILL-sizeof(prompt)];

// describe the valid match patterns and the function to call
static const cmd_t cmd_list[] = {
  { 1, {END},
    0, // empty command is ok be we don't give help on it
    [](process_t *){ return; }
  },
  { 2, {HELP, END},
    "help | 'h' | '?'    - show available commands and arguments",
    cmd_help
  },
  { 2, {LED, END},
    "led                 - show led settings",
    cmd_led_display
  },
  { 3, {LED, BLINK, END},
    "led blink           - enable blink, on for 500 msecs off for 500 msecs",
    [](process_t *){ update_blink_settings(); }
  },
  { 4, {LED, BLINK, DEC, END},
    "led blink number    - enable blink, on for msecs off for msecs",
     [](process_t * args){ update_blink_settings(args[2].value, args[2].value); },
  },
  { 5, {LED, BLINK, DEC, HZ, END},
    "led blink number Hz - enable blink, enter cycles per second (1Hz, 4Hz, ...)",
    [](process_t * args){
      int value = args[2].value*2;

      if ( value > 1000 ) {
        printf("Error: Hz must be less than 500\n");
      }
      else {
        value = (1000/(args[2].value*2));
        update_blink_settings(value, value);
      }
    },
  },
  { 5, {LED, BLINK, DEC, DEC, END},
    0, // "led blink num1 num2 - enable blink, on for num1, off for num2 in msecs",
    [](process_t * args){ update_blink_settings(args[2].value, args[3].value); },
  },
  { 6, {LED, BLINK, DEC, ',', DEC, END},
    "led blink num1,num2 - enable blink, on for num1, off for num2 in msecs",
    [](process_t * args){ update_blink_settings(args[2].value, args[4].value); },
  },
  { 3, {LED, ON, END},
    "led on              - disable blink, force the led on",
    [](process_t *){ update_blink_settings(LED_ON_STATE); }
  },
  { 3, {LED, OFF, END},
    "led off             - disable blink, force the led off",
    [](process_t *){ update_blink_settings(LED_OFF_STATE); }
  },
  { 2, {UPTIME, END},
    "uptime              - display the elapsed msecs since power on",
    [](process_t *){ printf("up %d msecs\n", tickcnt); }
  },
};
