/*--------------------------------------------------------------------------------
 * main.h
 */
#pragma once

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <stdio.h>
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
static void cmd_led_blink_1(process_t *tokens);
static void cmd_led_blink_2(process_t *tokens);
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
    "help | 'h' | '?'          - display available commands and arguments",
    cmd_help
  },
  { 2, {LED, END},
    "led                       - show the current led settings",
    cmd_led_display
  },
  { 3, {LED, BLINK, END},
    "led blink                 - enable blink, 1Hz 50% duty",
    [](process_t *){ update_blink_settings(); }
  },
  { 4, {LED, BLINK, DEC, END},
    "led blink number          - enable blink, on and off time in msecs",
     cmd_led_blink_1
  },
  { 6, {LED, BLINK, DEC, ',', DEC, END},
    "led blink number1,number2 - enable blink, on time, off time in msecs",
    cmd_led_blink_2
  },
  { 3, {LED, ON, END},
    "led on                    - diable blink, force the led on",
    [](process_t *){ update_blink_settings(LED_ON_STATE); }
  },
  { 3, {LED, OFF, END},
    "led off                   - blink disable, force the led off",
    [](process_t *){ update_blink_settings(LED_OFF_STATE); }
  },
  { 2, {UPTIME, END},
    "uptime                    - display the elapsed msecs since power on",
    [](process_t *){ printf("up %d msecs\n", tickcnt); }
  },
};
