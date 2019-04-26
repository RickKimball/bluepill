/*---------------------------------------------------------------------
  main.cpp - example08 cooperative multi-tasking USART1/Blink sample

  Assumes a USB dongle connected to PA9 for 115200,8,n,1
*/
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
enum led_states { LED_ON_STATE, LED_OFF_STATE, LED_BLINK_STATE};

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
    "help                      - display available commands and arguments",
    cmd_help
  },
  { 2, {LED, END},
    "led                       - show the current led settings",
    cmd_led_display
  },
  { 3, {LED, BLINK, END},
    "led blink                 - enable blink, 1Hz 50% duty",
    [](process_t *){ update_blink_settings();}
  },
  { 3, {LED, ON, END},
    "led on                    - force the led on, blink disabled",
    [](process_t *){ update_blink_settings(LED_ON_STATE);}
  },
  { 3, {LED, OFF, END},
    "led off                   - force the led off, blink disabled",
    [](process_t *){ update_blink_settings(LED_OFF_STATE);}
  },
  { 4, {LED, BLINK, DEC, END},
    "led blink number          - enable blink, on and off time in msecs",
     cmd_led_blink_1
  },
  { 6, {LED, BLINK, DEC, ',', DEC, END},
    "led blink number1,number2 - enable blink, on time, off time in msecs",
    cmd_led_blink_2
  },
  { 2, {UPTIME, END},
    "uptime                    - display the elapsed msecs since power on",
    [](process_t *){ printf("up %d msecs\n", tickcnt);}
  },
};

/*---------------------------------------------------------------------
 Reset_Handler() - the reset exception handler

 Sets stack pointer and inits .bss and .data, optionally "color" memory,
 call global constructors, and finally calls main.
 */

void Reset_Handler(void) {
  /* set stack pointer to top of stack memory */
  asm volatile ("ldr r0, =%0" :: "i" (&_estack));
  asm volatile ("mov sp, r0");

  /* setup SYSCLK early */
  sysclock_set<F_CPU>();

#if VECT_ADDR > 0x08000000
  SCB->VTOR = VECT_ADDR;
#endif

  /* Copy the .data segment initializers from flash to SRAM */
  __asm__ volatile(
  "data_init:\n"
  " ldr   r0,=_sidata\n" /* start address of initialized data in flash */
  " ldr   r1,=_sdata\n" /* start address of data in SRAM */
  " ldr   r2,=_edata\n" /* end address of data in SRAM */
  "1:\n"
  " cmp   r1,r2\n"
  " ittt  lo\n"         /* exec following 3 instructions if r1 < r2 */
  " ldrlo r3,[r0],#4\n" /* post increment r0 after ldr */
  " strlo r3,[r1],#4\n" /* post increment r1 after str */
  " blo.n 1b\n"         /* try again */
  );

  /* Zero fill the .bss segment, assumes aligned on 4 bytes */
  __asm__ volatile(
  "bss_init:\n"
  " ldr   r0, =_sbss\n" /* start address of non initialized data in SRAM */
  " ldr   r1, =_ebss\n" /* end address of non initialized data in SRAM */
  " movs  r2, #0\n"     /* word sized zero constant */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );

#ifdef COLOR_RAM
  /* pattern fill heap ram from _end to _end_heap w/(0xcdcdcdcd) */
  /* optional, useful for gdb to detect which parts of memory are used  */
  __asm__ volatile(
  "color_heap_init:\n"
  " ldr   r0, =end\n"       /* start address of heap in SRAM */
  " ldr   r1, =_end_heap\n" /* end address of available heap SRAM */
  " mov   r2, #0xcdcd\n"    /* pattern constant */
  " movt  r2, #0xcdcd\n"    /* 0xbaadf00d */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );
#endif

#ifdef COLOR_RAM
  /* pattern fill ram from start of ram to top of stack _estack w/(0xcccccccc) */
  /* optional, useful for gdb to detect which parts of memory are used  */
  __asm__ volatile(
  "color_stack_init:\n"
  " ldr   r0, =_sstack\n"     /* start address of heap in SRAM */
  " ldr   r1, =_estack\n"     /* top of stack in SRAM */
  " mov   r2, #0xcccc\n"      /* pattern constant */
  " movt  r2, #0xcccc\n"      /* 0xcccccccc */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );
#endif

  /* _init() board, then call any global c/c++ constructors from newlib */
  __libc_init_array();
  
  main();

  while(1); /* trap if main exits */
}

/*---------------------------------------------------------------------
  _init() - initialize board

  NOTE: called from __libc_init() before main().
        It must be a "C" routine.
*/
void _init(void) {
  RCC->APB2ENR |= 0                                       /* turn on clocks */
               | RCC_APB2ENR_AFIOEN                       /* Alternate IO   */
               | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN  /* GPIO A/C       */
               | RCC_APB2ENR_USART1EN                     /* USART1         */
               ;

  /*---------------------------------------------------------------------- */
  // PA8 configured as Master Clock Out (0b10 ALT FUNC PP | 0b11 50MHz max output)
  GPIOA->CRH = (GPIOC->CRH & ~(0b1111 << ((8-8)*4))) | (0b1011 << ((8-8)*4));
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL,
    ((F_CPU > 36000000) ? RCC_CFGR_MCOSEL_PLL_DIV2 : RCC_CFGR_MCOSEL_SYSCLK));

  /*---------------------------------------------------------------------- */
  //  PC13 set to opendrain max 2MHz
  MODIFY_REG(GPIOC->CRH,(0b1111 << ((13-8)*4)),(0b0110 << ((13-8)*4)));

  /*---------------------------------------------------------------------- */
  //  PA9/PA10 configured as USART1
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  SET_BIT(RCC->APB2ENR,RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);

  GPIOA->CRH = (GPIOA->CRH & ~(0b11111111 << (9-8)*4))
                                    | (0b01001011 << (9-8)*4);  // PA10-RX1, PA9-TX1
  USART1->BRR = ((F_CPU) / BAUD) / APB2_DIV;                    // BUS SPEED is MCLK/1
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;     // enable TX, RX, USART

  /*---------------------------------------------------------------------- */
  // enable SWD debugging
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  /*---------------------------------------------------------------------- */
  // enable msec Systick ISR
  tickcnt=-500; // test tickcnt wraparound, start @ .5 seconds before rollover
  SysTick_Config(F_CPU/1000U);

#ifdef ENABLE_PROFILE_PIN
  // PA4 OUTPUT_PP, 50MHz toggle pin to profile yield time overhead (0b0011)
  MODIFY_REG(GPIOA->CRL,GPIO_CRL_CNF4,GPIO_CRL_MODE4);
#endif
}

/*---------------------------------------------------------------------
  int main() - 
  */
int main(void) {

  // blink task
  Scheduler.startLoop(blink_task,1024);

  // command line processor task
  task0();

  return 0;
}

/*---------------------------------------------------------------------
 blink_task() - toggle LED_BUILTIN on/off
 */
void blink_task() {
  unsigned start_tick=tickcnt;

  while(1) {
    switch(led_state) {
      case LED_OFF_STATE:  /* off */
        GPIOC->BSRR = (1<<13);
        delay_until(start_tick, 500);
        break;

      case LED_ON_STATE: /* on */
        GPIOC->BSRR = (1<<13) << 16;
        delay_until(start_tick, 500);
        break;

      case LED_BLINK_STATE:
        // turn on led - pull to gnd
        GPIOC->BSRR = (1<<13) << 16;
        delay_until(start_tick, on_time);

        // turn off led - let it float
        GPIOC->BSRR = (1<<13);
        delay_until(start_tick, off_time);
        break;

      default:
        printf("invalid led state %d", led_state);
    }
  }
}

/*---------------------------------------------------------------------
 task0() - main task loop, spew tickcnt
 */
void task0() {
  printf("\nexample08 - led command line processor\n\%s", prompt);
  fflush(stdout);

  while(1) {
    int rc, c;

    if ( (rc=read(0,&c,1)) > 0) {
      add_command_ch(c);
      if ( c == 0x0d) {
        command_handler();
        fprintf(stdout,prompt); fflush(stdout);
        pbuf = input_buf; // reset input buffer pointer to start
      }
    }
  }
}

/*---------------------------------------------------------------------
 add_command_ch()) - safely append input character to buffer with 
                     overflow protection and backspace assumes
                     backspace is DEL (127) but also works with CTRL-H

 */
void add_command_ch(int c) {
  switch(c) {
    case 8:
      if ( pbuf-1 >= input_buf )
        write(1, "\b ",2);
      /* fall through on purpose */
    case 127:
      if ( pbuf-1 >= input_buf ) {
        --pbuf;
        write(1,&c,1);
      }
      break;

    // '\r' is the expected enter key value
    //      and END for lexer
    case 0x0d:
      *pbuf++ = c;
      write(1,"\n",1);
      break;

    // don't output the unmatchables
    case 0x00:
      *pbuf++ = c;
      break;

    default:
      if ( pbuf < pbufend && c > 31 && c < 127 ) {
        *pbuf++ = c;
        write(1,&c,1);
      }
      break;
  }
 }

/*---------------------------------------------------------------------
 cmd_led_blink - set blink mode with default 500msec on off
 */
void cmd_led_display(process_t *tokens) {
  printf("led is %s",
          (
          led_state == LED_ON_STATE ? "On" :
          led_state == LED_OFF_STATE ? "Off" : "Blinking"
          )
        );
  if ( led_state == LED_BLINK_STATE) {
    printf(" on time is %d, off time is %d", on_time, off_time);
  }
  printf("\n");
}

/*---------------------------------------------------------------------
 cmd_led_blink_1 - set blink mode on time used for both on and off
 */
void cmd_led_blink_1(process_t *tokens) {
  update_blink_settings(tokens[2].value,tokens[2].value);
}

/*---------------------------------------------------------------------
 cmd_led_blink_2 - set blink mode with on and off time
 */
void cmd_led_blink_2(process_t *tokens) {
  update_blink_settings(tokens[2].value,tokens[4].value);
}

/*---------------------------------------------------------------------
 cmd_help - display command list and description
 */
void cmd_help(process_t *tokens) {
  printf("**help**\n");
  for (int i=0; i < sizeofs(cmd_list); ++i ) {
    if ( cmd_list[i].help)
      printf("  %s\n",cmd_list[i].help);
  }
}

/*---------------------------------------------------------------------
 command_handler() - process the command line using re2c 
                     generated process() function
 */
void command_handler() {
  // fill end of buffer with unmatchable characters
  for (int x = YYMAXFILL - 1; x; --x) add_command_ch(0x0);

  process_t process_scanner = {input_buf, 0, 0, 0, 0};
  process_t * const scanner = &process_scanner;
  
  while (1) {
      static const int max_tokens=6;
      process_t tokens[max_tokens];
      int arg_cnt=0;
  
      // scan the command input and create an array of found tokens
      // try to find the END token, bail after 6 parsed tokens

      bool done=false;
      for ( arg_cnt=0; !done && arg_cnt < max_tokens; ++arg_cnt) {
#ifdef SCANNER_DEBUG
        static const char *scanner_targets[] = {
        "led|help|uptime|'\\r'",
        "on|off|blink|\\r",
        "DEC|\\r",
        ",|\\r",
        "DEC",
        "\\r"
        };

        char msgbuf[32]; sprintf(msgbuf,"%s",scanner_targets[arg_cnt]);
#endif
        done=process(scanner,SCANNER_DEBUG_MSG(msgbuf));
        tokens[arg_cnt]=*scanner;
      }

      // iterate through the cmd list table, find entries that match the # of tokens entered
      //   then iterate through the array of token_ids and try to match with the patterns
      //     if a match found then invoke the cmd_handler and return
      while( 1 ) {
        for (int cmd_indx=0; cmd_indx < sizeofs(cmd_list); ++cmd_indx) {

          if ( cmd_list[cmd_indx].arg_cnt == arg_cnt ) {
            bool is_match=true;
            const cmd_t & cmd = cmd_list[cmd_indx];

            for ( int token_indx=0; token_indx < arg_cnt; ++token_indx ) {
              if ( tokens[token_indx].token_id != cmd.token_pattern[token_indx]) {
                is_match = false;
                break;
              }
            }

            if ( is_match ) {
              cmd.cmd_handler(tokens);
              return;
            }
          }
        }

        printf("Error: invalid command\n");
        return;
        break;
      }
  }
}

/*---------------------------------------------------------------------
 update_blink_settings() - toggle LED_BUILTIN on/off
 */
void update_blink_settings(led_states state) {
  led_state = state;
}

/*---------------------------------------------------------------------
 update_blink_settings() - turn on blinky and configure time on and off
 */
void update_blink_settings(unsigned on, unsigned off) {
  on_time = on;
  off_time = off;
  led_state = LED_BLINK_STATE;
}

/*---------------------------------------------------------------------
 delay_until(tick_start, msec) - delay for milliseconds

 once delay is met, tick_start is updated for the caller.
 */
void delay_until(unsigned & tick_start, const unsigned msecs) {
  while( (tickcnt-tick_start) < msecs) {
    yield();
  }
  tick_start += msecs;
  return;
} 

/*---------------------------------------------------------------------
  SysTick_Handler() - SYSTICK exception handler.

  Increment a 1 msec tick counter each time we get an interrupt
  from the SYSTICK.
  */

void SysTick_Handler(void) {
  ++tickcnt;
}

/*---------------------------------------------------------------------
 * _read() - implement usart input
 */
extern "C"
int _read(int file, char *ptr, int len) {
  switch(file) {
  case 0:   /* stdin */
    for (int n=0; n < len; ++n) {
      while (!(USART1->SR & USART_SR_RXNE)) {
        yield();
      }
           
      *(unsigned *)ptr++ = (USART1->DR & 0xff);
    }
    return len;
    // break;

  default:
    errno = EBADF;
    return -1;
  }
}


/*---------------------------------------------------------------------
 * _write() - implement usart output
 */
extern "C"
int _write(int file, char *ptr, int len) {
  switch(file) {
  case 1:   /* stdout */
  case 2:   /* stderr */
    for (int n=0; n < len; ++n) {
      while (!(USART1->SR & USART_SR_TXE)) {
        yield();
      }
      USART1->DR = (*ptr++ & (uint16_t)0xFF);
    }
    return len;
  default:
    errno = EBADF;
    return -1;
  }
}

/* vim: set expandtab ts=2 sw=2: */
