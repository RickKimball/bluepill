/*---------------------------------------------------------------------
  os.c - override new lib dummy functions with real ones
  */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <stm32f103xb.h>
#include <FreeRTOS.h>
#include <task.h>

/*---------------------------------------------------------------------
 * _write() - implement usart output
 */
int _write(int file, char *ptr, int len) {
  switch(file) {
  case 1:   /* stdout */
  case 2:   /* stderr */
    for (int n=0; n < len; ++n) {
      while (!(USART1->SR & USART_SR_TXE)) {
        //vTaskDelay(0);
        taskYIELD();
      }
      USART1->DR = (*ptr++ & (uint16_t)0xFF);
    }
    return len;
  default:
    errno = EBADF;
    return -1;
  }
}

/* vim: set ts=2 sw=2 expandtab: */
