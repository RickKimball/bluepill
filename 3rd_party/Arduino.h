/*----------------------------------------------------------------------
 Arduino.h - minimal header to support Scheduler.h
 
 This is original code for the bluepill examples.
 */

#pragma once

#include <inttypes.h>
#include <malloc.h>

#if __cplusplus
extern "C" {
#endif
void yield(void);
#if __cplusplus
}
#endif
