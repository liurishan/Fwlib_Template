#ifndef MY_DELAY_H
#define MY_DELAY_H

#include "os_type.h"
#include "osapi.h"
#include "ets_sys.h"

void ICACHE_FLASH_ATTR delay_s(uint16 time);
void ICACHE_FLASH_ATTR delay_ms(u32 C_time);

#endif
