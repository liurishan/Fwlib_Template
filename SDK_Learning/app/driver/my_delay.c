#include "os_type.h"
#include "osapi.h"
#include "ets_sys.h"
#include "driver/my_delay.h"

void ICACHE_FLASH_ATTR delay_s(uint16 time)
{
	for(; time > 0; time--)
	{
		int i = 1;
		for(; i <= 1000; i++)
		{
			os_delay_us(1000);
			system_soft_wdt_feed();
		}
	}
}

void ICACHE_FLASH_ATTR delay_ms(u32 C_time)
{	for(;C_time>0;C_time--)
	{
		os_delay_us(1000);
		system_soft_wdt_feed();
	}
}
