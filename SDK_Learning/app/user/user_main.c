/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"   //回调函数
#include "osapi.h"    //软件定时器
#include "eagle_soc.h"			// GPIO函数、宏定义
#include "os_type.h"			// os_XXX
#include "user_interface.h" 	// 系统接口、system_param_xxx接口、WIFI、RateContro
#include "driver/uart.h"
#include "driver/my_delay.h"
#include "mem.h"				// 内存申请等函数
#include "c_types.h"			// 变量类型
#include "ip_addr.h"			// 被"espconn.h"使用。在"espconn.h"开头#include"ip_addr.h"或#include"ip_addr.h"放在"espconn.h"之前
#include "espconn.h"			// TCP/UDP接口
#include "driver/oled.h"



/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/



/*************MY_SPACE*****************/

//宏定义



//全局变量的定义
int LED_Flag = 0;
os_timer_t OS_Timer;
os_event_t * Pointer_Task ;  //定义串口打印任务指针
struct espconn ST_NetCon;     //网络连接结构体



//STA初始化
void ICACHE_FLASH_ATTR STA_Init()
{
	struct station_config STA_Config;   //STA参数结构体
	struct ip_info ST_IP;               //STA信息结构体

	wifi_set_opmode(0x01);   //设置为STA模式，并且会保存到Flash

	//默认开启DHCP Client，会自动分配IP

	//配置STA参数
	os_memset(&STA_Config, 0, sizeof(struct station_config));	// STA参数结构体清零
	os_strcpy(STA_Config.ssid,wifi_name);				// 设置WIFI名
	os_strcpy(STA_Config.password, wifi_password);			// 设置WIFI密码

	wifi_station_set_config(&STA_Config);    //设置STA，并保存到Flash
}



//TCP成功发射回调函数
void ICACHE_FLASH_ATTR WIFI_Send_CallBack(void *arg)
{
	//os_printf("\nESP8266_WIFI_Send_OK\n");
}

//TCP成功接收回调函数 ,参数1：网络传输结构体espconn指针、参数2：网络传输数据指针、参数3：数据长度
void ICACHE_FLASH_ATTR WIFI_Recv_CallBack(void * arg, char * pdata, unsigned short len)
{
	struct espconn * T_arg = arg;		// 缓存网络连接结构体指针

	if(pdata[0] == 'o')
	{
		LED_Flag = 0;
		GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
		os_printf("灯以被远程打开\r\n");
		//OLED_ShowString(0,0,"The phone to turn on the lights.    ");
		OLED_ShowString(0,6,"LED is on!   ");
	}
	else
	{
		LED_Flag = 1;
		GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
		os_printf("灯以被远程关闭\r\n");
		//OLED_ShowString(0,0,"The phone to turn off the lights.    ");
		OLED_ShowString(0,6,"LED is off!   ");
	}
	//OLED_ShowString(0,0,"Client is successful receipt!         ");


	espconn_send(T_arg,"ESP8266成功接收！\r\n",os_strlen("ESP8266成功接收！\r\n"));	// 向对方发送应答
}

//TCP正常断开回调函数
void ICACHE_FLASH_ATTR TCP_Disconnect_CallBack(void *arg)
{
	os_printf("\nESP8266_TCP_Disconnect_OK\n");
	OLED_ShowString(0,0,"TCP was successful disconnect!         ");
}

//TCP成功连接回调函数
void ICACHE_FLASH_ATTR TCP_Connect_CallBack(void *arg)
{
	espconn_regist_sentcb((struct espconn *)arg, WIFI_Send_CallBack);			// 注册网络数据发送成功的回调函数
	espconn_regist_recvcb((struct espconn *)arg, WIFI_Recv_CallBack);			// 注册网络数据接收成功的回调函数
	espconn_regist_disconcb((struct espconn *)arg,TCP_Disconnect_CallBack);	// 注册成功断开TCP连接的回调函数
	os_printf("TCP连接成功\r\n");
	OLED_ShowString(0,0,"TCP was successful connect!         ");

}

void ICACHE_FLASH_ATTR TCP_Break_CallBack(void *arg,sint8 err)
{
	os_printf("TCP异常断开\r\n");
	OLED_ShowString(0,0,"TCP was break!                  ");
	espconn_connect(&ST_NetCon);	// 连接TCP-server

}

//TCP初始化
void ICACHE_FLASH_ATTR TCP_Init()
{
	ST_NetCon.type = ESPCONN_TCP ;	 //设置为TCP协议
	ST_NetCon.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));	// 开辟内存

	//设置目标IP/端口(ESP8266作为Client，需要预先知道Server的IP/端口)
	//-------------------------------------------------------------------------
	ST_NetCon.proto.tcp->local_port = 8266 ;	// 设置本地端口
	ST_NetCon.proto.tcp->remote_port = 8080;	// 设置目标端口
	ST_NetCon.proto.tcp->remote_ip[0] = 192;	// 设置目标IP地址
	ST_NetCon.proto.tcp->remote_ip[1] = 168;
	ST_NetCon.proto.tcp->remote_ip[2] = 0;
	ST_NetCon.proto.tcp->remote_ip[3] = 105;

	//注册相关回调函数
	espconn_regist_connectcb(&ST_NetCon, TCP_Connect_CallBack);	// 注册TCP连接成功建立的回调函数
	espconn_regist_reconcb(&ST_NetCon, TCP_Break_CallBack);		// 注册TCP连接异常断开的回调函数
	OLED_ShowString(0,0,"TCP is connecting!           ");

	espconn_connect(&ST_NetCon);	// 连接TCP-server
}


//GPIO0中断,中断前不加Flash宏
void GPIO_INTERRUPT(void)
{
	uint32 GPIO_INT;
	uint32 GPIO_0_INT;
	uint32 GPIO_5_INT;

	GPIO_INT = GPIO_REG_READ(GPIO_STATUS_ADDRESS);   //读取标志位
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_INT);
	GPIO_0_INT = GPIO_INT & (0x01 << 0); //获取GPIO_0的中断标志位
	GPIO_5_INT = GPIO_INT & (0x01 << 5); //获取GPIO_0的中断标志位

	//判断按键是否松开
	if(GPIO_0_INT)
	{
		delay_ms(50);
		if (GPIO_INPUT_GET(GPIO_ID_PIN(0)) == 0)
			{
				LED_Flag = !LED_Flag;

				GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
				if(LED_Flag == 0)
				{
					os_printf("LED以打开\r\n");
					//OLED_ShowString(0,0,"The key to turn o the lights.      ");
					OLED_ShowString(0,6,"LED is on!   ");
				}
				else
				{
					os_printf("LED以关闭\r\n");
					//OLED_ShowString(0,0,"The key to turn off the lights.      ");
					OLED_ShowString(0,6,"LED is off!   ");
				}
			}
	}

	if(GPIO_5_INT)
	{
		delay_ms(50);
		if(GPIO_INPUT_GET(GPIO_ID_PIN(5)) == 1)
		{
			LED_Flag = 0;
			GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
			os_printf("光太暗了，打开灯\r\n");
			//OLED_ShowString(0,0,"The light is too weak. Turn on the light      ");
			OLED_ShowString(0,6,"LED is on!   ");
		}
		else
		{
			LED_Flag = 1;
			GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
			os_printf("光太亮了，关上灯\r\n");
			//OLED_ShowString(0,0,"The light is too strong. Turn off the light!    ");
			OLED_ShowString(0,6,"LED is off!   ");
		}
	}
}

//定时器回调函数
void ICACHE_FLASH_ATTR OS_Timer_CallBack(void)
{
	u8 C_LED_Flash = 0;

	struct ip_info ST_ESP8266_IP;	// ESP8266的IP信息
	u8 ESP8266_IP[4];	          //IP地址
	OLED_ShowString(4,0,"Connecting");
	if( wifi_station_get_connect_status() == STATION_GOT_IP )	// 判断是否获取IP
		{
			wifi_get_ip_info(STATION_IF,&ST_ESP8266_IP);	// 获取STA的IP信息
			ESP8266_IP[0] = ST_ESP8266_IP.ip.addr;			// IP地址高八位 == addr低八位
			ESP8266_IP[1] = ST_ESP8266_IP.ip.addr>>8;		// IP地址次高八位 == addr次低八位
			ESP8266_IP[2] = ST_ESP8266_IP.ip.addr>>16;		// IP地址次低八位 == addr次高八位
			ESP8266_IP[3] = ST_ESP8266_IP.ip.addr>>24;		// IP地址低八位 == addr高八位

			// 显示ESP8266的IP地址
			//-----------------------------------------------------------------------------------------------
			os_printf("ESP8266_IP = %d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2],ESP8266_IP[3]);
			//OLED_ShowIP(24,2,ESP8266_IP);	// OLED显示ESP8266的IP地址
			//-----------------------------------------------------------------------------------------------

			// 接入WIFI成功后，LED快闪3次
			//----------------------------------------------------
			for(; C_LED_Flash<=5; C_LED_Flash++)
			{
				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),(C_LED_Flash%2));
				delay_ms(100);
			}


			os_timer_disarm(&OS_Timer);	// 关闭定时器

			TCP_Init();		// 初始化网络连接(TCP通信)
		}
}

void ICACHE_FLASH_ATTR OS_Timer_Init(uint32 time_ms, uint8 time_repetitive)
{
	os_timer_disarm(&OS_Timer);  //关闭定时器

	os_timer_setfn(&OS_Timer, (os_timer_func_t *)OS_Timer_CallBack, NULL);

	os_timer_arm(&OS_Timer, time_ms, time_repetitive);
}

void ICACHE_FLASH_ATTR GPIO_Init(void)
{
	//GPIO0-Boot键，GPIO4-LED
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);  //初始化LED,GPIO_4
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);   //初始化按键，GPIO_0
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);    //初始化GPIO5

	ETS_GPIO_INTR_DISABLE();   //关闭GPIO的中断
	ETS_GPIO_INTR_ATTACH((ets_isr_t)GPIO_INTERRUPT, NULL);             //注册GPIO中断函数
	gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_NEGEDGE);     //设置GPIO_0中断源类型
	gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_ANYEDGE);     //设置GPIO_5中断源类型

	ETS_GPIO_INTR_ENABLE();   //开启GPIO的中断
}




void ICACHE_FLASH_ATTR
user_init(void)
{
	uart_init(115200,115200);   //初始化串口一和二的波特率
	delay_ms(10);
	os_printf("\r\n\r\n\r\n开始了！\r\n\r\n");

	//OLED初始化
	OLED_Init();// OLED初始化
	OLED_ShowString(0,0,"come on");
	delay_s(3);

	delay_s(10);
	GPIO_Init();

	STA_Init();

	OS_Timer_Init(1000,1);
}

