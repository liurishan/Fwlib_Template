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

#include "ets_sys.h"   //�ص�����
#include "osapi.h"    //�����ʱ��
#include "eagle_soc.h"			// GPIO�������궨��
#include "os_type.h"			// os_XXX
#include "user_interface.h" 	// ϵͳ�ӿڡ�system_param_xxx�ӿڡ�WIFI��RateContro
#include "driver/uart.h"
#include "driver/my_delay.h"
#include "mem.h"				// �ڴ�����Ⱥ���
#include "c_types.h"			// ��������
#include "ip_addr.h"			// ��"espconn.h"ʹ�á���"espconn.h"��ͷ#include"ip_addr.h"��#include"ip_addr.h"����"espconn.h"֮ǰ
#include "espconn.h"			// TCP/UDP�ӿ�
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

//�궨��



//ȫ�ֱ����Ķ���
int LED_Flag = 0;
os_timer_t OS_Timer;
os_event_t * Pointer_Task ;  //���崮�ڴ�ӡ����ָ��
struct espconn ST_NetCon;     //�������ӽṹ��



//STA��ʼ��
void ICACHE_FLASH_ATTR STA_Init()
{
	struct station_config STA_Config;   //STA�����ṹ��
	struct ip_info ST_IP;               //STA��Ϣ�ṹ��

	wifi_set_opmode(0x01);   //����ΪSTAģʽ�����һᱣ�浽Flash

	//Ĭ�Ͽ���DHCP Client�����Զ�����IP

	//����STA����
	os_memset(&STA_Config, 0, sizeof(struct station_config));	// STA�����ṹ������
	os_strcpy(STA_Config.ssid,wifi_name);				// ����WIFI��
	os_strcpy(STA_Config.password, wifi_password);			// ����WIFI����

	wifi_station_set_config(&STA_Config);    //����STA�������浽Flash
}



//TCP�ɹ�����ص�����
void ICACHE_FLASH_ATTR WIFI_Send_CallBack(void *arg)
{
	//os_printf("\nESP8266_WIFI_Send_OK\n");
}

//TCP�ɹ����ջص����� ,����1�����紫��ṹ��espconnָ�롢����2�����紫������ָ�롢����3�����ݳ���
void ICACHE_FLASH_ATTR WIFI_Recv_CallBack(void * arg, char * pdata, unsigned short len)
{
	struct espconn * T_arg = arg;		// �����������ӽṹ��ָ��

	if(pdata[0] == 'o')
	{
		LED_Flag = 0;
		GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
		os_printf("���Ա�Զ�̴�\r\n");
		//OLED_ShowString(0,0,"The phone to turn on the lights.    ");
		OLED_ShowString(0,6,"LED is on!   ");
	}
	else
	{
		LED_Flag = 1;
		GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
		os_printf("���Ա�Զ�̹ر�\r\n");
		//OLED_ShowString(0,0,"The phone to turn off the lights.    ");
		OLED_ShowString(0,6,"LED is off!   ");
	}
	//OLED_ShowString(0,0,"Client is successful receipt!         ");


	espconn_send(T_arg,"ESP8266�ɹ����գ�\r\n",os_strlen("ESP8266�ɹ����գ�\r\n"));	// ��Է�����Ӧ��
}

//TCP�����Ͽ��ص�����
void ICACHE_FLASH_ATTR TCP_Disconnect_CallBack(void *arg)
{
	os_printf("\nESP8266_TCP_Disconnect_OK\n");
	OLED_ShowString(0,0,"TCP was successful disconnect!         ");
}

//TCP�ɹ����ӻص�����
void ICACHE_FLASH_ATTR TCP_Connect_CallBack(void *arg)
{
	espconn_regist_sentcb((struct espconn *)arg, WIFI_Send_CallBack);			// ע���������ݷ��ͳɹ��Ļص�����
	espconn_regist_recvcb((struct espconn *)arg, WIFI_Recv_CallBack);			// ע���������ݽ��ճɹ��Ļص�����
	espconn_regist_disconcb((struct espconn *)arg,TCP_Disconnect_CallBack);	// ע��ɹ��Ͽ�TCP���ӵĻص�����
	os_printf("TCP���ӳɹ�\r\n");
	OLED_ShowString(0,0,"TCP was successful connect!         ");

}

void ICACHE_FLASH_ATTR TCP_Break_CallBack(void *arg,sint8 err)
{
	os_printf("TCP�쳣�Ͽ�\r\n");
	OLED_ShowString(0,0,"TCP was break!                  ");
	espconn_connect(&ST_NetCon);	// ����TCP-server

}

//TCP��ʼ��
void ICACHE_FLASH_ATTR TCP_Init()
{
	ST_NetCon.type = ESPCONN_TCP ;	 //����ΪTCPЭ��
	ST_NetCon.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));	// �����ڴ�

	//����Ŀ��IP/�˿�(ESP8266��ΪClient����ҪԤ��֪��Server��IP/�˿�)
	//-------------------------------------------------------------------------
	ST_NetCon.proto.tcp->local_port = 8266 ;	// ���ñ��ض˿�
	ST_NetCon.proto.tcp->remote_port = 8080;	// ����Ŀ��˿�
	ST_NetCon.proto.tcp->remote_ip[0] = 192;	// ����Ŀ��IP��ַ
	ST_NetCon.proto.tcp->remote_ip[1] = 168;
	ST_NetCon.proto.tcp->remote_ip[2] = 0;
	ST_NetCon.proto.tcp->remote_ip[3] = 105;

	//ע����ػص�����
	espconn_regist_connectcb(&ST_NetCon, TCP_Connect_CallBack);	// ע��TCP���ӳɹ������Ļص�����
	espconn_regist_reconcb(&ST_NetCon, TCP_Break_CallBack);		// ע��TCP�����쳣�Ͽ��Ļص�����
	OLED_ShowString(0,0,"TCP is connecting!           ");

	espconn_connect(&ST_NetCon);	// ����TCP-server
}


//GPIO0�ж�,�ж�ǰ����Flash��
void GPIO_INTERRUPT(void)
{
	uint32 GPIO_INT;
	uint32 GPIO_0_INT;
	uint32 GPIO_5_INT;

	GPIO_INT = GPIO_REG_READ(GPIO_STATUS_ADDRESS);   //��ȡ��־λ
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_INT);
	GPIO_0_INT = GPIO_INT & (0x01 << 0); //��ȡGPIO_0���жϱ�־λ
	GPIO_5_INT = GPIO_INT & (0x01 << 5); //��ȡGPIO_0���жϱ�־λ

	//�жϰ����Ƿ��ɿ�
	if(GPIO_0_INT)
	{
		delay_ms(50);
		if (GPIO_INPUT_GET(GPIO_ID_PIN(0)) == 0)
			{
				LED_Flag = !LED_Flag;

				GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
				if(LED_Flag == 0)
				{
					os_printf("LED�Դ�\r\n");
					//OLED_ShowString(0,0,"The key to turn o the lights.      ");
					OLED_ShowString(0,6,"LED is on!   ");
				}
				else
				{
					os_printf("LED�Թر�\r\n");
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
			os_printf("��̫���ˣ��򿪵�\r\n");
			//OLED_ShowString(0,0,"The light is too weak. Turn on the light      ");
			OLED_ShowString(0,6,"LED is on!   ");
		}
		else
		{
			LED_Flag = 1;
			GPIO_OUTPUT_SET(GPIO_ID_PIN(4), LED_Flag);
			os_printf("��̫���ˣ����ϵ�\r\n");
			//OLED_ShowString(0,0,"The light is too strong. Turn off the light!    ");
			OLED_ShowString(0,6,"LED is off!   ");
		}
	}
}

//��ʱ���ص�����
void ICACHE_FLASH_ATTR OS_Timer_CallBack(void)
{
	u8 C_LED_Flash = 0;

	struct ip_info ST_ESP8266_IP;	// ESP8266��IP��Ϣ
	u8 ESP8266_IP[4];	          //IP��ַ
	OLED_ShowString(4,0,"Connecting");
	if( wifi_station_get_connect_status() == STATION_GOT_IP )	// �ж��Ƿ��ȡIP
		{
			wifi_get_ip_info(STATION_IF,&ST_ESP8266_IP);	// ��ȡSTA��IP��Ϣ
			ESP8266_IP[0] = ST_ESP8266_IP.ip.addr;			// IP��ַ�߰�λ == addr�Ͱ�λ
			ESP8266_IP[1] = ST_ESP8266_IP.ip.addr>>8;		// IP��ַ�θ߰�λ == addr�εͰ�λ
			ESP8266_IP[2] = ST_ESP8266_IP.ip.addr>>16;		// IP��ַ�εͰ�λ == addr�θ߰�λ
			ESP8266_IP[3] = ST_ESP8266_IP.ip.addr>>24;		// IP��ַ�Ͱ�λ == addr�߰�λ

			// ��ʾESP8266��IP��ַ
			//-----------------------------------------------------------------------------------------------
			os_printf("ESP8266_IP = %d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2],ESP8266_IP[3]);
			//OLED_ShowIP(24,2,ESP8266_IP);	// OLED��ʾESP8266��IP��ַ
			//-----------------------------------------------------------------------------------------------

			// ����WIFI�ɹ���LED����3��
			//----------------------------------------------------
			for(; C_LED_Flash<=5; C_LED_Flash++)
			{
				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),(C_LED_Flash%2));
				delay_ms(100);
			}


			os_timer_disarm(&OS_Timer);	// �رն�ʱ��

			TCP_Init();		// ��ʼ����������(TCPͨ��)
		}
}

void ICACHE_FLASH_ATTR OS_Timer_Init(uint32 time_ms, uint8 time_repetitive)
{
	os_timer_disarm(&OS_Timer);  //�رն�ʱ��

	os_timer_setfn(&OS_Timer, (os_timer_func_t *)OS_Timer_CallBack, NULL);

	os_timer_arm(&OS_Timer, time_ms, time_repetitive);
}

void ICACHE_FLASH_ATTR GPIO_Init(void)
{
	//GPIO0-Boot����GPIO4-LED
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);  //��ʼ��LED,GPIO_4
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);   //��ʼ��������GPIO_0
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);    //��ʼ��GPIO5

	ETS_GPIO_INTR_DISABLE();   //�ر�GPIO���ж�
	ETS_GPIO_INTR_ATTACH((ets_isr_t)GPIO_INTERRUPT, NULL);             //ע��GPIO�жϺ���
	gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_NEGEDGE);     //����GPIO_0�ж�Դ����
	gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_ANYEDGE);     //����GPIO_5�ж�Դ����

	ETS_GPIO_INTR_ENABLE();   //����GPIO���ж�
}




void ICACHE_FLASH_ATTR
user_init(void)
{
	uart_init(115200,115200);   //��ʼ������һ�Ͷ��Ĳ�����
	delay_ms(10);
	os_printf("\r\n\r\n\r\n��ʼ�ˣ�\r\n\r\n");

	//OLED��ʼ��
	OLED_Init();// OLED��ʼ��
	OLED_ShowString(0,0,"come on");
	delay_s(3);

	delay_s(10);
	GPIO_Init();

	STA_Init();

	OS_Timer_Init(1000,1);
}

