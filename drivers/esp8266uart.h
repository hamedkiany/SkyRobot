/*
 * esp8266uart.h
 *
 *  Created on: 11/11/2024
 *      Author: Hamed
 */

#ifndef ESP8266UART_H_
#define ESP8266UART_H_

#include<stdbool.h>
#include<stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "drivers/rgb.h"
#include "drivers/configADC.h"
#include <remotelink.h>
#include <serialprotocol.h>

void UART1_Init(void);

void UART1_SendString(const char *str) ;
void UART1_SendBytes(const uint8_t *data, uint32_t sized);
void UART1_SendStruct(uint8_t message_type,void *parameter,int32_t paramsize);
int32_t remotelinkesp_sendMessage(uint8_t message_type,void *parameter,int32_t paramsize);
static int32_t (* remotelinksp_messageReceived)(uint8_t message_type, void *parameters, int32_t parameterSize);
static int32_t remotelinkesp_waitForMessage(uint8_t *frame, int32_t maxFrameSize);
static portTASK_FUNCTION( remotelinkesp_Task, pvParameters );
BaseType_t remotelinkesp_init(int32_t remotelinkesp_stack, int32_t remotelinkesp_priority, int32_t (* message_receive_esp_callback)(uint8_t message_type, void *parameters, int32_t parameterSize));
#endif /* ESP8266uart_H_ */
