#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


//Funciones internas
void UART1_Init(void) {
    // Habilita el reloj para el puerto B y UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);

    // Configura PB0 y PB1 como pines UART
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configura UART1 para una velocidad de 115200 baudios, 8 bits de datos, sin paridad, 1 bit de parada
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void UART1_SendString(const char *str) {
    while (*str) {
        UARTCharPut(UART1_BASE, *str++);  // Envía cada carácter de la cadena
    }
}

void UART1_SendBytes(const uint8_t *data, uint32_t sized) {
    uint32_t i = 0;
    for (i = 0; i < sized; i++) {
        UARTCharPut(UART1_BASE, data[i]);  // Send each byte of the struct
    }
}
void UART1_SendStruct(uint8_t message_type,void *parameter,int32_t paramsize) {
//    int32_t numdatos;
//
//    numdatos=create_frame(Txframe,message_type,parameter,paramsize,MAX_FRAME_SIZE);
//    if (numdatos>=0)
//    {
//        UART1_SendBytes(Txframe,numdatos);//,portMAX_DELAY);
//    }

    UART1_SendBytes((uint8_t *)parameter, paramsize);  // Cast struct to byte array
}
//Puntero a funcion callback que va a recibir los mensajes. Esta funcion se instala al llamar a remotelink_init
static int32_t (* remotelink_esp_messageReceived)(uint8_t message_type, void *parameters, int32_t parameterSize);
static int32_t remotelinkesp_waitForMessage(uint8_t *frame, int32_t maxFrameSize){

}
int32_t remotelinkesp_sendMessage(uint8_t message_type,void *parameter,int32_t paramsize){

}

static portTASK_FUNCTION( remotelinkesp_Task, pvParameters ){

}
//Inicializa la tarea que recibe comandos e instala la callback para procesarlos.
BaseType_t remotelinkesp_init(int32_t remotelinkesp_stack, int32_t remotelinkesp_priority, int32_t (* esp_message_receive_callback)(uint8_t message_type, void *parameters, int32_t parameterSize))
{
       //Inicializo el  sistema USB-serie


    //Instala la callback...
    remotelink_esp_messageReceived=esp_message_receive_callback;
    //
    // Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
    //
    return (xTaskCreate(remotelinkesp_Task, (portCHAR *)"remotelinkesp",remotelinkesp_stack, NULL, remotelinkesp_priority, NULL));
}
