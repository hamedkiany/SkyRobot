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

static uint8_t Rxframe[MAX_FRAME_SIZE]; //Usar una global permite ahorrar pila en la tarea de RX.
static uint8_t Txframe[MAX_FRAME_SIZE]; //Usar una global permite ahorrar pila en las tareas, pero hay que tener cuidado al transmitir desde varias tareas!!!!
static uint32_t gRemoteProtocolErrors=0;

//Funciones internas
//Puntero a funcion callback que va a recibir los mensajes. Esta funcion se instala al llamar a remotelinkesp_init
static int32_t (* remotelinkesp_messageReceived)(uint8_t message_type, void *parameters, int32_t parameterSize);
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
        UARTCharPut(UART1_BASE, *str++);  // Env�a cada car�cter de la cadena
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


int32_t remotelinkesp_sendMessage(uint8_t message_type,void *parameter,int32_t paramsize)
{
    int32_t numdatos;

    numdatos=create_frame(Txframe,message_type,parameter,paramsize,MAX_FRAME_SIZE);
    if (numdatos>=0)
    {
        USBSerialWrite(Txframe,numdatos,portMAX_DELAY);
    }

    return numdatos;
}

static int32_t (* remotelinksp_messageReceived)(uint8_t message_type, void *parameters, int32_t parameterSize);
/* Recibe una trama (sin los caracteres de inicio y fin */
/* Utiliza la funcion bloqueante xSerialGetChar ---> Esta funcion es bloqueante y no se puede llamar desde una ISR !!!*/
static int32_t remotelinkesp_waitForMessage(uint8_t *frame, int32_t maxFrameSize)
{
    int32_t i;
    uint8_t rxByte;
    uint8_t *ptrtoframe=frame;
    do
    {
        //Elimino bytes de la cola de recepcion hasta llegar a comienzo de nueva trama
        USBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
    } while (rxByte!=START_FRAME_CHAR);

    i=0;
    do
    {
        USBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
        if (START_FRAME_CHAR==rxByte)
        {
            i=0;
            ptrtoframe=frame;
        }
        else {
            i++;
            *(ptrtoframe)=rxByte;
            ptrtoframe++;
        }
    } while ((rxByte!=STOP_FRAME_CHAR)&&(i<maxFrameSize));

    if (i==maxFrameSize)
    {
        return PROT_ERROR_RX_FRAME_TOO_LONG;    //Numero Negativo indicando error
    }
    else
    {
        if (i<(MINIMUN_FRAME_SIZE-START_SIZE))
        {
            return PROT_ERROR_BAD_SIZE; //La trama no tiene el tamanio minimo
        }
        else
        {
            return (i-END_SIZE); //Tamanio de la trama sin los bytes de inicio y fin
        }
    }
}

// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")
//Esta tarea decodifica los comandos y ejecuta la funci�n que los procesa
//Tambi�n gestiona posibles errores en la comunicacion
static portTASK_FUNCTION( remotelinkesp_Task, pvParameters ){

    int32_t numdatos;
    uint8_t command;
    void *ptrtoreceivedparam;
    int32_t error_status=0;

    /* The parameters are not used. (elimina el warning)*/
    ( void ) pvParameters;

    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        numdatos=remotelinkesp_waitForMessage(Rxframe,MAX_FRAME_SIZE); //Esta funcion es bloqueante

        if (numdatos>0)
        {
            //Si no ha habido errores recibiendo la trama, la intenta decodificar y procesar
            //primero se "desestufa" y se comprueba el checksum
            numdatos=destuff_and_check_checksum(Rxframe,numdatos);
            if (numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                gRemoteProtocolErrors++;
                // Procesamiento del error (TODO, POR HACER!!)
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo comando
                command=decode_command_type(Rxframe);
                //Obtiene un puntero al campo de parametros y su tamanio.
                numdatos=get_command_param_pointer(Rxframe,numdatos,&ptrtoreceivedparam);

                //Llamo a la funcion que procesa la orden que nos ha llegado desde el PC
                error_status=remotelinkesp_messageReceived(command,ptrtoreceivedparam,numdatos);

                //Una vez ejecutado, se comprueba si ha habido errores.
                switch(error_status)
                {
                    //Se procesar�an a continuaci�n
                    case PROT_ERROR_NOMEM:
                    {
                        // Procesamiento del error NO MEMORY
                        UARTprintf("Remotelink Error: not enough memory\n");
                    }
                    break;
                    case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                    {
                        // Procesamiento del error STUFFED_FRAME_TOO_LONG
                        UARTprintf("Remotelink Error: Frame too long. Cannot be created\n");
                    }
                    break;
                    case PROT_ERROR_COMMAND_TOO_LONG:
                    {
                        // Procesamiento del error COMMAND TOO LONG
                        UARTprintf("Remotelink Error: Packet too long. Cannot be allocated\n");
                    }
                    break;
                    case PROT_ERROR_INCORRECT_PARAM_SIZE:
                    {
                        // Procesamiento del error INCORRECT PARAM SIZE
                        UARTprintf("Remotelink Error: Incorrect parameter size\n");
                    }
                    break;
                    case PROT_ERROR_UNIMPLEMENTED_COMMAND:
                    {
                        MESSAGE_REJECTED_PARAMETER parametro;
                        parametro.command=command;
                        numdatos=remotelinkesp_sendMessage(MESSAGE_REJECTED,&parametro,sizeof(parametro));
                        UARTprintf("Remotelink Error: Unexpected command: %x\n",(uint32_t)command);
                        gRemoteProtocolErrors++;
                        //Aqui se podria, ademas, comprobar numdatos....
                    }
                    break;
                        //A�adir casos de error adicionales aqui...
                    default:
                        /* No hacer nada */
                    break;
                }
            }
        }
        else
        { // if (numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            gRemoteProtocolErrors++;
            // Procesamiento del error (TODO)
        }
    }
}


//Inicializa la tarea que recibe comandos e instala la callback para procesarlos.
BaseType_t remotelinkesp_init(int32_t remotelinkesp_stack, int32_t remotelinkesp_priority, int32_t (* message_receive_esp_callback)(uint8_t message_type, void *parameters, int32_t parameterSize))
{
    //USBSerialInit(32,32);   //Inicializo el  sistema USB-serie
    UART1_Init();

    //Instala la callback...
    remotelinkesp_messageReceived=message_receive_esp_callback;
    //
    // Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
    //
    return (xTaskCreate(remotelinkesp_Task, (portCHAR *)"remotelinkesp",remotelinkesp_stack, NULL, remotelinkesp_priority, NULL));
}
