//*****************************************************************************
//
// Codigo de  Practica 1.
// Main Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include <driverlib/PWMLib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

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
#include "commands.h"
#include "drivers/esp8266uart.h"

#include <remotelink.h>
#include <serialprotocol.h>

// parametros de funcionamiento de la tareas
#define REMOTELINK_TASK_STACK (512)
#define REMOTELINK_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define REMOTELINKesp_TASK_STACK (512)
#define REMOTELINKesp_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define ADC_TASK_STACK (512)
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define SW1TASKPRIO (tskIDLE_PRIORITY + 1)    // Prioridad para la tarea SW1TASK
#define SW1TASKSTACKSIZE (256)                // Tama�o de pila para la tarea SW1TASK
#define SW2TASKPRIO (tskIDLE_PRIORITY + 1)    // Prioridad para la tarea SW2TASK
#define SW2TASKSTACKSIZE (256)                // Tama�o de pila para la tarea SW2TASK
#define SW3TASKPRIO (tskIDLE_PRIORITY + 1)    // Prioridad para la tarea SW3TASK
#define SW3TASKSTACKSIZE (256)                // Tama�o de pila para la tarea SW3TASK
#define wheelLTASKPRIO (tskIDLE_PRIORITY + 1) // Prioridad para la tarea wheelTASK
#define wheelLTASKSTACKSIZE (256)             // Tama�o de pila para la tarea wheelTASK
#define wheelRTASKPRIO (tskIDLE_PRIORITY + 1) // Prioridad para la tarea wheelTASK
#define wheelRTASKSTACKSIZE (256)             // Tama�o de pila para la tarea wheelTASK

#define stateMachineTaskTASKPRIO (tskIDLE_PRIORITY + 1) // Prioridad para la tarea stateMachineTask
#define stateMachineTaskTASKSTACKSIZE (256)             // Tama�o de pila para la tarea stateMachineTask
// Globales
volatile uint32_t g_ui32CPUUsage;
volatile uint32_t g_ulSystemClock;
volatile uint32_t g_ulSystemClock2;

volatile int VelocidadF2 = 75, VelocidadF3 = 75, routcount = 0;
SemaphoreHandle_t miSemaforo, miSemaforo2, miSemaforo3, miSemaforo4, miSemaforo5;

volatile int motor1 = 0;
volatile int motor2 = 0;
volatile float x1 = 3;
volatile float x2 = 7;
volatile float x3 = 18;
volatile float x4 = 30;
volatile float R = 2.5;
volatile int L = 10;
volatile float whitecount = 0;
volatile int theta = 0;
volatile int realtheta = 0;
volatile int thetaR = 0;
volatile int realthetaR = 0;
volatile float thetatempF = 0;
volatile float RadioCircle = 20;
volatile     float thetatemp = 0;
// variable for state machine

// Define states and events
typedef enum
{
    STATE_IDLE,
    STATE_MAPPING,
    STATE_PROCESSING,
    STATE_ATTACK,
    STATE_ERROR
} State;

typedef enum
{
    EVENT_START,
    EVENT_STOP,
    EVENT_OBSTACLE_SEARCH,
    EVENT_OBSTACLE,
    EVENT_OBSTACLE_ZONE,
    EVENT_OBSTACLE_TARGET,
    EVENT_FRONT_WHITE,
    EVENT_BACK_WHITE,
    EVENT_ZONE_WHITE,
    EVENT_ERROR
} Event;
// Create a queue to hold events
QueueHandle_t eventQueue;

// Variables de posici�n actual
float posX = 0.0;
float posY = 0.0;
float angulo_actual = 0.0; // En grados

// Prototipos de funciones void mover(float cm); void girar(float grados);

// Funci�n para actualizar la posici�n del robot
void actualizarPosicion(float distancia)
{
    float angulo_radianes = angulo_actual * M_PI / 180.0;
    posX += distancia * cos(angulo_radianes);
    posY += distancia * sin(angulo_radianes);
}

// Funci�n para verificar si est� dentro del c�rculo
bool dentroDelCirculo()
{
    float distancia_centro = sqrt(posX * posX + posY * posY);
    return distancia_centro <= RadioCircle;
}

// Function to send an event to the queue
void sendEvent(Event event)
{
    if (eventQueue == NULL)
    {
        // Ensure the queue is created before using
        return;
    }

    if (xQueueSend(eventQueue, &event, portMAX_DELAY) != pdPASS)
    {
        // Handle the error, e.g., log an error message
    }
}

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while (1) // Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    {         // Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

// Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while (1)
    {
    }
}

// Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook(void)
{
    static uint8_t count = 0;

    if (++count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
    // return;
}

// Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook(void)
{
    SysCtlSleep();
}

// Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook(void)
{
    int i = 1;
    while (i)
        ;
}

int mover_robotM(int32_t c)
{

    thetatempF = c / R;
    thetatempF = (thetatempF * 6) / (2 * M_PI);
    if (c > 0)
    {

        while (abs(thetatempF) > realtheta)
        {
            forward();
        }
        stop();
        realtheta = 0;
    }
    if (c < 0)
    {
        while (abs(thetatempF) > realtheta)
        {
            rewind();
        }
        realtheta = 0;
        stop();
    }

    return 0;
}

int girar_robotM(int32_t g)
{

    thetatemp = (g * M_PI) / 180;
    thetatemp = (L / R) * thetatemp;
    thetatemp = (thetatemp * 3) / (2 * M_PI);
    if (g > 0)
    {
        while (abs(thetatemp) > (realtheta))
        {
            right();
        }
        realtheta = 0;
        stop();
    }
    if (g < 0)
    {
        while (abs(thetatemp) >= (realtheta))
        {
            left();
        }
        realtheta = 0;
        stop();
    }

    return 0;
}
int lazocerado()
{
    mover_robotM(12);
    SysCtlDelay(600000);

    girar_robotM(90);
    SysCtlDelay(600000);

    mover_robotM(18);
    SysCtlDelay(600000);

    girar_robotM(90);
    SysCtlDelay(600000);

    mover_robotM(12);
    SysCtlDelay(600000);

    girar_robotM(90);
    SysCtlDelay(600000);

    mover_robotM(18);
    SysCtlDelay(600000);

    girar_robotM(90);

    return 0;
}
//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

static portTASK_FUNCTION(ADCTask, pvParameters)
{
    MuestrasADCLive muestras;
    MESSAGE_ADC_SAMPLE_PARAMETER parameter;
    double distancia = 1110;
    Event event;
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while (1)
    {
        configADC_LiveADC(&muestras); // Espera y lee muestras del ADC (BLOQUEANTE)
        // Copia los datos en el parametro (es un poco redundante)
        parameter.chan1 = muestras.chan1;
        parameter.chan2 = muestras.chan2;
        parameter.chan3 = muestras.chan3;
        parameter.chan4 = muestras.chan4;
        parameter.chan5 = muestras.chan5;
        parameter.chan6 = muestras.chan6;
        parameter.chan7 = muestras.chan7;
        parameter.chan8 = muestras.chan8;
        if (parameter.chan2 < 3420 && parameter.chan2 > 899)
        {
            distancia = -(parameter.chan2 - 3614) / 179.08;
        }

        else if (parameter.chan2 < 900 && parameter.chan2 > 286)
        {
            distancia = -(parameter.chan2 - 1222.6) / 24.853;
        }
        else
        {
            distancia = 111111;
        }
        if (distancia >= x1 && distancia < x2)
        {
            event = EVENT_OBSTACLE_TARGET;
            sendEvent(event);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00000008);
        }
        else if (distancia >= x2 && distancia < x3)
        {
            event = EVENT_OBSTACLE_ZONE;
            sendEvent(event);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00000002);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00000002);
        }
        else if (distancia >= x3 && distancia <= x4)
        {
            event = EVENT_OBSTACLE;
            sendEvent(event);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00000002);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00000008);
        }
        else if (distancia >= x4 && distancia <= 2 * x4)
        {
            event = EVENT_OBSTACLE_SEARCH;
            sendEvent(event);

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
        }
        if (parameter.chan1 > 900 && parameter.chan3 > 900)
        {
            stop();
            event = EVENT_ZONE_WHITE;
            sendEvent(event);
        }
        else if (parameter.chan1 > 900 && parameter.chan3 < 900)
        {
            stop();
            event = EVENT_BACK_WHITE;
            sendEvent(event);
        }
        else if (parameter.chan1 < 900 && parameter.chan3 > 900)
        {
            stop();
            event = EVENT_FRONT_WHITE;
            sendEvent(event);
        }
        else
        {

        }
        // Encia el mensaje hacia QT
        // remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&parameter,sizeof(parameter));
    }
}

// static void Switch1Task(void *pvParameters)
static portTASK_FUNCTION(Switch1Task, pvParameters)
{

    xSemaphoreTake(miSemaforo, portMAX_DELAY);
    //
    // Loop forever.
    //
    while (1)
    {
        char *mensaje = ("S1\r\n");
        UART1_SendString(mensaje); // Env�a el mensaje
        // lazocerado();
        xSemaphoreTake(miSemaforo, portMAX_DELAY);
        //        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
    }
}

static portTASK_FUNCTION(Switch2Task, pvParameters)
{
    xSemaphoreTake(miSemaforo2, portMAX_DELAY);
    //
    // Loop forever.
    //
    Event event;
    while (1)
    {
        char *mensaje = ("S2\r\n");
        UART1_SendString(mensaje); // Env�a el mensaje
        event = EVENT_START;
        sendEvent(event);

        xSemaphoreTake(miSemaforo2, portMAX_DELAY);
        //        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
    }
}

static portTASK_FUNCTION(Switch3Task, pvParameters)
{
    xSemaphoreTake(miSemaforo3, portMAX_DELAY);
    //
    // Loop forever.
    //
    while (1)
    {
        //        configADC_DisparaADC(); //Dispara la conversion (por software)
        char *mensaje = ("line\r\n");
        UART1_SendString(mensaje); // Env�a el mensaje
        stop();
        // lazocerado();
        xSemaphoreTake(miSemaforo3, portMAX_DELAY);
        //        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
    }
}

static portTASK_FUNCTION(wheelLTask, pvParameters)
{
    xSemaphoreTake(miSemaforo4, portMAX_DELAY);
    //
    // Loop forever.
    //
    while (1)
    {
        theta = 0;
        realtheta = realtheta + 1;
        xSemaphoreTake(miSemaforo4, portMAX_DELAY);
    }
}

static portTASK_FUNCTION(wheelRTask, pvParameters)
{
    xSemaphoreTake(miSemaforo5, portMAX_DELAY);
    //
    // Loop forever.
    //
    while (1)
    {
        thetaR = 0;
        realthetaR = realthetaR + 1;
        xSemaphoreTake(miSemaforo5, portMAX_DELAY);
    }
}
// State machine task
void stateMachineTask(void *pvParameters)
{
    State currentState = STATE_IDLE;
    Event currentEvent;
    char *mensaje = " ";
    int Obsta = 0, ObstAt = 0;
    for (;;)
    {
        // Wait for an event
        if (xQueueReceive(eventQueue, &currentEvent, portMAX_DELAY) == pdPASS)
        {
            switch (currentState)
            {
            case STATE_IDLE:
                switch (currentEvent)
                {
                case EVENT_START:
                    mensaje = ("Mapping");
                    UART1_SendString("Mapping\r\n"); // Env�a el mensaje
                    mover_robotM(RadioCircle); // voy a centro
                    sendEvent(EVENT_START);
                    currentState = STATE_MAPPING; // STATE_PROCESSING;
                    break;
                case EVENT_ERROR:
                    // Transition to error state
                    currentState = STATE_ERROR;
                    break;
                default:
                    // sendEvent(EVENT_START);
                    break;
                }
                break;
            case STATE_MAPPING:
                switch (currentEvent)
                {
                case EVENT_START:
                    // Transition to processing state
                    mensaje = ("girar360");
                    UART1_SendString("girar360\r\n"); // Env�a el mensaje

                    girar_robotM(180);
                    sendEvent(EVENT_OBSTACLE_SEARCH);
                    currentState = STATE_PROCESSING; // STATE_PROCESSING;
                    // Perform actions for the new state
                    break;
                case EVENT_ERROR:
                    // Transition to error state
                    currentState = STATE_ERROR;
                    break;
                default:
                    //sendEvent(EVENT_START);

                    break;
                }
                break;
            case STATE_PROCESSING:
                switch (currentEvent)
                {
                case EVENT_OBSTACLE_SEARCH:
                    mensaje = ("girar360");
                    UART1_SendString("EVENT_OBSTACLE_SEARCH\r\n"); // Env�a el mensaje

                    girar_robotM(180);
                    mover_robotM(10);
                    //sendEvent(EVENT_OBSTACLE_SEARCH);
                    //currentState = STATE_PROCESSING; // STATE_PROCESSING;
                    break;
                case EVENT_OBSTACLE:
                    // Transition to idle state

                    Obsta = Obsta + 1;
                    if (Obsta == 2)
                    {
                        actualizarPosicion(10);
                        mover_robotM(10);
                        mensaje = ("mover 10 pro");
                        UART1_SendString("EVENT_OBSTACLE\r\n"); // Env�a el mensaje
                        Obsta = 0;
                    }
                    //currentState = STATE_PROCESSING;
                    break;
                case EVENT_OBSTACLE_ZONE:
                    // Transition to error state
                    actualizarPosicion(5);
                    mover_robotM(5);
                    mensaje = ("mover 5 pro");
                    UART1_SendString("mover 5 pro\r\n"); // Env�a el mensaje

                    //currentState = STATE_PROCESSING;
                    break;
                case EVENT_FRONT_WHITE:
                    // Transition to idle state
                    actualizarPosicion(-5);
                    mover_robotM(-5);
                    mensaje = ("mover -5 pro");
                    UART1_SendString("mover -5 pro\r\n"); // Env�a el mensaje
                    //sendEvent(EVENT_OBSTACLE_SEARCH);
                    //currentState = STATE_PROCESSING;
                    break;
                case EVENT_BACK_WHITE:
                    // Transition to error state
                    actualizarPosicion(10);
                    mover_robotM(10);
                    mensaje = ("mover 5 2 pro");
                    UART1_SendString("mover 5 2 pro\r\n"); // Env�a el mensaje
                    //sendEvent(EVENT_OBSTACLE_SEARCH);
                    //currentState = STATE_PROCESSING;
                    break;
                case EVENT_ZONE_WHITE:
                    // Transition to error state
                    stop();
                    girar_robotM(-20);
                    mensaje = ("girar 45  pro");
                    UART1_SendString("girar 45  pro\r\n"); // Env�a el mensaje
                    //mover_robotM(5);
                    //sendEvent(EVENT_OBSTACLE_SEARCH);
                    //currentState = STATE_PROCESSING;
                    break;
                case EVENT_OBSTACLE_TARGET:
                    sendEvent(EVENT_OBSTACLE_TARGET);
                    currentState = STATE_ATTACK;

                    break;

                default:
                     //sendEvent(EVENT_OBSTACLE_SEARCH);
                     break;
                }
                break;

            case STATE_ATTACK:
                switch (currentEvent)
                {
                case EVENT_OBSTACLE_TARGET:
                    // Transition to processing state
                    forward();
                    ObstAt = ObstAt + 1;
                    if (ObstAt == 4)
                    {
                        sendEvent(EVENT_OBSTACLE_SEARCH);
                        currentState = STATE_PROCESSING;
                        ObstAt = 0;
                    }
                    //currentState = STATE_ATTACK; // STATE_ATTACK;
                    // Perform actions for the new state
                    break;
                case EVENT_FRONT_WHITE:
                    // Transition to idle state
                    actualizarPosicion(-5);
                    mover_robotM(-5);
                    //sendEvent(EVENT_OBSTACLE_SEARCH);

                    currentState = STATE_PROCESSING;
                    break;
                case EVENT_BACK_WHITE:
                    // Transition to error state
                    actualizarPosicion(10);
                    mover_robotM(10);
                    //sendEvent(EVENT_OBSTACLE_SEARCH);

                    currentState = STATE_PROCESSING;
                    break;
                case EVENT_ZONE_WHITE:
                    // Transition to error state
                    stop();
                    girar_robotM(30);
                    //sendEvent(EVENT_OBSTACLE_SEARCH);
                    //mover_robotM(10);
                    currentState = STATE_PROCESSING;
                    break;

                case EVENT_ERROR:
                    // Transition to error state
                    currentState = STATE_ERROR;
                    break;
                default:
                    // sendEvent(EVENT_START);
                    // currentState = STATE_IDLE;

                    break;
                }
                break;

            case STATE_ERROR:
                switch (currentEvent)
                {
                case EVENT_STOP:
                    // Attempt recovery, transition to idle
                    stop();
                    sendEvent(EVENT_START);
                    currentState = STATE_IDLE;
                    break;
                default:
                    // Stay in error state
                    sendEvent(EVENT_START);
                    currentState = STATE_IDLE;

                    break;
                }
                break;

            default:
                // Handle unexpected state
//                sendEvent(EVENT_START);
//                currentState = STATE_IDLE;
                break;
            }
        }
    }
}

// Funcion callback que procesa los mensajes recibidos desde el PC (ejecuta las acciones correspondientes a las ordenes recibidas)
static int32_t messageReceived(uint8_t message_type, void *parameters, int32_t parameterSize)
{
    int32_t status = 0; // Estado de la ejecucion (positivo, sin errores, negativo si error)

    // Comprueba el tipo de mensaje
    switch (message_type)
    {
    case MESSAGE_PING:
    {
        status = remotelink_sendMessage(MESSAGE_PING, NULL, 0);
    }
    break;
    case MESSAGE_LED_GPIO:
    {
        MESSAGE_LED_GPIO_PARAMETER parametro;

        if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro)) > 0)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, parametro.value);
        }
        else
        {
            status = PROT_ERROR_INCORRECT_PARAM_SIZE; // Devuelve un error
        }
    }
    break;
    case MESSAGE_MOTOR:
    {
        MESSAGE_MOTOR_PARAMETER parametro;

        if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro)) > 0)
        {
        }
        else
        {
            status = PROT_ERROR_INCORRECT_PARAM_SIZE; // Devuelve un error
        }
    }
    break;

    case MESSAGE_ADC_SAMPLE:
    {
        configADC_DisparaADC(); // Dispara la conversion (por software)
    }
    break;
    default:
        // mensaje desconocido/no implementado
        status = PROT_ERROR_UNIMPLEMENTED_COMMAND; // Devuelve error.
    }

    return status; // Devuelve status
}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************

int main(void)
{

    //
    // Set the clocking to run at 50 MHz from the PLL.
    //
    //	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
    //			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 5)
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Get the system clock speed.
    g_ulSystemClock = SysCtlClockGet();

    // Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                         deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ / 10, 3);
    // Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1
    RGBInit(1);
    MAP_SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH); // Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Volvemos a configurar los LEDs en modo GPIO POR Defecto
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3);
    // MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ButtonsInit();
    MAP_IntPrioritySet(INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY); // para a�adir prioridad by HAMED
    MAP_GPIOIntEnable(GPIO_PORTF_BASE, ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);
    // para port A
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_DIR_MODE_IN);
    // MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    MAP_IntPrioritySet(INT_GPIOA, configMAX_SYSCALL_INTERRUPT_PRIORITY); // para a�adir prioridad by HAMED
    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_BOTH_EDGES);

    MAP_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    MAP_IntEnable(INT_GPIOA);
    // para port B
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_IN);
    // MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    MAP_IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY); // para a�adir prioridad by HAMED
//    MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2,
//                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_RISING_EDGE);
    MAP_GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    MAP_IntEnable(INT_GPIOB);

    miSemaforo = xSemaphoreCreateBinary();
    miSemaforo2 = xSemaphoreCreateBinary();
    miSemaforo3 = xSemaphoreCreateBinary();
    miSemaforo4 = xSemaphoreCreateBinary();
    miSemaforo5 = xSemaphoreCreateBinary();

    /********************************      Creacion de tareas *********************/

    // Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK, COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while (1)
            ;
    }

    // Esta funcion crea internamente una tarea para las comunicaciones USB.
    // Ademas, inicializa el USB y configura el perfil USB-CDC
    if (remotelink_init(REMOTELINK_TASK_STACK, REMOTELINK_TASK_PRIORITY, messageReceived) != pdTRUE)
    {
        while (1)
            ; // Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero remotelink.c
    }
//    if (remotelink_init(REMOTELINKesp_TASK_STACK, REMOTELINK_TASK_PRIORITY, messageReceived) != pdTRUE)
//    {
//        while (1)
//            ; // Inicializo la aplicacion de comunicacion con el esp (Remote). Ver fichero esp8266uart.c
//    }
//    // Para especificacion 2: Inicializa el ADC y crea una tarea...
    // configADC_IniciaADC();
    configADC_Timer();
    if ((xTaskCreate(ADCTask, (portCHAR *)"ADC", ADC_TASK_STACK, NULL, ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while (1)
            ;
    }

    if ((xTaskCreate(Switch1Task, (portCHAR *)"Sw1", SW1TASKSTACKSIZE, NULL, SW1TASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }
    if ((xTaskCreate(Switch2Task, (portCHAR *)"Sw2", SW2TASKSTACKSIZE, NULL, SW2TASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }
    if ((xTaskCreate(Switch3Task, (portCHAR *)"Sw3", SW3TASKSTACKSIZE, NULL, SW3TASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }
    activatePWM(75, 75);
    UART1_Init();

    if ((xTaskCreate(wheelLTask, (portCHAR *)"wheelL", wheelLTASKSTACKSIZE, NULL, wheelLTASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }
    if ((xTaskCreate(wheelRTask, (portCHAR *)"wheelR", wheelRTASKSTACKSIZE, NULL, wheelRTASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }
    eventQueue = xQueueCreate(10, sizeof(Event));
    if ((xTaskCreate(stateMachineTask, (portCHAR *)"wheelR", stateMachineTaskTASKSTACKSIZE, NULL, stateMachineTaskTASKPRIO, NULL) != pdTRUE))
    {
        while (1)
            ;
    }

    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    vTaskStartScheduler(); // el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
    // De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

    while (1)
    {
        // Si llego aqui es que algo raro ha pasado
    }
}

// Rutinas de interrupcion
void GPIOFIntHandler(void)
{
    // Lee el estado del puerto (activos a nivel bajo)
    // Event event;

    int32_t i32PinStatus = MAP_GPIOPinRead(GPIO_PORTF_BASE, ALL_BUTTONS | GPIO_PIN_3);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOF);
    int ui8Delay = 0;

    for (ui8Delay = 0; ui8Delay < 32; ui8Delay++)
    {
    }
    if (!(i32PinStatus & LEFT_BUTTON))
    {

        xSemaphoreGiveFromISR(miSemaforo, &xHigherPriorityTaskWoken);
    }

    if (!(i32PinStatus & RIGHT_BUTTON))
    {
        // Send a START event
        //        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
        // Delay for demonstration
        //
        xSemaphoreGiveFromISR(miSemaforo2, &xHigherPriorityTaskWoken);
    }

    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS | GPIO_PIN_4);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void counterroute(void)
{

    int32_t i32PinStatus = MAP_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOA);
    int ui8Delay = 0;
    for (ui8Delay = 0; ui8Delay < 16; ui8Delay++)
    {
    }

    if (!(i32PinStatus & GPIO_PIN_3))
    {
        theta = 1;
        xSemaphoreGiveFromISR(miSemaforo4, &xHigherPriorityTaskWoken);
    }

    if (!(i32PinStatus & GPIO_PIN_2))
    {
        thetaR = 1;
        xSemaphoreGiveFromISR(miSemaforo5, &xHigherPriorityTaskWoken);
    }
    MAP_IntEnable(INT_GPIOA);
    MAP_GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void esp32(void)
{
    Event event;
    int32_t i32PinStatus = MAP_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOB);
////    MAP_GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);
    int ui8Delay = 0;
    for (ui8Delay = 0; ui8Delay < 160; ui8Delay++)
    {
    }

    if ((i32PinStatus & GPIO_PIN_2))
    {
        event = EVENT_FRONT_WHITE;

        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);

        xSemaphoreGiveFromISR(miSemaforo3, &xHigherPriorityTaskWoken);
    }
    if((i32PinStatus & GPIO_PIN_3))
    {
        event = EVENT_BACK_WHITE;
        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(miSemaforo3, &xHigherPriorityTaskWoken);
    }
    MAP_IntEnable(INT_GPIOB);
    MAP_GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
