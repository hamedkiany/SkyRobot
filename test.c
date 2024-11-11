//*****************************************************************************
//
// Codigo de  Practica 1.
// Main Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include <driverlib/PWMLib.h>
#include<stdbool.h>
#include<stdint.h>
#include<math.h>

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


//parametros de funcionamiento de la tareas
#define REMOTELINK_TASK_STACK (512)
#define REMOTELINK_TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define REMOTELINKesp_TASK_STACK (512)
#define REMOTELINKesp_TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define ADC_TASK_STACK (512)
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define SW1TASKPRIO (tskIDLE_PRIORITY+1)            // Prioridad para la tarea SW1TASK
#define SW1TASKSTACKSIZE (256)     // Tamaño de pila para la tarea SW1TASK
#define SW2TASKPRIO (tskIDLE_PRIORITY+1)            // Prioridad para la tarea SW2TASK
#define SW2TASKSTACKSIZE (256)     // Tamaño de pila para la tarea SW2TASK
#define SW3TASKPRIO (tskIDLE_PRIORITY+1)            // Prioridad para la tarea SW3TASK
#define SW3TASKSTACKSIZE (256)     // Tamaño de pila para la tarea SW3TASK
#define wheelTASKPRIO (tskIDLE_PRIORITY+1)            // Prioridad para la tarea wheelTASK
#define wheelTASKSTACKSIZE (256)     // Tamaño de pila para la tarea wheelTASK

//Globales
volatile uint32_t g_ui32CPUUsage;
volatile uint32_t g_ulSystemClock;
volatile uint32_t g_ulSystemClock2;

int VelocidadF2 = 75 , VelocidadF3 = 75,routcount = 0;
SemaphoreHandle_t miSemaforo,miSemaforo2,miSemaforo3,miSemaforo4;
float x = 0.5;  // Valor X del joystick
float y = 0.3;  // Valor Y del joystick

int motor1 = 0;
int motor2 = 0;
float x1 = 5;
float x2 = 10;
float x3 = 15;
float x4 = 20;
float R = 2.5;
int L = 10;
volatile float whitecount = 0;
volatile int theta = 0;
volatile int realtheta=0;
volatile float thetatempF = 0;


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
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t count = 0;

    if (++count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
    //return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
    SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
    while(1);
}



int mover_robotM(int32_t c)
{
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);           // Habilitar el módulo de Timer0
//    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);     // Configurar como temporizador de cuenta ascendente de 32 bits
//    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() - 1); // Configurar para que cuente cada segundo
//    TimerEnable(TIMER0_BASE, TIMER_A);                      // Habilitar el Timer0

    //D = (R * (thetaRight+thetaLeft)/2)
    thetatempF = c / R ;
    thetatempF = (thetatempF * 6) / (2 * M_PI) ;
    if(c > 0){
//    uint32_t g_ulSystemClock3  = TimerValueGet(TIMER0_BASE, TIMER_A);

    while(abs(thetatempF) > realtheta)
    {
        forward();
    }
     stop();
    realtheta = 0;
    }
    if(c < 0){
    while(abs(thetatempF) > realtheta)
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

    //theta = R/l (tethaLeft - tethaRight) ** que l es destancia de las reudas
    float thetatemp = 0 ;
    thetatemp = (g * M_PI) / 180 ;
    thetatemp = (L/R) * thetatemp;
    thetatemp = (thetatemp * 6) / (2 * M_PI) ;
    if(g > 0){
    while(abs(thetatemp) > (realtheta))
    {
        right();
    }
    realtheta = 0;
    stop();
    }
    if(g < 0){
    while(abs(thetatemp) >= (realtheta ))
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


//Para especificacion 2. Esta tarea no tendria por que ir en main.c
static portTASK_FUNCTION(ADCTask,pvParameters)
{

    MuestrasADCLive muestras;
    MESSAGE_ADC_SAMPLE_PARAMETER parameter;
    double distancia = 1110 ;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        configADC_LiveADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        //Copia los datos en el parametro (es un poco redundante)
        parameter.chan1=muestras.chan1;
        parameter.chan2=muestras.chan2;
        parameter.chan3=muestras.chan3;
        parameter.chan4=muestras.chan4;
        parameter.chan5=muestras.chan5;
        parameter.chan6=muestras.chan6;
        parameter.chan7=muestras.chan7;
        parameter.chan8=muestras.chan8;
        if( parameter.chan2 < 3420 && parameter.chan2 > 899)
        {
            distancia = -(parameter.chan2 - 3614) / 179.08;
        }

        else if( parameter.chan2 < 900 && parameter.chan2 > 286)
        {
            distancia = -(parameter.chan2 - 1222.6) / 24.853;
        }
        else
        {
            distancia = 111111;
        }
        if (distancia >= x1 && distancia < x2 )
        {
            // cm se enciende el led verde PF3
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000008);

        }
        else if (distancia >= x2 && distancia < x3 )
        {
                    // cm se enciende el led rojo PF1

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00000002);

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0x00000002);


        }
        else if (distancia >= x3 && distancia <= x4 )
        {
                    //  cm se encienden ambos leds rojo y verde

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 ,0x00000002);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,0x00000008);


        }
        else
        {
            //los leds permanecen apagados
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);

        }

       // UARTprintf("He leedo ADC0 %d\n ",muestras.chan2);
        //Encia el mensaje hacia QT
        //remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&parameter,sizeof(parameter));
        UART1_SendStruct(MESSAGE_ADC_SAMPLE,(void *)&parameter, sizeof(parameter));
    }
}

//static void Switch1Task(void *pvParameters)
static portTASK_FUNCTION(Switch1Task,pvParameters)
{

    xSemaphoreTake(miSemaforo,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
//        MESSAGE_SW_PARAMETER parametro;
//        parametro.sw.number = 1;
//        //parametro.sw.state = 1;
//        if (VelocidadF2 > 74 && VelocidadF2 < 101){
//            if(!(VelocidadF2 == 75))
//                    VelocidadF2 = VelocidadF2 - 1;
//            activatePWM(VelocidadF2,VelocidadF3);
//        }
//        if (VelocidadF3 > 49 && VelocidadF3 < 76){
//                    if(!(VelocidadF2 == 75))
//                            VelocidadF3 = VelocidadF3 + 1;
//                    activatePWM(VelocidadF2,VelocidadF3);
//                }
 //       lazocerado();//eligimos para lazo cerado
         //girar_robotM(90);//eligimos para probar mover
         lazocerado();

       xSemaphoreTake(miSemaforo,portMAX_DELAY);
//        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
//       UARTprintf("He puesto botton ye mandado mensaje\n");
    }
}

static portTASK_FUNCTION(Switch2Task,pvParameters)
{
    xSemaphoreTake(miSemaforo2,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
//        MESSAGE_SW_PARAMETER parametro;
//        parametro.sw.number = 2;
//        //parametro.sw.state = 1;
//        if (VelocidadF2 > 74 && VelocidadF2 < 101){
//            if(!(VelocidadF2 == 100))
//                    VelocidadF2 = VelocidadF2 + 1;
//                    activatePWM(VelocidadF2,VelocidadF3);
//                }
//        if (VelocidadF3 > 49 && VelocidadF3 < 76){
//            if(!(VelocidadF3 == 50))
//                    VelocidadF3 = VelocidadF3 - 1;
//                    activatePWM(VelocidadF2,VelocidadF3);
//                }

       mover_robotM(12);//eligimos para probar mover
        xSemaphoreTake(miSemaforo2,portMAX_DELAY);
//        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
        //UARTprintf("He puesto botton drecha ye mandado mensaje\n");
    }
}

static portTASK_FUNCTION(Switch3Task,pvParameters)
{
    xSemaphoreTake(miSemaforo3,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
//        configADC_DisparaADC(); //Dispara la conversion (por software)
        lazocerado();
        xSemaphoreTake(miSemaforo3,portMAX_DELAY);
//        remotelink_sendMessage(MESSAGE_SW,&parametro,sizeof(parametro));
        //UARTprintf("He puesto botton drecha ye mandado mensaje\n");
    }
}

static portTASK_FUNCTION(wheelTask,pvParameters)
{
    xSemaphoreTake(miSemaforo4,portMAX_DELAY);
    //
    // Loop forever.
    //
    while(1)
    {
        theta = 0;
        realtheta = realtheta + 1 ;
        xSemaphoreTake(miSemaforo4,portMAX_DELAY);
    }
}

// Función para mapear los valores de un rango a otro
float map(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Función para convertir los valores del joystick (x, y) en señales para los motores
void joystickToMotor(float x, float y, int* motor1, int* motor2) {
    // Normalizar el valor del joystick (x, y) para que esté en el rango [-1, 1]
    float magnitude = sqrt(x * x + y * y);
    if (magnitude > 1) {
        x /= magnitude;
        y /= magnitude;
    }

    // Convertir el valor de 'x' del joystick para el rango de los motores (50-100)
    // Para motor1 (gira en un sentido)
    *motor1 = (int)map(x, -1.0, 1.0, 50, 100);

    // Para motor2 (gira en sentido contrario al motor1)
    *motor2 = (int)map(x, -1.0, 1.0, 100, 50);
}


//Funcion callback que procesa los mensajes recibidos desde el PC (ejecuta las acciones correspondientes a las ordenes recibidas)
static int32_t messageReceived(uint8_t message_type, void *parameters, int32_t parameterSize)
{
    int32_t status=0;   //Estado de la ejecucion (positivo, sin errores, negativo si error)

    //Comprueba el tipo de mensaje
    switch (message_type)
    {
        case MESSAGE_PING:
        {
            status=remotelink_sendMessage(MESSAGE_PING,NULL,0);
        }
        break;
        case MESSAGE_LED_GPIO:
        {
                MESSAGE_LED_GPIO_PARAMETER parametro;

                if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,parametro.value);
                }
                else
                {
                    status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                }
        }
        break;
        case MESSAGE_MOTOR:
                {
                        MESSAGE_MOTOR_PARAMETER parametro;

                        if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                        {
                            joystickToMotor(parametro.x,parametro.y, &motor1, &motor2);
                            activatePWM(motor1,motor2);
                        }
                        else
                        {
                            status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                        }
                }
                break;

        //        case MESSAGE_LED_PWM_BRIGHTNESS:
//        {
//            MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER parametro;
//
//            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
//            {
//                RGBIntensitySet(parametro.rIntensity);
//            }
//            else
//            {
//                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
//            }
//        }
//        break;
        case MESSAGE_ADC_SAMPLE:
        {
            configADC_DisparaADC(); //Dispara la conversion (por software)
        }
        break;
       default:
           //mensaje desconocido/no implementado
           status=PROT_ERROR_UNIMPLEMENTED_COMMAND; //Devuelve error.
    }


    return status;   //Devuelve status
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
//  MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
//          SYSCTL_OSC_MAIN);   //Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 5)
   MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Get the system clock speed.
    g_ulSystemClock = SysCtlClockGet();


    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);
    //Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1
    RGBInit(1);
    MAP_SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
    MAP_SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);  //Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Volvemos a configurar los LEDs en modo GPIO POR Defecto
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3);
    //MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ButtonsInit();
    MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);
//para port A
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_3| GPIO_PIN_4, GPIO_DIR_MODE_IN);
    //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_4);
    MAP_IntPrioritySet(INT_GPIOA,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED
    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4,
                             GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOIntEnable(GPIO_PORTA_BASE,GPIO_PIN_3| GPIO_PIN_4);
    MAP_IntEnable(INT_GPIOA);
    //para port B
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
        ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_DIR_MODE_IN);
        //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4);
        MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_2);
        MAP_IntPrioritySet(INT_GPIOB,configMAX_SYSCALL_INTERRUPT_PRIORITY);//para añadir prioridad by HAMED
        MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2,
                                 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        MAP_GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_2);
        MAP_IntEnable(INT_GPIOB);



    miSemaforo = xSemaphoreCreateBinary();
    miSemaforo2 = xSemaphoreCreateBinary();
    miSemaforo3 = xSemaphoreCreateBinary();
    miSemaforo4 = xSemaphoreCreateBinary();


    /********************************      Creacion de tareas *********************/

    //Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK,COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while(1);
    }

    //Esta funcion crea internamente una tarea para las comunicaciones USB.
    //Ademas, inicializa el USB y configura el perfil USB-CDC
    if (remotelink_init(REMOTELINK_TASK_STACK,REMOTELINK_TASK_PRIORITY,messageReceived)!=pdTRUE)
    {
        while(1); //Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero remotelink.c
    }
    if (remotelinkesp_init(REMOTELINKesp_TASK_STACK,REMOTELINKesp_TASK_PRIORITY,messageReceived)!=pdTRUE)
    {
        while(1); //Inicializo la aplicacion de comunicacion con el esp (Remote). Ver fichero esp8266uart.c
    }

    //Para especificacion 2: Inicializa el ADC y crea una tarea...
    //configADC_IniciaADC();
    configADC_Timer();
    if((xTaskCreate(ADCTask, (portCHAR *)"ADC", ADC_TASK_STACK,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(Switch1Task,(portCHAR *) "Sw1",SW1TASKSTACKSIZE, NULL,SW1TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    if((xTaskCreate(Switch2Task,(portCHAR *) "Sw2",SW2TASKSTACKSIZE, NULL,SW2TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    if((xTaskCreate(Switch3Task,(portCHAR *) "Sw3",SW3TASKSTACKSIZE, NULL,SW3TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    activatePWM(75,75);
    if((xTaskCreate(wheelTask,(portCHAR *) "wheel",wheelTASKSTACKSIZE, NULL,wheelTASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    UART1_Init();
    vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

    while(1)
    {
        //Si llego aqui es que algo raro ha pasado
    }
}

// Rutinas de interrupcion
void GPIOFIntHandler(void){
    //Lee el estado del puerto (activos a nivel bajo)

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS | GPIO_PIN_3);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOF);
    int ui8Delay = 0;
    for(ui8Delay = 0; ui8Delay < 16; ui8Delay++)
    {
    }
    if (!(i32PinStatus & LEFT_BUTTON))

    {
        // Mensaje de ejemplo


            char *mensaje = ("Boton 1\r\n ") ;
            UART1_SendString(mensaje);  // Envía el mensaje
  //          SysCtlDelay(SysCtlClockGet() / 3);  // Espera un segundo (ajustar según frecuencia del sistema)

        xSemaphoreGiveFromISR(miSemaforo,&xHigherPriorityTaskWoken);
    }

    if (!(i32PinStatus & RIGHT_BUTTON))
    {
        // Mensaje de ejemplo


            char *mensaje = ("Boton 2\r\n ") ;
            UART1_SendString(mensaje);  // Envía el mensaje
 //           SysCtlDelay(SysCtlClockGet() / 3);  // Espera un segundo (ajustar según frecuencia del sistema)

        xSemaphoreGiveFromISR(miSemaforo2,&xHigherPriorityTaskWoken);

    }


    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS | GPIO_PIN_4);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void counterroute(void){

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3 );
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOA);
    MAP_GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3);
    int ui8Delay = 0;
    for(ui8Delay = 0; ui8Delay < 16; ui8Delay++)
    {
    }


    if (!(i32PinStatus & GPIO_PIN_3))
    {
        theta = 1;
        char *mensaje = ("OnWhite\r\n ") ;
         UART1_SendString(mensaje);  // Envía el mensaje
        xSemaphoreGiveFromISR(miSemaforo4,&xHigherPriorityTaskWoken);
    }

    MAP_IntEnable(INT_GPIOA);
    MAP_GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3 );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}


void esp32(void){

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_2);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ROM_IntDisable(INT_GPIOB);
    MAP_GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2);
    int ui8Delay = 0;
    for(ui8Delay = 0; ui8Delay < 1600; ui8Delay++)
    {
    }


    if ((i32PinStatus & GPIO_PIN_2))
     {
         xSemaphoreGiveFromISR(miSemaforo3,&xHigherPriorityTaskWoken);

     }
    MAP_IntEnable(INT_GPIOB);
    MAP_GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
