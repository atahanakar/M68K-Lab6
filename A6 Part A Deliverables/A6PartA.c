#include <stdio.h>
#include <Bios.h>
#include <ucos_ii.h>

#define STACKSIZE 256
unsigned char led_count;
unsigned char hex_count;

/*
** Stacks for each task are allocated here in the application in this case = 256 bytes
** but you can change size if required
*/

OS_STK Task1Stk[STACKSIZE];
OS_STK Task2Stk[STACKSIZE];

/* Prototypes for our tasks/threads*/
void Task1(void *); /* (void *) means the child task expects no data from parent*/
void Task2(void *);

/*
** Our main application which has to
** 1) Initialise any peripherals on the board, e.g. RS232 for hyperterminal + LCD
** 2) Call OSInit() to initialise the OS
** 3) Create our application task/threads
** 4) Call OSStart()
*/

void main(void)
{
    led_count = hex_count = 0;

    // initialise board hardware by calling our routines from the BIOS.C source file
    Init_RS232();
    Init_LCD();

    /* display welcome message on LCD display */

    Oline0("Altera DE1/68K");
    Oline1("Micrium uC/OS-II RTOS");

    OSInit(); // call to initialise the OS

/* Now create the 4 child tasks and pass them no data.
 * the smaller the numerical priority value, the higher the task priority
 */

    OSTaskCreate(Task1, OS_NULL, &Task1Stk[STACKSIZE], 11); // highest priority task
    OSTaskCreate(Task2, OS_NULL, &Task2Stk[STACKSIZE], 12);

    OSStart(); // call to start the OS scheduler, (never returns from this function)
}

/*
** IMPORTANT : Timer 1 interrupts must be started by the highest priority task
** that runs first which is Task2
*/

/*
** Task 1 below was created with the highest priority so it must start timer1
** so that it produces interrupts for the 100hz context switches
*/
void Task1(void *pdata)
{
    // must start timer ticker here
    Timer1_Init(); // this function is in BIOS.C and written by us to start timer

    for (;;)
    {
        PortA = led_count++; // increment an LED count on PortA with each tick of Timer 1
        printf("Drive the LEDs to count up \n");
        OSTimeDly(10);
    }
}

void Task2(void *pdata)
{
    for (;;)
    {
        HEX_A = hex_count++; // increment an hex display count on PortA with each tick of Timer 1
        printf("Drive the 7 segment displays to count up \n");
        OSTimeDly(200);
    }
}
