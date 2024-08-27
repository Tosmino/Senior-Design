#include "mcc_generated_files/system.h"

unsigned int echoWidth;
unsigned int tofFlag;
unsigned int x;

void _ISR _IC1Interrupt(void)
{
    static enum {wait4RISE, wait4FALL} tofState = wait4RISE;

    switch(tofState)
    {
        case wait4RISE:
            
            timeSTART = IC1BUF; // initialize start point
            IC1CON = 0x0000; // turn module off
            IC1CON = 0x0002; // Set input capture mode for falling edge
            tofState = wait4FALL; // Next state
            break;

        case wait4FALL:
            timeSTOP = IC1BUF; // store end time
            echoWidth = timeSTOP - timeSTART; // Pulse width
            tofFlag = 1; // Set flag that it's time to calculate
            tofState = wait4RISE; // Reset state machine
            IC1CON = 0x0000; // Turn module off
            break;
    }
    _IC1IF = 0;
}

unsigned getDistance(unsigned temperature)
{
    long Dinternal;
    unsigned in Dobject;
    tofFlag = 0; // Clear flag
    _RD9 = 1; // trigger
    TMR3 = 0;while (TMR3 < 3); // Set for 10 us
    _RD9 = 0; // reset trigger

    TMR3 = 0; // Clear count value 
    
    _IC1IF = 0; // clear interrupt flag
    _IC1IE = 1; // enable interrupts
    IC1CON = 0x0000; // Turn module off
    IC1CON = 0x0003; // Make it look for rising edge

    while(tofFlag == 0);
    _IC1IE = 0; // Clear interrupt enable

    Dinternal = ((33010 + 6 * temperature) * ((long)echoWidth)) / 5000;
    Dobject = (int) Dinternal;
}

int main(void)
{
    unsigned int temp = 20;
    SYSTEM_Initialize();

    TRISDbits.TRISD9 = 0; // Set as output
    TRISDbits.TRISD8 = 1; // Set as input
    T3CON = 0x8020; // 64:1 prescalar
    _IC1IF = 0; // Clear interrupt flag

    while(1)
    {
        x = getDistance(temp);
        TMR3 = 0; while(TMR3<10000);
    }

}