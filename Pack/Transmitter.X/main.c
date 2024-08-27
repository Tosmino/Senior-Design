#include "mcc_generated_files/system.h"
#include <stdio.h>
#include <stdlib.h>
#define CTS _RD6
#define RTS _RF12

// Initialize variables
int finalValue;
int emergency;
int warning;
int waveCounter;
int wave[8] = {1, 2, 4, 8, 16, 32, 64, 128};
int armedState;

void initU2(int BRG)
{
    U2BRG = BRG;
    U2MODE = 0x8008;
    U2STA = 0x0400;
    TRISFbits.TRISF12 = 1;
    RTS = 1;
}

char putU2(char c)
{
    while (U2STAbits.UTXBF)
        ;
    U2TXREG = c;
    return c;
}

void cancelWarning()
{
    emergency = 0;
    PORTA = 0;
    warning = 0;
}

char * getU2(void)
{
    
    char output[3];
    for (int i = 0; i < 3; i++)    //For loop for blocking operation as a delay
    {
        RTS = 0;
        while (!U2STAbits.URXDA) // Wait for data to be in the recieve register
            ;
        output[i] = U2RXREG; 
        RTS = 1;
    }

    return output;
}

char * grabU2(void)
{
    
    char output[3];
    for (int i = 0; i < 3; i++)    //For loop for blocking operation as a delay
    {
        RTS = 0;
        if (U2STAbits.URXDA) // Wait for data to be in the recieve register
            output[i] = U2RXREG; 
            RTS = 1;
    }

    return output;
}

int main(void)
{
    SYSTEM_Initialize();
    
    // Baud rate of 115000
    initU2(32);
    
    
    //putU2('SF,2');
    
    // Configure timer
    T1CON = 0x8030;
    
    // Send ready over uart
    putU2('R');
    putU2('e');
    putU2('a');
    putU2('d');
    putU2('y');
    putU2('\n');
    
    // Initialize the LEDs
    TRISA = 0x00;
    
    armedState = 0;
    
    while (1)
    {        
        
        waveCounter = 0;
        
        // Create the char pointer for the char array
        char *armed;
        
        while (armedState == 0)
        {
            
            // Get the value from the module to check for armed state
            armed = grabU2();
            
            // If the returned value is armed then break to armed
            if (armed[0] == 'a')
            {
                PORTA = 0;
                armedState = 1;
            }
            
            // If the button has been pushed then arm the system
            if (!CTS)
            {
                PORTA = 0;
                armedState = 1;
            }

            
            // Wait one second
            for (long i = 0; i < 128000; i++)
                {
                    Nop();
                }
            
            // Iterate the wave counter
            waveCounter++;
            
            // If the wave counter is out of bounds bring it back
            if (waveCounter >= 8)
            {
                waveCounter = 0;
            }
            
            // Set the LED value to the proper wave value
            // This means the LED's will move down the line while checking for
            // the armed state
            PORTA = wave[waveCounter];
            
        }
        
        // Zero out the LEDs after the armed state is entered
        PORTA = 0;
        
        // Create the char pointer for the char array
        char *result;
        
        // Get the value from the bluetooth module
        result = getU2();
        for (int i = 0; i < 3; i++)
        {   
            // Send the value back to the bluetooth module
            putU2(result[i]);
        }

        // Calculate the value by splitting up the different char digits
        int hundreds;
        int tens;
        int ones;
        hundreds = (result[0] - '0') * 100;
        tens = (result[1] - '0') * 10;
        ones = (result[2] - '0');
        
        // Add together the calculated values
        finalValue = hundreds + tens + ones;
        
        // If the value is out of bounds then start the warning flash
        if (finalValue > 120 || finalValue < 60) 
        {
            warning = 1;
            
            // Flash the LEDs for 30 seconds before entering emergency broadcast
            int counter = 0;
            while (warning == 1) {
                
                // Value to turn all LEDs on
                PORTA = 255;
                for (long i = 0; i < 255999; i++)
                {
                    // If the button has been pushed then stop the warning
                    if (!CTS)
                    {
                        armedState = 0;
                        cancelWarning();
                    }
                    Nop();
                }
                
                // If the button has been pushed then stop the warning
                if (!CTS)
                {
                    armedState = 0;
                    cancelWarning();
                }
                
                // If the counter has reached 30 then start next phase
                if (counter == 30) 
                {
                    emergency = 1;
                    warning = 0;
                    break;
                }
                
                // Iterate the current counter for the warning time
                counter++;
                
                // Value to turn all LEDs off
                PORTA = 0;
                for (long i = 0; i < 255999; i++)
                {
                    // If the button has been pushed then stop the warning
                    if (!CTS)
                    {   
                        armedState = 0;
                        cancelWarning();
                    }
                    Nop();
                }
                
                // If the button has been pushed then stop the warning
                if (!CTS)
                {
                    armedState = 0;
                    cancelWarning();
                }
                
            }
            
            while (emergency == 1) 
            {
                // Value to turn half of the LEDs on
                PORTA = 240;
                for (long i = 0; i < 255999; i++)
                {
                    Nop();
                }
                
                // Value flip the LEDs
                PORTA = 15;
                for (long i = 0; i < 255999; i++)
                {
                    Nop();
                }
            }
        }  
    }
    return 1;
}