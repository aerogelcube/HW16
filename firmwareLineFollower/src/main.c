/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <xc.h> 

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );
    // PWM digital pin, set to OC1 / B15
    ANSELBbits.ANSB15 = 0; // 0 = digital, set this since default for this pin is analog
    RPB15Rbits.RPB15R = 0b0101; //Set B15 as OC1

    // PWM for servo motor
    OC1CONbits.OCTSEL = 0; 	// 0=Timer 2 is used for output compare
    OC1CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC1CON bits are defaults
    OC1R = 1250; 		// duty cycle = OC1RS/(PR2+1) = 0.075, 1.5ms/20ms for neutral servo position
				// initialize before turning OC1 on; afterward it is read-only
    T2CONbits.TCKPS = 0b100; 	// set prescaler to 1:16 (N=16)
    PR2 = 49999;                // period = (PR2+1)*N=16*25ns = 20 ms, or 50 Hz
    TMR2 = 0;			// initalize counter to 0

    T2CONbits.ON = 1; 		// turn on Timer2
    OC1CONbits.ON = 1;          // turn on OC1

    /*Motors - both use Timer 3*/
    T3CONbits.TCKPS = 0; 	// set prescaler to 1:1 (N=1)
    TMR3 = 0;			// initalize counter to 0
    PR3 = 799;                  // period = (PR3+1)*N=1*25ns = 20 us, or 20 kHz

    /*Motor B: LEFT WHEEL (top down view)*/
    //PHA1 pin
    TRISBbits.TRISB4 = 0; //digital output
    LATBbits.LATB4 = 0; //output on=0 initially

    // PWM digital pin, set to OC2 (ENA1)
    RPB5Rbits.RPB5R = 0b0101;   // Set B5 as OC2
    OC2CONbits.OCTSEL = 1; 	// 1=Timer 3 is used for output compare
    OC2CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC2CON bits are defaults
    OC2R = 0;                   // duty cycle = OC2RS/(PR3+1) = 0.075, 1.5ms/20ms for neutral servo position
    OC2RS = 0;			// initialize before turning OC2 on; afterward it is read-only

    /*Motor A: RIGHT WHEEL*/
    //PHA2 pin
    TRISBbits.TRISB8 = 0; //digital output
    LATBbits.LATB8 = 0; //output on=0 initially

    // PWM digital pin, set to OC3 (ENA2)
    RPB9Rbits.RPB9R = 0b0101;   // Set B9 as OC3
    OC3CONbits.OCTSEL = 1; 	// 1=Timer 3 is used for output compare
    OC3CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC2CON bits are defaults
    OC3R = 0;                   // duty cycle = OC3RS/(PR3+1) = 0.075, 1.5ms/20ms for neutral servo position
    OC3RS = 0;          	// initialize before turning OC1 on; afterward it is read-only

    T3CONbits.ON = 1; 		// turn on Timer4
    OC3CONbits.ON = 1;          // turn on OC3
    OC2CONbits.ON = 1;          // turn on OC2

    // set up LED1 pin as a digital output - light checker
    TRISBbits.TRISB7 = 0; //digital output
    LATBbits.LATB7 = 0; //output on=0 initially
    
    while ( true )
    {
        //LATBbits.LATB7 = 1;
        //LATBINV = 0b10000000;
        //OC3RS=400;
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

