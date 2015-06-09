
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <xc.h>

uint8_t message[256] __attribute__((coherent, aligned(4)));
char rxbuffer[100];
int rxbuffer_index = 0;
//int num1 = 1;
//int num2 = 0;
int COM;
int pwmleft=0;
int pwmright=0;
int timer = 20000000;//=1sec
int servoinc = 0;
int servotimer = 10;
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void* pData,
    uintptr_t userData
)
{
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    uint16_t * breakData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

             USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier =
                    controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            breakData = (uint16_t *)pData;
            appDataObject->breakData = *breakData;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            appDataObject->readLength =
                    ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)pData)->length;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. We have received the line coding from the host. Call
             * the usart driver routine to change the baud. We can call this
             * function here as it is not blocking. */

            DRV_USART_BaudSet(appDataObject->usartHandle, appDataObject->setLineCodingData.dwDTERate);

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/*******************************************************
 * USB Device Layer Events - Application Event Handler
 *******************************************************/

void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t configurationValue;
    switch ( event )
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* USB device is reset or device is deconfigured.
             * This means that USB device layer is about to deininitialize
             * all function drivers. Update LEDs to indicate
             * reset/deconfigured state. */

            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2  );
            //BSP_LEDOff( APP_USB_LED_3  );

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData)->configurationValue;
            if (configurationValue == 1)
            {
                /* The device is in configured state. Update LED indication */

                //BSP_LEDOff( APP_USB_LED_1 );
                //BSP_LEDOff( APP_USB_LED_2  );
                //BSP_LEDOn( APP_USB_LED_3  );

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0,
                        APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* mark that set configuration is complete */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Update LEDs */
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2  );
            //BSP_LEDOn( APP_USB_LED_3  );
            break;


        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. Connect the device */
            USB_DEVICE_Attach (appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS was removed. Disconnect the device */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.isConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;

        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
     /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* USART Driver Handle */
    appData.usartHandle = DRV_HANDLE_INVALID;

    /* CDC Instance index for this app object00*/
    appData.cdcInstance = USB_DEVICE_CDC_INDEX_0;

    /* app state */
    appData.state = APP_STATE_INIT ;

    /* device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bDataBits = 8;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bCharFormat = 0;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{  
  static int i = 0;
  //static int prevuser = 1;
  //static int mode = 0;
  //static int time = 0;
  USB_DEVICE_CDC_RESULT writeRequestResult;

    /* Update the application state machine based
     * on the current state */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,    DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                appData.usartHandle = DRV_USART_Open(DRV_USART_INDEX_0,
                    (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING));

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);



                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(appData.isConfigured)
            {
                /* Schedule the first read from CDC function driver */

                appData.state = APP_STATE_CHECK_CDC_READ;
                appData.isReadComplete = false;
                USB_DEVICE_CDC_Read (appData.cdcInstance, &(appData.readTransferHandle),
                        appData.readBuffer, 64);
            }
            break;

        case APP_STATE_CHECK_CDC_READ:
            
            if(APP_StateReset())
            {
                break;
            }

            /* If a read is complete, then schedule a read
             * else check if UART has received data */
            //LATBINV = 0b10000000; //code comes here

            if (appData.readBuffer[0] == '\n'){
                 sscanf(rxbuffer,"%d",&COM);
            }
            if(appData.isReadComplete == true) //if it read a character
            {
                //LATBINV = 0b10000000;
                appData.isReadComplete = false;
                USB_DEVICE_CDC_Read (appData.cdcInstance, &appData.readTransferHandle, appData.readBuffer, 64);
                //DRV_USART_Write(appData.usartHandle, appData.readBuffer, appData.readLength);


                // messages come in one char at a time
                // if the char is a newline, then the message is complete
                for(i=0;i<64;i++) {
                    rxbuffer[i] = appData.readBuffer[i];
                }

                sscanf(rxbuffer,"%d",&COM);

                int x = COM;
                pwmright = 700-x;
                pwmleft = x;
                //PWM out of 799

                if (COM <= 100) {
                    LATBbits.LATB4 = 0;
                    OC2RS = 100;
                    LATBbits.LATB8 = 0;
                    OC3RS = 600;
                }
                else if (COM < 500) {
                    leftwheel(pwmleft,0);
                    rightwheel(pwmright,0);
                }
                else if (COM >= 500) {
                    LATBbits.LATB4 = 0;
                    OC2RS = 600;
                    LATBbits.LATB8 = 0;
                    OC3RS = 100;
                }

                //SHELLS!
                if (servoinc == servotimer){
                    if (servotimer == 10)
                    {
                        servotimer = 1;
                        OC1RS = 5000;
                    }
                    else if (servotimer == 1) {
                        LATBINV = 0b10000000;
                        servotimer = 10;
                        OC1RS = 1250;
                    }
                    servoinc=0;
                }
                   
            }
           
            appData.state = APP_STATE_CHECK_UART_RECEIVE;
            break;

        case APP_STATE_CHECK_UART_RECEIVE:
            
            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was received on the UART */
            //timer corresp to 1Hz
            if (_CP0_GET_COUNT() > (timer) ) {
              servoinc++; 

              sprintf(message, ""); //NEED TO SEND SOMETHING
              int len = 0;
              while(message[len]>0){
                len++;
              }
              
              writeRequestResult = USB_DEVICE_CDC_Write(0,
                      &appData.writeTransferHandle,
                      message,
                      len,
                      USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
              if(USB_DEVICE_CDC_RESULT_OK != writeRequestResult) {
                //LATBbits.LATB7 = 1;
              }
              else {
                //LATBbits.LATB7 = 0;
              }
              _CP0_SET_COUNT(0);
            }

//            if(DRV_USART_Read(appData.usartHandle, &appData.uartReceivedData, 1) > 0)
//            {
//                /* We have received data on the UART */
//
//                USB_DEVICE_CDC_Write(0, &appData.writeTransferHandle,
//                        &appData.uartReceivedData, 1,
//                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//
//            }

            appData.state = APP_STATE_CHECK_CDC_READ;
            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}

void leftwheel(int OCRSval, int dir) //forward = 0, backward = 1
{
    LATBbits.LATB4 = dir;
    if ((OCRSval < PR3) || (OCRSval > 0)) {
        OC2RS = OCRSval;
    }
    
}

void rightwheel(int OCRSval, int dir) //forward = 0, backward = 1
{
    LATBbits.LATB8 = dir;
    if ((OCRSval < PR3) || (OCRSval > 0)) {
        OC3RS = OCRSval;
    }
}

/*
int readADC(void) {
    int elapsed = 0;
    int finishtime = 0;
    int sampletime = 20;
    int a = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finishtime = elapsed + sampletime;
    while (_CP0_GET_COUNT() < finishtime) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    a = ADC1BUF0;
    return a;
}
*/

/*******************************************************************************
 End of File
 */

