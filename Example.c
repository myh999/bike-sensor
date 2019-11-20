#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <msp430.h>
#include <string.h>
#include "driverlib/driverlib.h"
#include <stdlib.h>
#include <msp430fr4133.h>
#include "driverlib/timer_a.h"
#include <stdio.h>
/*
 * This project contains some code samples that may be useful.
 *
 */
#define delay_for_keypad 300000
#define NUM_THRESHOLDS 5
char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

volatile int thresholds[NUM_THRESHOLDS];
volatile unsigned int curr_index = 0;
volatile unsigned int x = 0;
volatile unsigned int y = 0;
volatile unsigned int first_time = 1;
int test = 10;
int rear = 1;
int reset = 0;
void setup_keypad() {
   GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN0); // Col3
   GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5); // Col1
   GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN3); // Col2
   GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
   GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
   // Set Rows as Outpin rows
   GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // Row2
   GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7); // Row3
   GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2); // Row1
   GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1); // Row4
}
void displayNum(int value) {
    x = 69;
    int value2 = value;
    int value3 = value;
    int count = 0;
    while(value2 != 0) {
        count++;
        value2 = value2/10;
    }
    int where[6];
    where[0] = pos6;
    where[1] = pos5;
    where[2] = pos4;
    where[3] = pos3;
    where[4] = pos2;
    where[5] = pos1;
    int i = 0;
    for(i = 0; i<count; i++) {
        showChar('0' + value3%10, where[i]);
        value3 = value3/10;
    }
}
void initialize_keypad() {
    unsigned int i = 0;
    for(i = 0; i<NUM_THRESHOLDS; i++)
        thresholds[i] = 0;
}
int get_thresholds() {
    // Iterate over the rows
    // Enable ith row
    // Check all columnn
    while(curr_index < NUM_THRESHOLDS) {
        // Row 1
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH) {
            __delay_cycles(delay_for_keypad);
            return 3;
         }
           if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_HIGH) {
              __delay_cycles(delay_for_keypad);
              return 1;
           }
           if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH) {
                      __delay_cycles(delay_for_keypad);
                      return 2;
           }
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        // Row 2
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH) {
            __delay_cycles(delay_for_keypad);
            return 6;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_HIGH) {
                   __delay_cycles(delay_for_keypad);
                   return 4;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH) {
                   __delay_cycles(delay_for_keypad);
                   return 5;
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

        // Row 3
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH) {
            __delay_cycles(delay_for_keypad);
            return 9;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_HIGH) {
                   __delay_cycles(delay_for_keypad);
                   return 7;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH) {
                   __delay_cycles(delay_for_keypad);
                   return 8;
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        // Row4
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
        if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH) {
            __delay_cycles(delay_for_keypad);
                 return -1;

         }
         if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_HIGH) {
             __delay_cycles(delay_for_keypad);
             return -1;

         }
         if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH) {
                    __delay_cycles(delay_for_keypad);
                    return 0;
         }
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    }
}
void beep() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN7);
        __delay_cycles(70000);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7);
}
void two_beeps() {
    /*Timer_A_outputPWM(TIMER_A0_BASE, &param);
    __delay_cycles(250000);
    Timer_A_stop(TIMER_A0_BASE);
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
    __delay_cycles(250000);
    Timer_A_stop(TIMER_A0_BASE);*/
    beep();
    beep();
}
void four_beeps() {
    beep();
    beep();
    beep();
    beep();
}
void rear_LCDS(int time) {
    // Before this we should have all thresholds sorted
    //Set all LCDS to low
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    // 5.3, 1.3, 1.4, 1.5
    if(time <= thresholds[0]) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        return;
    }
    if(time >= thresholds[1] && time < thresholds[2]) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        return;
    }
    else if(time >= thresholds[2] && time < thresholds[3]) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        return;
    }
    else if(time >= thresholds[3]) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
        return;
    }

}
void rear_functions() {
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
   __delay_cycles(1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
   Timer_A_clear(TIMER_A1_BASE);
    while(GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN0) == GPIO_INPUT_PIN_LOW){        //goes ahead after echo is low
      ; }
   Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);        //start timer
   while(GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){        //goes ahead after echo is low
      ; }
   Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);     // This is not supposed to be here
   Timer_A_stop(TIMER_A1_BASE);
   volatile uint16_t time;
   time = Timer_A_getCounterValue(TIMER_A1_BASE);
  // time = test;
   time = time / 58;
   int distance = time;
  /*showChar('0' + distance%10,pos6);
   distance = distance/10;
   showChar('0' + distance%10,pos5);
   distance = distance/10;
   showChar('0' + distance%10,pos4);
   distance = distance/10;
   showChar('0' + distance%10,pos3);
   distance = distance/10;
   showChar('0' + distance%10,pos2);
   distance = distance/10;
   showChar('0' + distance%10,pos1); */
   test++;
   if(test > 30)
       test = 0;
   distance = 0;
   rear_LCDS(time);
   __delay_cycles(16000);
   time = 0;
}
void front_functions() {
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
   __delay_cycles(1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
   Timer_A_clear(TIMER_A1_BASE);
   while(GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN2) == GPIO_INPUT_PIN_LOW){        //goes ahead after echo is low
      ; }
   Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);        //start timer
   while(GPIO_getInputPinValue(GPIO_PORT_P5,GPIO_PIN2) == GPIO_INPUT_PIN_HIGH){        //goes ahead after echo is low
      ; }
   Timer_A_stop(TIMER_A1_BASE);
   volatile uint16_t time;
   time = Timer_A_getCounterValue(TIMER_A1_BASE);
   time = time / 58;
   int distance = time;
  showChar('0' + distance%10,pos6);
   distance = distance/10;
   showChar('0' + distance%10,pos5);
   distance = distance/10;
   showChar('0' + distance%10,pos4);
   distance = distance/10;
   showChar('0' + distance%10,pos3);
   distance = distance/10;
   showChar('0' + distance%10,pos2);
   distance = distance/10;
   showChar('0' + distance%10,pos1);
   distance = 0;
   //rear_LCDS(time);
   time = 0;
   __delay_cycles(16000);
   //if(test >= 1000)
     //  test = 11;
}
void Init_InputEcho() {
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
}
void Init_interrupt(){
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_enableInterrupt(SW1_PORT, SW1_PIN);
    GPIO_selectInterruptEdge(SW1_PORT, SW1_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);

}
void run_keypad() {
    initialize_keypad();
    setup_keypad();
    curr_index = 0;
    while(curr_index < NUM_THRESHOLDS) {
        int curr = 0;
        int total = 0;
        while(1) {
            curr = get_thresholds();
            clearLCD();
            __delay_cycles(delay_for_keypad);
            if(curr == -1)
                break;
            total = total*10 + curr;
            displayNum(total);
        }
        thresholds[curr_index] = total;
        curr_index++;
        displayScrollText("RECORDED");
    }
//    while(curr_index < NUM_THRESHOLDS) {
//        get_thresholds();
//        curr_index++;
//        return;
//    }
}
void run_sensors() {
    int i = 0;
   for(i = 0; i<10; i++)
   front_functions();
   __delay_cycles(20);
   for(i = 0; i<10; i++)
   rear_functions();
}
void run_everything() {
    while(1) {
        displayScrollText("SETUP MODE");
        run_keypad();
        displayScrollText("USER MODE");
        while(reset == 0) {
            run_sensors();
        }
        reset = 0;
        clearLCD();
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
          GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    }
}
void main(void)
{
    //int delay_for_keypad = 2000;
    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */
    //Turn off interrupts during initialization
    __disable_interrupt();
    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);
    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_Timer();
    Init_interrupt();
    PMM_unlockLPM5();
    Init_InputEcho();
    __enable_interrupt();
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
     //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    // Keypad
   // setup_keypad();
 //   get_thresholds();
  //  displayNum(thresholds[0]);
   //1`  return;
    //All done initializations - turn interrupts back on.

    //return;
   //thresholds[0] = 5;
   //thresholds[1] = 10;
   //thresholds[2] = 15;
   //thresholds[3] = 20;
   //thresholds[4] = 30;
   run_everything();
}
      /* if(rear == 1) {
           rear_functions();
           rear = 0;
       }
       if(rear == 0) {
           front_functions();
           rear = 1;
       } */
       /*int i = 0;
       for(i = 0; i<10; i++)
       rear_functions();
       __delay_cycles(100);
       i = 0;
       for(i = 0; i<10; i++)
       front_functions(); */
/*    {
      if(!audio_run_once) {
            int count = 100;
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN4);
            while(count > 0) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
            __delay_cycles(70000);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
            count--;
          }
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
            audio_run_once = 1;
      }
*/
   /*    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1) {
            displayScrollText("KEY 1");
        } else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == 1) {
            displayScrollText("KEY *");
        } else if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
            displayScrollText("KEY 7");
        } else if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
            displayScrollText("KEY 4");
        }
*/
        // Setting the pulse of the sensor
       /* GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
        int timer = 0;
        int waiting = 0;
        // waiting for the rising edge of the echo(p1.4)
        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == 0) {
            ;
        }
        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) != 0) {
            timer = timer + 1;
        }
         */



        // Sensor + audio
        //Checking value of Port 8 , Pin 0
        /*if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1)
        {
            showChar('N',pos1);
            // Convert the output to distance ()
            // need short key for keypad, basically diodes
            // Get the audio to work
           // GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
            //__delay_cycles(70000);
          //  GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
            // Keypad input
            // LED input
        }
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
        {
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            buttonState = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
        {
            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
            buttonState = 0;                            //Capture new button state
        }
        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
        if (ADCState == 0)
        {
            showHex((int)ADCResult); //Put the previous result on the LCD display
            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }
    }
}
    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    //Set LED1 and LED2 as outputs
    GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}
/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */
    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}
/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */
    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */
    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;
    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }
    EUSCI_A_UART_enable(EUSCI_A0_BASE);
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}
/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);
    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}
/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h
    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}
void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */
    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);
    ADC_enable(ADC_BASE);
    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);
    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);
    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);
    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}
//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);
    ADC_clearInterrupt(ADC_BASE, ADCStatus);
    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
void Init_Timer(void){
    timer_param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timer_param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    timer_param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    timer_param.timerClear = TIMER_A_DO_CLEAR;
    //startTimer = true;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &timer_param);
}

#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
     __disable_interrupt();
    if(first_time == 1) {
        first_time = 0;
        GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
            __enable_interrupt();
        return;
    }
    reset = 1;
    displayScrollText("VOLARE");
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
    __enable_interrupt();
}
