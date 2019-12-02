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
<<<<<<< HEAD
=======

/*
 * This project contains some code samples that may be useful.
 *
 */
>>>>>>> 14fd5e62cb80b0301b9815920bb55f707d58e9c7

char ADCState = 0;                                 //Busy state of the ADC
int16_t ADCResult = 0;                   //Storage for the ADC conversion result
int pos[6] = { pos6, pos5, pos4, pos3, pos2, pos1 }; // to display to the LCD
long ref = 0;
long ref_distance = 0;
int calibrated = 0;
int mode = 0;
int index = 0;
unsigned int t_value[5] = {0,0,0,0,0}; //0 and 1 are beeps, 2-4 are thresholds
int beep_t[2] = { 50, 100 };
unsigned int LED_t[3] = { 50, 100, 150 };
unsigned int fV[3] = { 0, 0, 0 };
int bV[3] = { 0, 0, 0 };
int i = 1;
int index_t = 0;
int exit_flag_f = 0;
int exit_flag_b = 0;

unsigned int fD = 0;
unsigned int bD = 0;

char KeyPadLayout[4][3] = {{'1','2','3'},
                           {'4','5','6'},
                           {'7','8','9'},
                           {'A','0','P'}};
char chars[3] = {'0', '0', '0'};

//#if defined(_TI_COMPILER_VERSION_) || defined(_IAR_SYSTEMS_ICC_)
//#pragma vector = PORT1_VECTOR
//__interrupt
//#elifdefined(_GNUC_)
//    __attribute__((interrupt(PORT1_VECTOR)))
//#endif

/*
 void
 P1_ISR(void)
 {
 __disable_interrupt();
 __delay_cycles(10000); //Might not need this but put in here
 //This will start timer on rising edge, stop on falling, then print counter value

 GPIO_clearInterrupt(PB2_PORT, PB2_PIN);
 __enable_interrupt();
 // Need to get RTC working
 }
 */

int scan_cols(void){
    if(GPIO_getInputPinValue(COL_1_PORT, COL_1_PIN) == GPIO_INPUT_PIN_HIGH){
        return 1;
    }
    else if(GPIO_getInputPinValue(COL_2_PORT, COL_2_PIN) == GPIO_INPUT_PIN_HIGH){
        return 2;
    }
    else if(GPIO_getInputPinValue(COL_3_PORT, COL_3_PIN) == GPIO_INPUT_PIN_HIGH){
        return 3;
    }
    return 0;
}

void setup_front(){

    int beepTwice;

    int beepTwiceFlag = 1;

    int beepFour;

    int beepFourFlag = 0;

    while(1){
        if(beepTwiceFlag){
            displayScrollText("2 BEEPS");
        }

        while(beepTwiceFlag){

            beepTwice = keypad();

            if(beepTwice == -2){

                return;

            }

            if(beepTwice > 400 || beepTwice < 2){

                displayScrollText("MUST BE BETWEEN 2 AND 400");

                continue;

            }

            beepTwiceFlag = 0;

            beepFourFlag = 1;

            break;

        }

        if(beepFourFlag){

            displayScrollText("4 BEEPS");

        }

        while(beepFourFlag){

            beepFour = keypad();

            if(beepFour == -1){

                beepTwiceFlag = 1;

                beepFourFlag = 0;

                break;

            }

            if(beepFour == -2){

                return;

            }

            if(beepFour >= beepTwice){

                displayScrollText("MUST BE LESS THAN MIN 2 BEEPS");

                continue;

            }

            beepFourFlag = 0;

            break;

        }

        if(beepFourFlag == 0 && beepTwiceFlag == 0){

            break;

        }

    }

    beep_t[1] = beepTwice;

    beep_t[0] = beepFour;

    exit_flag_f = 1;

}

int keypad(void){
    showChar('C', pos5);
    showChar('M', pos6);
    int counted = 0;
    int key;
    int row;
    int column;
    int num = 0;
    int power = 1;

    while(1){
        while(1){
//            if(setup_interrupted){
//                clearLCD();
//                return -2;
//            }
            GPIO_setOutputHighOnPin(ROW_1_PORT, ROW_1_PIN);
            row = 1;
            column = scan_cols();
            if(column != 0){
                GPIO_setOutputLowOnPin(ROW_1_PORT, ROW_1_PIN);
                break;
            }
            GPIO_setOutputLowOnPin(ROW_1_PORT, ROW_1_PIN);

            GPIO_setOutputHighOnPin(ROW_2_PORT, ROW_2_PIN);
            row = 2;

            column = scan_cols();
            if(column != 0){
                GPIO_setOutputLowOnPin(ROW_2_PORT, ROW_2_PIN);
                break;
            }
            GPIO_setOutputLowOnPin(ROW_2_PORT, ROW_2_PIN);

            GPIO_setOutputHighOnPin(ROW_3_PORT, ROW_3_PIN);
            row = 3;

            column = scan_cols();
            if(column != 0){
                GPIO_setOutputLowOnPin(ROW_3_PORT, ROW_3_PIN);
                break;
            }
            GPIO_setOutputLowOnPin(ROW_3_PORT, ROW_3_PIN);

            GPIO_setOutputHighOnPin(ROW_4_PORT, ROW_4_PIN);
            row = 4;

            column = scan_cols();
            if(column != 0){
                GPIO_setOutputLowOnPin(ROW_4_PORT, ROW_4_PIN);
                break;
            }
            GPIO_setOutputLowOnPin(ROW_4_PORT, ROW_4_PIN);

        }

        __delay_cycles(250000);

        key = KeyPadLayout[row - 1][column - 1];

        if(key == 'P'){
            clearLCD();
            power /= 10;
            int i;
            for(i = 0; i < counted; i++){
                num += (chars[i]-48)*power;
                power /= 10;
            }
            return num;
        }
        if(key == 'A'){
            if(counted == 0){
                return -1;
            }
            clearLCD();
            counted = 0;
            power = 1;
            showChar('C', pos5);
            showChar('M', pos6);
            continue;
        }
        if(counted == 3){
            continue;
        }

        chars[counted] = key;
        showChar(chars[counted], pos4);
        if(counted == 1){
            showChar(chars[counted - 1], pos3);
        }
        if(counted == 2){
            showChar(chars[counted - 1], pos3);
            showChar(chars[counted - 2], pos2);
        }
        counted++;
        power *= 10;
    }

}


void setup_back(){
    int green = -1;
    int greenFlag = 1;
    int yellowFlag = 0;
    int yellow = -1;
    int orangeFlag = 0;
    int orange = -1;

    while(1){
        if(greenFlag){
            displayScrollText("GREEN");
        }
        while(greenFlag){
            green = keypad();
            if(green == -2){
                return;
            }
            if(green > 400 || green < 3){
                displayScrollText("MUST BE BETWEEN 3 AND 400");
                continue;
            }
            greenFlag = 0;
            yellowFlag = 1;
        }

        if(yellowFlag){
            displayScrollText("YELLOW");
        }
        while(yellowFlag){
            yellow = keypad();
            if(yellow == -1){
                greenFlag = 1;
                yellowFlag = 0;
                break;
            }
            if(yellow == -2){
                return;
            }
            else if(yellow >= green || yellow < 2){
                displayScrollText("MUST BE BETWEEN 2 AND GREEN");
                continue;
            }
            yellowFlag = 0;
            orangeFlag = 1;
        }

        if(orangeFlag){
            displayScrollText("ORANGE");
        }
        while(orangeFlag){
            orange = keypad();
            if(orange == -1){
                  yellowFlag = 1;
                  orangeFlag = 0;
                  break;
            }
            if(orange == -2){
                return;
            }
            else if(orange >= yellow || orange < 1){
                displayScrollText("MUST BE BETWEEN 1 AND YELLOW");
                continue;
            }
            orangeFlag = 0;
            break;
        }
        if(greenFlag == 0 && yellowFlag == 0 && orangeFlag == 0){
            break;
        }
    }

    LED_t[2] = green;
    LED_t[1] = yellow;
    LED_t[0] = orange;
    exit_flag_b = 1;
}




void main(void)
{

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO(); //Sets all pins to output low as a default
    Init_LCD();  //Sets up the LaunchPad LCD display
    Init_Clock(); //sets up system clocks
//    Init_UART(); //Sets up an echo over a COM port
//    Init_ADC();     //Sets up the ADC to sample
    Init_Timer();
//    Init_interrupt();
//     PMM_unlockLPM5();
//    init_inecho();
    __enable_interrupt();

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    //displayScrollText("WTF IS THIS123");
    //All done initializations - turn interrupts back on.
    //__enable_interrupt();

    uint8_t col1 = 0;
    uint8_t col2 = 0;
    uint8_t col3 = 0;
    uint8_t keypadState = 0;
    uint16_t outputRow = 0;
    uint16_t output_row_port = ROW_1_PORT;
    uint16_t output_row_pin = ROW_1_PIN;
    int keyVal = 0;

    GPIO_setOutputHighOnPin(output_row_port, output_row_pin);
    char chars[6];

    while (1) //Do this when you want an infinite loop of code
    {
        unsigned int calibrate_value = 0;
        // Setup Mode
        while (mode == 0)
        {
            setup_front();
            setup_back();
            if(exit_flag_f && exit_flag_b){
                mode = 1;
            }
        }

        // Mode 2 User mode
        while (mode == 1)
        {
            //Will need to start both rear and front
            run_sensor();
        }
    }
}

int update_calibrate_value(int input, int value) {
    int final_value = value;
    if (input == 7) {
        final_value = 0;
    } else if (input == 9) {
        //Set this as our threshold value then increase the global pointer and keep doing that for everything
        if (index_t == 1){
            if(t_value[index_t - 1] >= final_value){
                displayScrollText("MUST BE BETWEEN GREATER THAN VALUE BEFORE");
            } else{
                t_value[index_t] = final_value;
            }
        }
        else if (index_t > 1){
            //index_t ==2 means red
            //index_t == 3 means Yellow
            //index_t == 4 means Orange
            // Else its green
        }
            //index_t ==2 means
        t_value[index_t] = final_value;
        index_t++;

        mode = 1;
    } else if (input == 4) {
        final_value -= 10;
    } else if (input == 6) {
        final_value += 10;
    }
    displayIntLCD((unsigned int) final_value);
    return final_value;
}

void calibrate(long value)
{
    ref = value;
    ref_distance = fetch_data();
    calibrated = true;
}

unsigned int front_sensor()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7); //Second echo Front
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); //This is trigger so we need to start the trigger
    __delay_cycles(16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

    //We clear the timer before the start
    Timer_A_clear(TIMER_A1_BASE);
    long timeout = 0;
    while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_LOW && timeout < 300000) {
        timeout++;
    }        //goes ahead after echo is low for front
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    //displayScrollText("DRFEFE");
    timeout = 0;
    while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH && timeout < 300000) {
        timeout++;
    }
    Timer_A_stop(TIMER_A1_BASE);
    unsigned int time;
    time = Timer_A_getCounterValue(TIMER_A1_BASE) / 58;
    __delay_cycles(50000);

    return time;
}

unsigned int back_sensor()
{
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1); //Will need to change the pins to be the ones we used 1.7 and 5.1
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1); //First echo Rear
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); //This is trigger so we need to start the trigger
    __delay_cycles(16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

    Timer_A_clear(TIMER_A1_BASE);
    long timeout = 0;
    while (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == GPIO_INPUT_PIN_LOW
            && timeout < 30000)
    {        //goes ahead after echo is low
        timeout++;
    }
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);  //start timer
    timeout = 0;
    while (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == GPIO_INPUT_PIN_HIGH
            && timeout < 30000)
    {        //goes ahead after echo is low
        timeout++;
    }
    Timer_A_stop(TIMER_A1_BASE);
    unsigned int time;
    time = Timer_A_getCounterValue(TIMER_A1_BASE) / 58;
    __delay_cycles(16000);
    return time;

}

void run_sensor()
{
    while (1)
    {
        fV[index] = front_sensor();
        bV[index] = back_sensor();

        fD = (fV[index] + 4 * fV[(index + 2) % 3] + fV[(index + 1) % 3]) / 6;
        bD = (bV[index] + 4 * bV[(index + 2) % 3] + bV[(index + 1) % 3]) / 6;
        index++;
        index = index % 3;

        unsigned int lowest = fD < bD ? fD : bD;
        displayIntLCD(lowest);
        turn_LED(lowest);
        if (lowest <= beep_t[0])
        {
            beep(1);
//            pulse_LED();
        }
        else if (lowest > beep_t[0] && lowest <= beep_t[1])
        {
            beep(0);
//            pulse_LED();
        }
//        __delay_cycles(16000);
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == GPIO_INPUT_PIN_LOW) {
            mode = 0;
            break;
        }
    }
    return;
}

void displayIntLCD(unsigned int numDisp)
{
    char charDisp[3];
    charDisp[0] = numDisp % 10;
    numDisp /= 10;
    charDisp[1] = numDisp % 10;
    numDisp /= 10;
    charDisp[2] = numDisp % 10;
    numDisp /= 10;

    showChar('0' + (charDisp[0]), pos6);
    showChar('0' + (charDisp[1]), pos5);
    showChar('0' + (charDisp[2]), pos4);

}

void Init_Timer(void)
{
    timer_param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timer_param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    timer_param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    timer_param.timerClear = TIMER_A_DO_CLEAR;
    //startTimer = true;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &timer_param);
}

void Init_interrupt()
{
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_enableInterrupt(SW1_PORT, SW1_PIN);
    GPIO_selectInterruptEdge(SW1_PORT, SW1_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
}

void pulse_LED()
{
    GPIO_setOutputHighOnPin(YELLOW_PORT, YELLOW_PIN);
    __delay_cycles(20000);
    GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
    __delay_cycles(20000);
}

void turn_LED(unsigned int distance)
{
    if (distance <= LED_t[0])
    {
        //Turn off everything but RED
        GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
        GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
        GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
        GPIO_setOutputHighOnPin(RED_PORT, RED_PIN);
    }
    else if (distance <= LED_t[1] && distance > LED_t[0])
    {
        //Turn off everything but YELLOW
        GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
        GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
        GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        GPIO_setOutputHighOnPin(YELLOW_PORT, YELLOW_PIN);
    }
    else if (distance <= LED_t[2] && distance > LED_t[1])
    {
        //Turn off everything but orange
        GPIO_setOutputLowOnPin(GREEN_PORT, GREEN_PIN);
        GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
        GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        GPIO_setOutputHighOnPin(ORANGE_PORT, ORANGE_PIN);
    }
    else
    {
        //Turn green on and turn off everything
        GPIO_setOutputLowOnPin(ORANGE_PORT, ORANGE_PIN);
        GPIO_setOutputLowOnPin(YELLOW_PORT, YELLOW_PIN);
        GPIO_setOutputLowOnPin(RED_PORT, RED_PIN);
        GPIO_setOutputHighOnPin(GREEN_PORT, GREEN_PIN);
    }
}

void beep(int state)
{
    int i = 0;

    if (state == 1)
    {
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(500);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
        __delay_cycles(50000);
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(500);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
        __delay_cycles(50000);
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(500);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
        __delay_cycles(50000);
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(500);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
    }
    else
    {
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(1000);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
        __delay_cycles(100000);
        for (i = 0; i < 100; i++)
        {
            GPIO_setOutputHighOnPin(AUDIO_PORT, AUDIO_PIN);
            __delay_cycles(1000);
            GPIO_setOutputLowOnPin(AUDIO_PORT, AUDIO_PIN);
        }
    }
}

uint16_t getNextKeypadInputPort(uint16_t row)
{
    if (row == 0) {
        return ROW_1_PORT;
    } else if (row == 1) {
        return ROW_2_PORT;
    } else if (row == 2) {
        return ROW_3_PORT;
    } else {
        return ROW_4_PORT;
    }
}

uint16_t getNextKeypadInputPin(uint16_t row)
{
    if (row == 0) {
        return ROW_1_PIN;
    } else if (row == 1) {
        return ROW_2_PIN;
    } else if (row == 2) {
        return ROW_3_PIN;
    } else {
        return ROW_4_PIN;
    }
}

int getKeypadValue(uint16_t row_pin, uint16_t col_pin)
{
    if (row_pin == 0 && col_pin == 0)
        return 1;
    if (row_pin == 0 && col_pin == 1)
        return 2;
    if (row_pin == 0 && col_pin == 2)
        return 3;
    if (row_pin == 1 && col_pin == 0)
        return 4;
    if (row_pin == 1 && col_pin == 1)
        return 5;
    if (row_pin == 1 && col_pin == 2)
        return 6;
    if (row_pin == 2 && col_pin == 0)
        return 7;
    if (row_pin == 2 && col_pin == 1)
        return 8;
    if (row_pin == 2 && col_pin == 2)
        return 9;
    if (row_pin == 3 && col_pin == 0)
        return 10;
    if (row_pin == 3 && col_pin == 1)
        return 0;
    if (row_pin == 3 && col_pin == 2)
        return 11;
    return -1;
}

int fetch_data()
{
    int timer = 0;

    while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 0)
        ;

    while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) != 0)
    {
        timer += 1;
        __delay_cycles(16);
    }
    return timer;
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P1,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P2,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P3,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P4,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P5,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P6,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P7,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4
                    | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P8,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN4 | GPIO_PIN5
                    | GPIO_PIN6 | GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN3);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN2);

//    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
//    GPIO_setAsInputPinWithPullDownResistor(COL_PORT,
//                                           COL_PIN_1 | COL_PIN_2 | COL_PIN_3);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);

    //Set LEDS as outputs
    GPIO_setAsOutputPin(GREEN_PORT, GREEN_PIN);
    GPIO_setAsOutputPin(YELLOW_PORT, YELLOW_PIN);
    GPIO_setAsOutputPin(ORANGE_PORT, ORANGE_PIN);
    GPIO_setAsOutputPin(RED_PORT, RED_PIN);

    //SET AUDIO
    GPIO_setAsOutputPin(AUDIO_PORT, AUDIO_PIN);

    //SET COLUMNS AND ROWS
    GPIO_setAsInputPinWithPullDownResistor(COL_1_PORT,COL_1_PIN);
    GPIO_setAsInputPinWithPullDownResistor(COL_2_PORT,COL_2_PIN);
    GPIO_setAsInputPinWithPullDownResistor(COL_3_PORT,COL_3_PIN);
    GPIO_setAsOutputPin(ROW_1_PORT,ROW_1_PIN);
    GPIO_setAsOutputPin(ROW_2_PORT,ROW_2_PIN);
    GPIO_setAsOutputPin(ROW_3_PORT,ROW_3_PIN);
    GPIO_setAsOutputPin(ROW_4_PORT,ROW_4_PIN);

    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
//    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
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
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN1 + GPIO_PIN2,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

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
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1,
                                               GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0,
                                                GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 6;
    param.firstModReg = 8;
    param.secondModReg = 17;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = 1;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector = USCI_A0_VECTOR
__interrupt void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(
            EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE,
                                  EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
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
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN,
                                                GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

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
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE,
                                               ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

int start_keypad()
{

}
