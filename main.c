/*===================================CPEG222====================================
 * Program:     Project 4 (Line Follower)
 * Authors:     Raphael Daluz & Mekhai Waples
 * Date:        11/13/24
 * This is a guide that you can use to write your project 4 code
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))

#define TRUE 1
#define FALSE 0
#define SW1 PORTFbits.RF5
#define SW0 PORTFbits.RF3
#define SW6 PORTBbits.RB10
#define SW7 PORTBbits.RB9
#define IR1 PORTCbits.RC13
#define IR2 PORTDbits.RD1
#define IR3 PORTDbits.RD0
#define IR4 PORTCbits.RC14

#define TWOMS ((int)(((((float)(1000000)/(16*5))+0.5)*0.2)*10))/9.8
#define ONEPTFIVEMS ((int)(((((float)(1000000)/(16*5))+0.5)*0.2)*10))/13.3
#define ONEMS ((int)(((((float)(1000000)/(16*5))+0.5)*0.2)*10))/20




// Libraries
#include <string.h>
#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"
#include "ssd.h"
#include "lcd.h"
#include "pmods.h"
#include "mic.h"
#include "utils.h"

/* *****************************************************************************
 End of File
 */

//Variables

//Motors
int lDir; // 0 = STP, 1 = FWD, -1 = REV
int rDir; // 0 = STP, 1 = FWD< -1 = REV

//controls
int robotON = 0; // default to 1 for debugging
int takePasses = 1;
int passes = 0;
int claps = 0;

//Timing
int mSec = 0; // every 100ms this increments
int sec1=0; // then this increments every second
int sec2=-1; // every ten seconds
int sec3=-1; // every hundred seconds
int temp_mSec;
int clapSec; //store when the last clap occurred relative to program time
int program_mSec = 0; //track entire running time of program, not just lap time

//Sensors
int IR[4];

//modes
typedef enum {STP, FWD, REV, FWDRIGHT, FWDLEFT, REVLEFT, REVRIGHT, RIGHT90, LEFT90} eModes;
eModes robotMode = STP;

// Function Declarations (a lot of these are midstage or unused)
void initializePorts(); //initializes ports
void pwmConfig(); //configures pwm for motors
void activateServo(); 
void updateLeftMotors();
void updateRightMotor();
void handleMotors();
void SSD_Algorithm();
void T2Config();
void handleSSD();
void IR_Init();
void updateMode();


int main(void) {
    initializePorts();
    pwmConfig();
    T2Config();
    LCD_WriteStringAtPos("Jackson  Storm", 0, 1);
    
    while (TRUE) {
        //startup process
        if (!robotON) {
            if ((program_mSec - clapSec) >= 10) {
                claps = 0;
            }
            waitforClaps();
        }
        //robot starts automatic process
        else {
            updateMode();
        } 
    }
}




// Initialize ports on board
void initializePorts() {
    MIC_Init();
    IR_Init();
    LED_Init();
    LCD_Init();
    SSD_Init();
    DDPCONbits.JTAGEN = 0; //Allows LED 5,4,1,0 to be usable
    //          //
    // switches //
    //         // 
    TRISBbits.TRISB10 = 1; //SW6
    ANSELBbits.ANSB10 = 0;
    //SW7
    TRISBbits.TRISB9 = 1;
    ANSELBbits.ANSB9 = 0;
    //SW0
    TRISFbits.TRISF3 = 1;
    //SW1
    TRISFbits.TRISF5 = 1;
}

void pwmConfig() {
        
    PR3 = (int)(((((float)(1000000)/(16*5))+0.5)*0.2)*10); //Period Register.
    T3CONbits.TCKPS = 3; //Timer Prescaler 
    T3CONbits.TGATE = 0; // not gated input (the default)
    T3CONbits.TCS = 0; // PBCLK input (the default)
    T3CONbits.ON = 1;  //Turn on Timer
    TMR3 = 0; // Set Timer X to 0
    
    IPC3bits.T3IP = 7;  //    priority
    IPC3bits.T3IS = 3;  //    subpriority
    IFS0bits.T3IF = 0; //    clear interrupt flag
    IEC0bits.T3IE = 1; //    enable interrupt
    
    
    // Configure Output Compare Module 4
    
    OC4CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC4CONbits.OCTSEL = 1;   // Select the timer to use as a clock source
    OC4RS = PR3/13.3 ;//OC4RS is some fraction of the Period
    OC4R = OC4RS;
    OC4CONbits.ON = 1;       // Start the OC4 module
    
    //Do The same for OC5**************************
   
    OC5CONbits.OCM = 6;      // PWM mode on OC5; Fault pin is disabled
    OC5CONbits.OCTSEL = 1;   // Select the timer to use as a clock source
    OC5RS =  PR3/13.3 ;//OC5RS is some fraction of the Period
    OC5R = OC5RS;
    OC5CONbits.ON = 1;       // Start the OC5 module
    
    //
   
   TRISBbits.TRISB8 = 0; //set servo 0 as output
   TRISAbits.TRISA15 = 0; //set servo 1 as output
   ANSELBbits.ANSB8 = 0; //set servo 0 as digital

   RPB8R = 0x0B; // connect Servo 0 to OC5
   RPA15R = 0x0B;// connect Servo 1 to OC4

    //Set up additional timers here if necessary
}

void T2Config(void) {
    T2CONbits.ON = 0;
    T2CONbits.TCKPS = 0b111;
    T2CONbits.TCS = 0;
    //100 ms timer
    PR2 = (10000000 / 2560);
    TMR2 = 0;
    IPC2bits.T2IP = 4;
    IPC2bits.T2IS = 0;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CONbits.ON = 1;
}

void __ISR(_TIMER_2_VECTOR) Timer2ISR(void) {
    IEC0bits.T2IE = 0; // disable interrupt
    
    //SSD time will be handled here
    handleSSD();
    //program time (from before first clap to robot stops)
    program_mSec++;
    
    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; // enable interrupt
}

void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
    IEC0bits.T3IE = 0; // disable interrupt
    
    //
    
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
}

void handleSSD() {
    if (robotON == 0) { //2 claps not yet processed
        mSec = 0;
        sec1 = 0;
        sec2 = -1; 
        sec3 = -1;
        SSD_WriteDigits(mSec, sec1, sec2, sec3, 0, 1, 0, 0);
    }
    else if (robotON == 1 && robotMode != STP) { //2 claps processed and robot has not hit 2nd checkpoint
        SSD_Algorithm();
    }
}

void SSD_Algorithm(void) { //simple algorithm to handle SSD digits flipping and incrementing
    if (mSec > 9) {
        mSec = 0;
        if (sec1 == -1) {
            sec1 += 2;
        }
        else {
            sec1 ++;
        }
    }
    else {
        mSec ++;
    }
    if (sec1 > 9) {
        sec1 = 0;
        if (sec2 == -1) {
            sec2+=2;
        }
        else {
            sec2++;
        }
    }
    else if (sec2 > 9) {
        sec2 = 0;
        if (sec3 == -1) {
            sec3+=2;
        }
        else {
            sec3++;
        }
    }
    SSD_WriteDigits(mSec, sec1, sec2, sec3, 0, 1, 0, 0);
}

void modeHandler(eModes MODE){
    switch(MODE) {
        case (STP):
            OC5RS = ONEPTFIVEMS; // right wheel stops
            OC4RS = ONEPTFIVEMS; // left wheel stops
            break;
        case(FWD):
            OC5RS = ONEMS; // right wheel forward (1ms is fwd for RIGHT))
            OC4RS = TWOMS; // left wheel forward
            break;
        case(REV):
            OC5RS = TWOMS; // right wheel reverse (2ms is rev for RIGHT))
            OC4RS = ONEMS; // left wheel reverse
            break;
        case(FWDRIGHT):
            OC4RS = TWOMS; // left wheel forward
            OC5RS = ONEPTFIVEMS; // right wheel stops
            break;
        case(FWDLEFT):
            OC5RS = ONEMS; // right wheel forward (1ms is fwd for RIGHT))
            OC4RS = ONEPTFIVEMS; // left wheel stops
            break;
        case(REVLEFT):
            OC4RS = ONEPTFIVEMS; // left wheel stops
            OC5RS = TWOMS; // right wheel reverse (2ms is rev for RIGHT))
            break;
        case(REVRIGHT):
            OC5RS = ONEPTFIVEMS; // right wheel stops
            OC4RS = ONEMS; // left wheel reverse
            break;
        
        case(LEFT90):
            OC5RS = ONEMS; //left wheel rev
            OC4RS = ONEMS; //right wheel fwd
            break;
        case(RIGHT90):
            OC5RS = TWOMS; //left wheel fwd
            OC4RS = TWOMS; //right wheel rev
        
    }
}

void IR_Init(void) {
    //initializes IR ports
    TRISCbits.TRISC14 = 1;
    TRISDbits.TRISD0 = 1;
    TRISDbits.TRISD1 = 1;
    TRISCbits.TRISC13 = 1;
}

void updateIR(void) {
    //reinitializes each IR index to IR sensor ports on call time
    IR[0] = IR4;
    IR[1] = IR3;
    IR[2] = IR2;
    IR[3] = IR1;
}

void updateMode(void) {
    //updates mode based on IR sensors
        updateIR();
        if (IR[0] == 0 && IR[1] == 0 && IR[2] == 0 && IR[3] == 0) { //0000 (all black)
            if (sec2 >= 3 && sec1 > 1) {
                robotMode = STP;
            }
            else if (sec2 < 3){
                robotMode = FWD;
            }
        }
        else if (IR[0] == 1 && IR[1] == 0 && IR[2] == 1 && IR[3] == 1) { //1011
            robotMode = RIGHT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 0 && IR[2] == 1 && IR[3] == 0) {//1010
            robotMode = FWD;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 1 && IR[2] == 0 && IR[3] == 1) { //1101
            robotMode = LEFT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 1 && IR[2] == 1 && IR[3] == 1) { //1111
            robotMode = FWD; 
            temp_mSec = mSec;
            
        }
        else if (IR[0] == 0 && IR[1] == 1 && IR[2] == 1 && IR[3] == 1) { //0111
            robotMode = RIGHT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 0 && IR[1] == 0 && IR[2] == 1 && IR[3] == 1) { //0011
            robotMode = RIGHT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 0 && IR[1] == 0 && IR[2] == 0 && IR[3] == 1) { //0001
            robotMode = FWDRIGHT;
            temp_mSec = mSec; 
        }
        else if (IR[0] == 1 && IR[1] == 1 && IR[2] == 1 && IR[3] == 0) { //1110
            robotMode = LEFT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 0 && IR[2] == 0 && IR[3] == 1) { //1001
            robotMode = FWD;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 1 && IR[2] == 0 && IR[3] == 0) { //1100
            robotMode = LEFT90;
            temp_mSec = mSec;
        }
        else if (IR[0] == 1 && IR[1] == 0 && IR[2] == 0 && IR[3] == 0) { //1000
            robotMode = FWDLEFT;
            temp_mSec = mSec;
        }
        modeHandler(robotMode);
    }

void activateRobot(void) { // checks if two claps have been processed
    if (claps >= 2) {
        robotON = 1;
    }
}

void waitforClaps(void) {
    //use microphone to increment claps counter
    if (MIC_Val() > 570) {
        clapSec = program_mSec;
        claps++;
        DelayAprox10Us(10000); //delay present to prevent one clap being registered as multiple claps
    }
    activateRobot(); //check if robot should be activated
}
