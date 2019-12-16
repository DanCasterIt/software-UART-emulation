/*
 * File:   main.c
 * Author: daniele
 *
 * Created on 17 giugno 2017, 20.21
 */

// DSPIC33FJ128GP802 Configuration Bit Settings

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config FWDTEN = OFF             // Watchdog Timer Enabled/disabled by user software
// (LPRC can be disabled by clearing SWDTEN bit in RCON register

#include <xc.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <p33FJ128GP802.h>
#include <libpic30.h>

#define FOSC    (80000000)
#define FP (FOSC/2)
#define BAUDRATE 19600
#define BRGVAL ((FP/BAUDRATE)/16)-1

#define PERIOD_AND_HALF 521 //520.8333333
#define PERIOD 347 //347.2222222
#define PRESCALER 0 //0 = 1, 1 = 8, 2 = 64, 3 = 256

void oscillator_setup(void);
void uart_setup(void);
void putch(char c);
void timer1_setup(void);
void change_notification_pin_setup(void);
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void);

unsigned short int i, e, res;
uint16_t rxbuff, buff;
char rxbyte;

int main(void) {
    oscillator_setup();
    uart_setup();
    printf("PIN: RA0\n\r"); // debug
    timer1_setup();
    change_notification_pin_setup();
    AD1PCFGLbits.PCFG0 = 1; // RA0 as digital
    TRISAbits.TRISA0 = 1; // RA0 as digital input
    while (1) {
    }
    return 0;
}

void oscillator_setup() {
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 41; // M = 43
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
}

void uart_setup(void) {
    AD1PCFGL = 0xFFFF; //all pins as digital
    TRISBbits.TRISB3 = 0; // TX as output
    TRISBbits.TRISB2 = 1; // RX as input

    RPINR18bits.U1RXR = 2; //U1RX on RP2 pin
    RPOR1bits.RP3R = 0b00011; //U1TX on RP3 pin

    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 0; // Standard-Speed mode
    U1BRG = BRGVAL; // Baud Rate setting for 9600
    U1MODEbits.UARTEN = 1; // Enable 
    U1STAbits.UTXEN = 1; // Enable UART TX
    __C30_UART = 1;
}

void putch(char c) {
    // wait the end of transmission
    while (U1STAbits.TRMT == 0) {
    };
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    U1TXREG = c; // send the new byte
}

void timer1_setup(void) {
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timermode
    T1CONbits.TCKPS = PRESCALER; // Select 1:1 Prescaler
    TMR1 = 0x0; // Clear timer register
    PR1 = PERIOD_AND_HALF; // Load first period value
    //IPC0bits.T1IP = 0x01;// Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrup
    //T1CONbits.TON = 1; // Start Timer
}

void change_notification_pin_setup(void) {
    CNEN1bits.CN2IE = 1; // Enable CN2 pin for interrupt detection
    IEC1bits.CNIE = 1; // Enable CN interrupts
    IFS1bits.CNIF = 0; // Reset CN interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    buff = PORTAbits.RA0; //get pin value
    PR1 = PERIOD; //load start timer value
    rxbuff = buff << i | rxbuff;
    if (i >= 9) {
        if (i == 9) { //parity check
            for (; e < 9; e++) res = ((rxbuff >> e) & 0b0000000000000001) + res;
            if ((res % 2) != 0) rxbyte = 0; //error detected
            else rxbyte = rxbuff;
        } else { //wait for the overflow
            T1CONbits.TON = 0; // Disable Timer
            PR1 = PERIOD_AND_HALF; // Load first period value
            TMR1 = 0x0; // Clear timer register
            printf("Carattere ricevuto: %c\n\r", rxbyte); //debug
            IEC1bits.CNIE = 1; // Enable CN interrupts
            IFS1bits.CNIF = 0; // Reset CN interrupt
        }
    }
    i++;
}

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void) {
    IFS1bits.CNIF = 0; // Clear CN interrupt
    if (PORTAbits.RA0 == 0) { //check pin value to be low
        T1CONbits.TON = 1; // Start Timer
        i = 0;
        rxbyte = 0;
        buff = 0;
        rxbuff = 0;
        e = 0;
        res = 0;
        IEC1bits.CNIE = 0; // Disable CN interrupts
    }
}