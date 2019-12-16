#include <xc.h>
#include <stdio.h>
#include <stdint.h>

#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = 0     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = 1    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = 0      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = 0       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = 0     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = 0       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

//#define PERIOD_AND_HALF 65457 //65456.875
//#define PERIOD 65483 //65482.91666
//#define PRESCALER 4 //2^(PRESCALER+1)

//#define PERIOD_AND_HALF 65533 //65533.37239
//#define PERIOD 65534 //65533.91493
//#define PRESCALER 7 //2^(PRESCALER+1)

//#define PERIOD_AND_HALF 65327 //65326.66666
//#define PERIOD 65396 //65396.11111

#define PERIOD_AND_HALF 65327 //65326.66666
#define PERIOD 65396 //65396.11111
#define PRESCALER 0 //2^(PRESCALER+1)

void OSCILLATOR_Initialize(void);
void TIMER0_Initialize(void);
void UART_Initialize(void);
void putch(char c);

void main(void) {
    OSCILLATOR_Initialize();
    TIMER0_Initialize();
    UART_Initialize(); //debug
    printf("\n\rRESET\n\r"); //debug
    unsigned short int i, e, res;
    char rxbyte;
    uint16_t rxbuff, buff;
    ANSELAbits.ANSA0 = 0; //set pin as digital
    TRISAbits.RA0 = 1; //set pin as digital input
    while (1) {
        if (PORTAbits.RA0 == 0) { //check pin value to be low
            TMR0 = PERIOD_AND_HALF; //load start timer value
            T0CONbits.TMR0ON = 1; //start the timer
            i = 0;
            rxbyte = 0;
            buff = 0;
            rxbuff = 0;
            e = 0;
            res = 0;
            while (i < 9) {
                if (INTCONbits.TMR0IF == 1) { //check if overflow happened
                    TMR0 = PERIOD; //load start timer value
                    INTCONbits.TMR0IF = 0; //clear the overflow
                    buff = PORTAbits.RA0; //get pin value
                    rxbuff = buff << i | rxbuff;
                    i++;
                }
            }
            //parity check
            for (; e < 9; e++) res = ((rxbuff >> e) & 0b0000000000000001) + res;
            if ((res % 2) != 0) rxbyte = 0; //error detected
            else rxbyte = rxbuff;
            while (INTCONbits.TMR0IF == 0); //wait for the overflow
            T0CONbits.TMR0ON = 0; //stop the timer
            INTCONbits.TMR0IF = 0; //clear the overflow
            printf("Carattere ricevuto: %c\n\r", rxbyte); //debug
        }
    }
    return;
}

void OSCILLATOR_Initialize(void) {
    // setup oscillator
    OSCCONbits.SCS = 0b00; // primary clock determined by CONFIG1H
    OSCCONbits.IRCF = 0b111; // 16 MHz internal oscillator
    OSCTUNEbits.PLLEN = 1; // Enable PLL
    while (OSCCONbits.HFIOFS == 0); // busy-wait until high frequency
    // oscillator becomes stable
    // now, using 16MHz + 4xPLL, we have an FOSC of 64 MHz
}

void TIMER0_Initialize(void) {
    //timer setup
    T0CONbits.T0CS = 0; //Fosc/4 as clock source
    T0CONbits.TMR0ON = 0; //stop the timer
    T0CONbits.T08BIT = 0; //16-bit mode
    T0CONbits.PSA = 0; //prescaler assigned
    T0CONbits.T0PS = PRESCALER; //prescaler division factor
}

//void TIMER0_Initialize(void) {
//    //timer setup
//    T0CONbits.T0CS = 0; //Fosc/4 as clock source
//    T0CONbits.TMR0ON = 0; //stop the timer
//    T0CONbits.T08BIT = 0; //16-bit mode
//    T0CONbits.PSA = 1; //prescaler not assigned
//    //T0CONbits.T0PS = PRESCALER; //prescaler division factor
//}

void UART_Initialize(void) {
    // setup UART
    TRISCbits.TRISC6 = 0; // TX as output
    TRISCbits.TRISC7 = 1; // RX as input
    TXSTA1bits.SYNC = 0; // Async operation
    TXSTA1bits.TX9 = 0; // No tx of 9th bit
    TXSTA1bits.TXEN = 1; // Enable transmitter
    RCSTA1bits.RC9 = 0; // No rx of 9th bit
    RCSTA1bits.CREN = 1; // Enable receiver
    RCSTA1bits.SPEN = 1; // Enable serial port
    // Setting for 19200 BPS
    BAUDCON1bits.BRG16 = 0; // Divisor at 8 bit
    TXSTA1bits.BRGH = 0; // No high-speed baudrate
    SPBRG1 = 51; // divisor value for 19200

    PIE1bits.TX1IE = 0; // disable TX hardware interrupt
    PIE1bits.RC1IE = 0; // disable RX hardware interrupt
}

void putch(char c) {
    // wait the end of transmission
    while (TXSTA1bits.TRMT == 0);
    TXREG1 = c; // send the new byte
}