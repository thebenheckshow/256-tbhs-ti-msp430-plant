/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//
//  Demo code for ADS7042 Ultra-Low Power Data Acquisition BoosterPack.
//  Uses the MSP-EXP430FR4133, go to Tools -> Board to change board type.
//  There are three sampling speeds available: 1, 63, and 125 kSPS.
//  These can be cycled through by either pressing S2 on the LaunchPad to 
//  increase the sampling speed or S1 on the LaunchPad to decrease.  The
//  default operating mode is termed "Power Mode" and displays the total
//  power consumption of the ADS7042 BoosterPack in microWatts.  By
//  pressing S1 on the BoosterPack the mode is changed to "Hex Mode", where
//  the value of the on-board light sensor is displayed in hexadecimal
//  representation up to 0xFFF.  The user can return to Power Mode by
//  pushing any push button available.  The battery segment portion of the
//  LCD displays the relative intensity of light on the sensor at any
//  moment, regardless of operation mode.
//
//  Author :  Ryan Brown
//  Date   :  April 18, 2016
//  Version:  1.00
//  File   :  BOOST-ADS7042_Firmware.ino
//

// Include application, user and local libraries
#include "LCD_Launchpad.h"
#include "msp430.h"
#include <SPI.h>

// Defines
#define LED1 11                                   // BoosterPack LED3
#define LED2 5                                    // BoosterPack LED1
#define LED3 GREEN_LED                            // LaunchPad LED2
#define S1 PUSH1                                  // LaunchPad S1
#define S2 PUSH2                                  // LaunchPad S2
#define S3 18                                     // BoosterPack S1
#define powerPin A9                               // BoosterPack A_IN pin
#define CS 12                                     // BoosterPack CS pin
LCD_LAUNCHPAD LCD;                                // LaunchPad LCD

// Variables
char sps[3] = {'1', ' ', ' '};
char powerString[4];
char hexString[6];
long powerValue = 0;
long ADCresult = 0;
int hexState = 0;
int newHex = 0;
int state = 4;
int lastState = 0;
int tick = 0;
int RXMSB, RXLSB;
int delayCount;
int LCDDelay, i;

// Main function
// Set clock speeds as well as initialize UCB0 (SPI), LCD, ADC, UCA0 (UART), and TA0
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                      // Stop WDT
    
    FRCTL0 = FRCTLPW | NWAITS_1;                   // Set FRAM wait states, necessary for MCLK > 8 MHz

    // Set MCLK = SMCLK = DCOCLKDIV = 16 MHz
    // Set ACLK = REF0CLK = ~32 kHz
    __bis_SR_register(SCG0);                       // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                     // Set REFO as FLL reference source
    CSCTL0 = 0;                                    // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                        // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                           // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                         // DCOCLKDIV = 16MHz
    __delay_cycles(3);  
    __bic_SR_register(SCG0);                       // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));     // FLL locked
    
    CSCTL4 |= SELMS__DCOCLKDIV | SELA__REFOCLK;    // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                   // default DCOCLKDIV as MCLK and SMCLK source
    
    // Initialize SPI for 4 MHz SCLK, used to communicate with the ADS7042
    pinMode(CS, OUTPUT);
    P5SEL0 |= BIT1 | BIT2 | BIT3;
    UCB0CTLW0 |= UCSWRST;                          // **Put state machine in reset**
    UCB0CTLW0 |= UCMST|UCSYNC|UCMSB|UC7BIT;        // 3-pin, 8-bit SPI master
    UCB0CTLW0 |= UCSSEL__SMCLK;                    // MODCLK
    UCB0BR0 = 4;                                   // /2,fBitClock = fBRCLK/(UCBRx+1).
    UCB0BR1 = 0;                                   //
    UCB0CTLW0 &= ~UCSWRST;                         // **Initialize USCI state machine**\

    // Initialize LCD followed by LEDs and push buttons
    LCD.init();                                    // Initialize the LCD
    __delay_cycles(16000000);                      // Wait one second
    pinMode(LED1, OUTPUT);                         // Set LED pins to output
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    digitalWrite(LED1, HIGH);                      // LED1 & LED2 are active LOW, LED3 is active HIGH
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
    pinMode(S1, INPUT_PULLUP);                     // LED1 & LED2 are normally HIGH, pulled down when active
    pinMode(S2, INPUT_PULLUP);
    pinMode(S3, INPUT);                            // LED3 is kept low, pulled up when active
    
    //Added to pull EN_PH high
    pinMode(19, OUTPUT);
    digitalWrite(19, HIGH);
    
    PM5CTL0 &= ~LOCKLPM5;                          // Unlock LPMx.5 configuration
    analogRead(powerPin);                          // Help initialize ADC basics through Energia
    
    // Initialize ADC for power measurement numbers
    ADCCTL0 &= ~ADCENC;                            // Disable ADC conversion
    ADCCTL1 = ADCSSEL_1 | ADCDIV_0 | ADCSHP;       // ACLK source, no clock divider, use sampling timer
    ADCCTL0 = ADCON | ADCSHT_15;                   // Turn on ADC, use 1024 ADCCLK cycle sample-and-hold time
    ADCCTL2 |= ADCRES;                             // 10-bit ADC resolution
    ADCIFG = 0;                                    // Turn off ADC IFG
    ADCIE |= ADCIE0;                               // Enable ADC interrupts
    ADCMCTL0 |= digitalPinToADCIn(powerPin);       // Set ADC pin to memory control register

    // Configure UART pins
    P1SEL0 |= BIT0 | BIT1;

    // Configure UART for backchannel UART data logging
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;
    
    // Baud Rate calculation
    // 16000000/(16*9600) = 104.167
    // Fractional portion = 0.167
    // User's Guide Table 14-4: UCBRSx = 0xD6
    // UCBRFx = int ( (104.167-104)*16) = 2
    UCA0BR0 = 104;                                 // 16000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0xD600 | UCOS16 | UCBRF_2;
  
    UCA0CTLW0 &= ~UCSWRST;                         // Initialize eUSCI
    
    // Run introductory text
    LCD.displayText("ADS");
    __delay_cycles(16000000);
    LCD.displayText("7042");
    __delay_cycles(16000000);
    LCD.displayText("DEMO");
    __delay_cycles(16000000);
    
    // Initialize timer for ADS7042 sampling purposes
    TA0CCTL0 |= CCIE;                             // TACCR0 interrupt enabled
    TA0CCR0 = 2000;                               // 16000000/(8*2000) = 1 kSPS
    TA0CTL = TASSEL__SMCLK | MC__UP | ID_3;       // SMCLK/8, UP mode

    __bis_SR_register(GIE);                       // General interrupts enabled
    // Continuously repeat loop to update LCD based on user & BoosterPack input
    while(1)
    {
        // Push button logic for determining LCD setting
        if(!digitalRead(S1) && state !=4) 
        {
            digitalWrite(LED1, LOW);              // If push button 1 pressed (LaunchPad S1),
            state++;                              // turn on LED1 (BoosterPack LED3) and decrease sample rate
        }
        else digitalWrite(LED1, HIGH);
        
        if(!digitalRead(S2) && state !=2) 
        {
            digitalWrite(LED2, LOW);              // If push button 2 pressed (LaunchPad S2),
            state--;                              // turn on LED2 (BoosterPack LED1) and increase sample rate
        }
        else digitalWrite(LED2, HIGH);
        
        if(!digitalRead(S3))
        {
            digitalWrite(LED3, HIGH);             // If push button 3 pressed (BoosterPack S1),
            hexState ^= 1;                        // turn on LED3 (Launcpad LED2) and change power/hex state
            newHex = 1;
        }
        else digitalWrite(LED3, LOW);
        
        // Determines if the state of the LCD has changed since the last loop
        if(lastState != state || newHex == 1) {
            if(state == 2) {
              sps[0] = '1'; sps[1] = '2'; sps[2] = '5';    // 16000000/(8*16) = 125 kSPS
              TA0CCR0 = 16;
            }
            else if(state == 3) {
              sps[0] = '6'; sps[1] = '3'; sps[2] = ' ';    // 16000000/(8*32) = 63 kSPS
              TA0CCR0 = 32;
            }
            else if(state == 4) {
              sps[0] = '1'; sps[1] = ' '; sps[2] = ' ';    // 16000000/(8*2000) = 1 kSPS
              TA0CCR0 = 2000; 
            }
            
            powerValue = 0;
            tick = 0;
            LCDDelay = state*10;
            
            // Update LCD dependent upon whether we are in power or hex mode
            LCD.clear();
            // Hex mode banner
            if (newHex == 1 && hexState == 1) 
            {
                LCD.displayText("HEX");
                for(delayCount = 0; delayCount < powf(2,state); delayCount++) __delay_cycles(1000000);
                LCD.displayText("CODE");
                for(delayCount = 0; delayCount < powf(2,state); delayCount++) __delay_cycles(1000000);
            }
            // Power mode banner
            else
            {
                hexState = 0;
                LCD.displayText(sps);
                for(delayCount = 0; delayCount < powf(2,state); delayCount++) __delay_cycles(1000000);
                LCD.displayText("KSPS");
                for(delayCount = 0; delayCount < powf(2,state); delayCount++) __delay_cycles(1000000);
            }
            // Reset state, begin displaying selected mode on the LCD
            lastState = state;
            newHex = 0;
            LCD.clear();
            LCD.showChar('u', 4);                                  // Display units in microWatts for power mode
            LCD.showChar('W', 5);                                  // Will be overwritten if in hex mode
            LCD.showSymbol(LCD_SEG_BAT_ENDS, 1);                   // Display LCD battery end bars
        }
        
        tick++;
        
        ADCresult = (RXMSB << 7) | RXLSB;                          // Send ADS7042 result via backchannel UART
        sendASCII((ADCresult >>8) & 0x000F);                       // Send upper nibble
        sendASCII((ADCresult >> 4) & 0x000F);                      // Send middle nibble
        sendASCII(ADCresult & 0x000F);                             // Send lower nibble
        while(!(UCA0IFG&UCTXIFG)); UCA0TXBUF = 10;                 // End line
      
        // Logic necessary for the LCD battery bar
        for(int i = 0; i<3; i++) 
        {
            if(ADCresult > 683+ i*1365) LCD.showSymbol(13+i, 1);
            else LCD.showSymbol(13+i, 0);
        }
        for(int i = 0; i<3; i++) 
        {
            if(ADCresult > i*1365) LCD.showSymbol(19+i, 1);
            else LCD.showSymbol(19+i, 0);
        }
        
        // If hex mode enough time has passed, update the LCD with the ADS7042 result
        if(hexState == 1)
        {
            if(tick >= LCDDelay)
            {
                sprintf(hexString, " 0x%03X", ADCresult);
                LCD.displayText(hexString);
                tick = 0;
            }
        }
        // If in power mode, take a running sum of the ADC readings
        // If enough time has passed, average the readings and update the LCD
        else
        {
            ADCCTL0 |= ADCENC | ADCSC;                                // Sample ADC
            while (ADCCTL1 & ADCBUSY);                                // Wait for sampling to finish
            powerValue = powerValue + ADCMEM0;                        // Add current result to the sum
            ADCCTL0 &= ~(ADCENC);                                     // Manually turn off ADC enable bit
            __delay_cycles(160000);                                   // Delay 10 ms
            
            if(tick >= LCDDelay)
            {
                powerValue = powerValue/LCDDelay;                     // Divide sum by number of samples
                powerValue = (powerValue)*165/1024;                   // Equation for power numbers based on input voltage
                sprintf(powerString, "%u", powerValue);
                LCD.showChar(powerString[0], 0);                      // Update LCD accordingly
                LCD.showChar(powerString[1], 1);
                LCD.showChar(powerString[2], 2);
                LCD.showChar(powerString[3], 3);
                if(powerValue < 1000) LCD.showChar(' ', 3);
                if(powerValue < 100) LCD.showChar(' ', 2);
                if(powerValue < 10) LCD.showChar(' ', 1);
                powerValue = 0;                                       // Restart average and counter
                tick = 0;
            }
        }
    }
}

// Timer A0 interrupt service routine
// This ISR determines when a sample should be taken from the ADS7042
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    P1OUT &= ~BIT4;                       // Drive CS pin low to begin SPI communication
    UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
    while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
    RXMSB = UCB0RXBUF;                    // Place byte in RXMSB
    
    UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
    while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
    RXLSB = UCB0RXBUF;                    // Place byte in RXLSB
    P1OUT |= BIT4;                        // Return CS pin high, done with ADS7042 SPI for now
}

// sendASCII subfunction
// takes a hex value and turns it into its ASCII representation
// then sends the number to the host via backchannel UART
void sendASCII(int hexvalue)
{
  hexvalue &= 0x000F;
  if(hexvalue <= 9) {while(!(UCA0IFG&UCTXIFG)); UCA0TXBUF = hexvalue + 48;}
  else if (hexvalue <= 15) {while(!(UCA0IFG&UCTXIFG)); UCA0TXBUF = hexvalue + 55;} 
}
