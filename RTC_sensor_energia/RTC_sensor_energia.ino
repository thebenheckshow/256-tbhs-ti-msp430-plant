// Include application, user and local libraries
#include "LCD_Launchpad.h"
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
RTC_DS1307 RTC;

// Defines
#define lightsOn  13
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
int amPM = 0;
long ADCresult = 0;
int tick = 0;
int RXMSB, RXLSB;
int LCDDelay = 10000; //1000;

int menuState = 0;
unsigned long menuTimer = 0;

int startHour = 3;
int endHour = 21;
int lightState = 0;                              //0 = off 1 = on
unsigned long supplementLevel = 500;                      //If light falls below this level during active time, supplement light

void setup() {

  Wire.begin();
  RTC.begin();
   
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
  
  pinMode(lightsOn, OUTPUT);                         // Set LED pins to output
  
  pinMode(S1, INPUT_PULLUP);                     // LED1 & LED2 are normally HIGH, pulled down when active
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  
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
  
}

void loop() {

  if (tick++ >= LCDDelay) {                //Update at set rate   
   tick = 0; 
   drawLCD();              //Do the screen 
  }

  if (menuTimer) {
    
    menuTimer -= 1;
    
    if (menuTimer == 0) {
      menuState = 0;  
    }   
  }
  
  if (lightState and (ADCresult < supplementLevel)) {  //If light time and not bright enough, turn on supplement                       
    digitalWrite(lightsOn, 1);
  }
  else {
    digitalWrite(lightsOn, 0);  
  }

  if (digitalRead(S1) == 0) {
    while(digitalRead(S1) == 0) {
      __delay_cycles(16000);  
    }
    __delay_cycles(16000); 
    startHour += 1;
    if (startHour == endHour) {
      startHour = 0;
    }
    menuState = 1; 
    menuTimer = 100000;
  }
  if (digitalRead(S2) == 0) {
    while(digitalRead(S2) == 0) {
      __delay_cycles(16000); 
    }  
    __delay_cycles(16000); 
    endHour += 1;
    if (endHour == 24) {          //End of day?
      endHour = startHour + 1;    //Can go as low as startHour + 1
    }    
    menuState = 2; 
    menuTimer = 100000; 
  }
  if (digitalRead(S3) == 0) {
    while(digitalRead(S3) == 0) {
      __delay_cycles(16000); 
    }  
    __delay_cycles(16000); 
    
    calibrate();
  }  
}

void drawLCD() {

  switch(menuState) {
  
    case 0:
      drawMonitor();    
    break;
    case 1:              //Show starting hour?
      LCD.clear();
      LCD.print("ST");
      LCD.showSymbol(LCD_SEG_COLON2, 1);
      drawHour(startHour);
      drawAMPM();    
    break;    
    case 2:
      LCD.clear();
      LCD.print("EN");
      LCD.showSymbol(LCD_SEG_COLON2, 1);
      drawHour(endHour);
      drawAMPM();        
    break;    
    
  }  

}

void drawHour(unsigned char whatHour) {

  amPM = 0;
  
  if (whatHour > 11) {         //Do AM/PM and lob off 24 hour time
      amPM = 1;
      if (whatHour > 12) {
         whatHour -= 12; 
      } 
  }
  
  if (whatHour < 10) {         //Single digit hour? Print space first
    if (whatHour == 0) {       //12 AM?
      LCD.print("12");  
    }
    else {
      LCD.print(" ");          //1-9 AM? 
      LCD.print(whatHour);
    }       
  }
  else {
    LCD.print(whatHour);
  }  

}

void drawAMPM() {

  if (amPM) {
      LCD.println("PM");    
  }
  else {
      LCD.println("AM");    
  }  
  
}


void drawMonitor() {

  P1OUT &= ~BIT4;                       // Drive CS pin low to begin SPI communication
  UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
  while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
  RXMSB = UCB0RXBUF;                    // Place byte in RXMSB
  
  UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
  while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
  RXLSB = UCB0RXBUF;                    // Place byte in RXLSB
  P1OUT |= BIT4;                        // Return CS pin high, done with ADS7042 SPI for now   
  
  ADCresult = (RXMSB << 7) | RXLSB;     // Send ADS7042 result via backchannel UART
  
  DateTime now = RTC.now();             //Get time
  
  LCD.clear();
   
  LCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  
  if (ADCresult > 30 ) {
      LCD.showSymbol(LCD_SEG_BAT0, 1);    
  }
  if (ADCresult > 371 ) {
      LCD.showSymbol(LCD_SEG_BAT1, 1);    
  }            
  if (ADCresult > 722) {
      LCD.showSymbol(LCD_SEG_BAT2, 1);    
  }      
  if (ADCresult > 1068) {
      LCD.showSymbol(LCD_SEG_BAT3, 1);    
  }
  if (ADCresult > 1414) {
      LCD.showSymbol(LCD_SEG_BAT4, 1);    
  }            
  if (ADCresult > 1760) {
      LCD.showSymbol(LCD_SEG_BAT5, 1);    
  }  
  
  drawHour(now.hour());
  
  if (now.hour() >= startHour and now.hour() < endHour and lightState == 0) {      //Did we get to the starting hour and light isn't on?
    lightState = 1;  
  }
  if ((now.hour() >= endHour or now.hour() < startHour) and lightState == 1) {      //Did we get to the ending hour and the light isn't off?
    lightState = 0;  
  }  
  
  if (now.second() & 1) {                 //Blink cursor
      LCD.showSymbol(LCD_SEG_COLON2, 1);
  }

  if (now.minute() < 10) {
      LCD.print("0");
  }
  
  LCD.print(now.minute());
  
  drawAMPM();

  if (lightState) {                        //Light helper time?
    LCD.showSymbol(LCD_SEG_CLOCK, 1);      
    if (ADCresult < supplementLevel) {      //Is it too dark? Turn on the light! (or turn off the dark if you're Spider-Man)
      LCD.showSymbol(LCD_SEG_HEART, 1);   
    }
  }

}


void calibrate() {

  //Get 10 samples...

  supplementLevel = 0;                  //Reset this

  LCD.clear();
  LCD.println("START");
  __delay_cycles(16000000);            //Wait for finger to be gone...
 
   //Burn one off:
 
  P1OUT &= ~BIT4;                       // Drive CS pin low to begin SPI communication
  UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
  while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
  RXMSB = UCB0RXBUF;                    // Place byte in RXMSB
  
  UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
  while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
  RXLSB = UCB0RXBUF;                    // Place byte in RXLSB
  P1OUT |= BIT4;                        // Return CS pin high, done with ADS7042 SPI for now   
  
  ADCresult = (RXMSB << 7) | RXLSB;     // Send ADS7042 result via backchannel UART

  //NOW do the averaging

  for (int x = 0 ; x < 10 ; x++) {

    LCD.clear();
    LCD.print("SAMP");
    LCD.println(x + 1);
    
    P1OUT &= ~BIT4;                       // Drive CS pin low to begin SPI communication
    UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
    while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
    RXMSB = UCB0RXBUF;                    // Place byte in RXMSB
    
    UCB0TXBUF = 0x00;                     // Send empty byte to ADS7042
    while(!(UCB0IFG & UCRXIFG));          // Wait for RXBUF to receive byte
    RXLSB = UCB0RXBUF;                    // Place byte in RXLSB
    P1OUT |= BIT4;                        // Return CS pin high, done with ADS7042 SPI for now   
    
    ADCresult = (RXMSB << 7) | RXLSB;     // Send ADS7042 result via backchannel UART
        
    supplementLevel += ADCresult;  
    
    __delay_cycles(5000000);
    
  }

  //And average them:

  supplementLevel /= 10;
  
  supplementLevel -= 100;                //For hysteresis
  
  LCD.clear();
  LCD.println("AVERGE");  
  __delay_cycles(16000000);
  LCD.clear();
  LCD.println(supplementLevel);  
  __delay_cycles(16000000);
  
}



