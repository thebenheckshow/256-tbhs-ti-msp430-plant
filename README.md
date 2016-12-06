# 256-tbhs-ti-msp430-plant
MSP430 SUPER PLANT POWER!!!

Ben Heck's TI Launchpad Plant Booster

Ben builds a light compensator to supply indoor plants light when the sun isn’t able to provide enough light for them. He uses a Texas Instrument MSP430 Launchpad Development Board, a TI BOOST-AD7042, a DS1307 real time clock, and a grow light. Win Ben Heck’s Lunch Box Dev Kit: http://bit.ly/2cKuqn5

Visit the Ben Heck page on the element14 community: http://bit.ly/2dkTD6z

Let us know about any projects revolving around energy efficiency: http://bit.ly/2d4z4Mq

Win Ben Heck’s Lunch Box Dev Kit plus BeagleBone Black Board: http://bit.ly/2cKuqn5

Ben gets his hands on a Texas Instrument MSP430 Launchpad Microcontroller Development Board with a TI BOOST ADS7042 booster pack. BOOST-ADS7042 is a high-speed, low powered ADC analog-to-digital converter. Karen suggests making a light array that gives supplemental light to plants only when they need it suggest as in the winter time.




Texas Instrument MSP430 Launchpads have top and bottom headers making it easy to stack many BoosterPacks. With this in mind they add a DS1307 real time clock via the I2C bus so they know what time of day it is. This is important because it gives plants a start time and an end time as well as resting points for receiving light. If the sun is not bright enough then a supplemental light will be activated to help the plant. DS1307 RTC is very common and there are many examples online to get you started. 

The booster pack has a light sensor on it which will allow the device to know how much light is needed. The booster pack includes an LCD and three buttons that you can use to program it. One button is used to set the default amount of light. The other two buttons are used to set the start and end time to cycle through the hours. 
The light box is controlled by an SSR controlled by the MSP430 so the microncontroller can turn on an alternating current device. Ben adds a header for a DS1307 real time clock via the 12C bus. Ben populates the board and wires up to the booster pack to the MSP430. He also handles all the coding to make the real time clock work. 

Ben introduces us to rudimentary electrics on wiring a USA mains plug and using a solid state relay to turn it on and off, and then guides us through using Code Composer Studio and Energia to compile code for the TI MSP430 with boosterpack. The growlight lights up red and blue. Plants are green so they do not care about the color green. He uses a solid state relay because it’s able to control alternating current loads with very low level logic.
